"""
Wi-Fi management through NetworkManager (``nmcli``).

The Wi-Fi interface is either connected to a known network or, if none is in reach,
runs an access point ("hotspot") serving the local dashboard. :class:`WifiManager`
watches the interface every few seconds, also while nobody looks at the dashboard,
and opens the hotspot once no network was connected for a while. The dashboard uses
it to switch between both and to manage the known networks. Changes run one at a
time in a background thread.

The hotspot uses NetworkManager's ``shared`` mode, which runs a DHCP server on
10.42.0.1/24 and forwards traffic to the internet. ``scripts/setup-hotspot.sh``
blocks that forwarding, and the hotspot isn't opened without the block.
"""

import contextlib
import logging
import os
import shutil
import subprocess  # nosec B404
import threading
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from typing import Any

from config import NetworkConfig

from . import commands

logger = logging.getLogger(__name__)

#: name of the NetworkManager connection profile for the access point
HOTSPOT_CONNECTION = "windbot-hotspot"
#: address of the dashboard on the access point
HOTSPOT_ADDRESS = "10.42.0.1"
#: nftables table of scripts/setup-hotspot.sh that blocks forwarding from the hotspot
FORWARDING_BLOCK_TABLE = "windbot_hotspot"
#: AP_PASSWORD of config.default.ini, which is public
DEFAULT_AP_PASSWORD = "WindSensor"  # noqa: S105  # nosec B105
#: NetworkManager device states, see `nmcli device status`
STATE_UNAVAILABLE = 20
STATE_DISCONNECTED = 30
STATE_CONNECTED = 100
#: seconds the list of known networks is cached
NETWORKS_CACHE_SECONDS = 10
#: seconds an activating connection may hold off the hotspot past its deadline
ACTIVATION_GRACE_SECONDS = 60
#: seconds nmcli waits for a connection to come up
CONNECT_TIMEOUT_SECONDS = 30
#: seconds between two checks of the interface in the background
SUPERVISION_INTERVAL_SECONDS = 10
#: seconds the hotspot stays open before the known networks are tried again, as long
#: as no device uses it
KNOWN_NETWORK_RETRY_SECONDS = 900
#: a device on the hotspot counts as using it if it sent something within this many
#: seconds; a phone with the dashboard open polls every few seconds
ACTIVE_CLIENT_SECONDS = 60
#: seconds to wait for the interface to leave hotspot mode before scanning
LEAVE_HOTSPOT_TIMEOUT_SECONDS = 10
#: nice value of the Wi-Fi threads, so they don't slow down the wind processing
BACKGROUND_NICE = 10
#: checks in a row that must miss the forwarding block before the hotspot is closed,
#: so a single failed nft call doesn't drop the connected phones
BLOCK_CHECK_FAILURES = 2
#: seconds the signal strength of the connected network is cached
SIGNAL_CACHE_SECONDS = 3
WIFI_TYPE = "802-11-wireless"
MAX_SSID_LENGTH = 32
MIN_PSK_LENGTH = 8
MAX_PSK_LENGTH = 63
#: range of NetworkManager's connection.autoconnect-priority
MIN_PRIORITY = -999
MAX_PRIORITY = 999


class WifiError(Exception):
    """A NetworkManager command failed."""


class WifiRequestError(WifiError):
    """The request can't be carried out as asked, e.g. a too short password."""


class WifiBusyError(WifiError):
    """Another change of the connection is still running."""


Runner = Callable[[list[str], float], str]


def run_nmcli(args: list[str], timeout: float) -> str:
    """Run nmcli and return its output, raising WifiError on failure."""
    try:
        result = commands.run(["nmcli", *args], timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        raise WifiError(str(e)) from e
    if result.returncode != 0:
        message = (result.stderr or result.stdout).strip()
        raise WifiError(message.removeprefix("Error: "))
    return result.stdout


def split_terse(line: str) -> list[str]:
    """Split a line of ``nmcli -t`` output at unescaped colons."""
    fields: list[str] = []
    current: list[str] = []
    escaped = False
    for char in line:
        if escaped:
            current.append(char)
            escaped = False
        elif char == "\\":
            escaped = True
        elif char == ":":
            fields.append("".join(current))
            current = []
        else:
            current.append(char)
    fields.append("".join(current))
    return fields


def parse_terse(output: str) -> list[list[str]]:
    return [split_terse(line) for line in output.splitlines() if line]


def parse_properties(output: str) -> dict[str, str]:
    """Parse ``nmcli -t -f a,b ... show`` output into a dictionary."""
    properties: dict[str, str] = {}
    for line in output.splitlines():
        key, _, value = line.partition(":")
        if key and key not in properties:
            properties[key] = ":".join(split_terse(value))
    return properties


def lower_priority() -> None:
    """Run the calling thread at a lower CPU priority (Linux has a nice per thread)."""
    with contextlib.suppress(OSError, AttributeError):
        os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), BACKGROUND_NICE)


def validate_ssid(ssid: str) -> str:
    if not 0 < len(ssid.encode()) <= MAX_SSID_LENGTH:
        msg = "The network name needs 1 to 32 characters."
        raise WifiRequestError(msg)
    return ssid


def validate_password(password: str) -> str:
    if password and not MIN_PSK_LENGTH <= len(password) <= MAX_PSK_LENGTH:
        msg = "The password needs 8 to 63 characters, or none for an open network."
        raise WifiRequestError(msg)
    return password


@dataclass
class DeviceState:
    state: int
    connection: str | None
    address: str | None


@dataclass
class WifiStatus:
    #: "client", "hotspot", "searching" or "unavailable"
    mode: str = "unavailable"
    ssid: str | None = None
    address: str | None = None
    signal: int | None = None
    #: monotonic time at which the hotspot will be opened
    hotspot_deadline: float | None = None
    #: description of the action that is currently running
    action: str | None = None
    message: str | None = None
    error: str | None = None
    clients: int | None = None
    networks: list[dict[str, Any]] = field(default_factory=list)


class WifiManager:
    def __init__(
        self,
        network_config: NetworkConfig,
        run: Runner = run_nmcli,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = network_config
        self.interface = network_config.WIFI_INTERFACE
        self._run = run
        self._clock = clock
        self._lock = threading.RLock()
        self._action_lock = threading.Lock()
        self._status = WifiStatus()
        self._had_first_deadline = False
        #: checks in a row that missed the forwarding block while the hotspot was up
        self._block_check_failures = 0
        #: whether the missing forwarding block was logged, so it's logged only once
        self._reported_unblocked = False
        #: monotonic time at which the known networks are tried again from the hotspot
        self._retry_at: float | None = None
        #: devices that used the hotspot recently, see ACTIVE_CLIENT_SECONDS
        self._active_clients: int | None = None
        self._signal_time: float | None = None
        self._networks_time: float | None = None

    # ------------------------------------------------------------------ queries

    def nmcli(self, *args: str, timeout: float = 10) -> str:
        return self._run(list(args), timeout)

    def device_state(self) -> DeviceState | None:
        try:
            output = self.nmcli(
                "-t",
                "-f",
                "GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS",
                "device",
                "show",
                self.interface,
            )
        except WifiError as e:
            logger.debug("Could not read the state of %s: %s", self.interface, e)
            return None
        properties = parse_properties(output)
        try:
            state = int(properties.get("GENERAL.STATE", "0").split(" ")[0])
        except ValueError:
            state = 0
        address = properties.get("IP4.ADDRESS[1]")
        return DeviceState(
            state=state,
            connection=properties.get("GENERAL.CONNECTION") or None,
            address=address.split("/")[0] if address else None,
        )

    def scan(self, *, rescan: bool = True, cached: bool = True) -> list[dict[str, Any]]:
        """
        Return the networks in reach, strongest first, one entry per SSID.

        ``cached``: return NetworkManager's last results if the new scan fails.
        """
        fields = ["-t", "-f", "IN-USE,SSID,SIGNAL,SECURITY", "device", "wifi"]
        args = [*fields, "list", "ifname", self.interface, "--rescan"]
        try:
            output = self.nmcli(*args, "yes" if rescan else "no", timeout=30)
        except WifiError:
            if not rescan or not cached:
                raise
            # scanning isn't always possible while the access point is running
            output = self.nmcli(*args, "no")
        networks: dict[str, dict[str, Any]] = {}
        for row in parse_terse(output):
            if len(row) < 4 or not row[1]:  # noqa: PLR2004
                continue
            in_use, ssid, signal, security = row[:4]
            entry = {
                "ssid": ssid,
                "signal": int(signal) if signal.isdigit() else None,
                "security": security,
                "in_use": in_use == "*",
            }
            existing = networks.get(ssid)
            if existing is None or (entry["signal"] or 0) > (existing["signal"] or 0):
                entry["in_use"] = entry["in_use"] or bool(
                    existing and existing["in_use"],
                )
                networks[ssid] = entry
        return sorted(networks.values(), key=lambda n: -(n["signal"] or 0))

    def known_networks(self) -> list[dict[str, Any]]:
        """Return the saved Wi-Fi networks, without their passwords."""
        output = self.nmcli(
            "-t",
            "-f",
            "NAME,UUID,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY,ACTIVE",
            "connection",
            "show",
        )
        rows = [
            row[:6]
            for row in parse_terse(output)
            if len(row) >= 6  # noqa: PLR2004
            and row[2] == WIFI_TYPE
            and row[0] != HOTSPOT_CONNECTION
        ]
        if not rows:
            return []
        # one call for the SSIDs and modes of all of them
        details = self._wifi_details([row[1] for row in rows])
        networks = []
        for name, uuid, _, autoconnect, priority, active in rows:
            detail = details.get(uuid, {})
            if detail.get("802-11-wireless.mode") == "ap":
                continue
            networks.append(
                {
                    "uuid": uuid,
                    "name": name,
                    "ssid": detail.get("802-11-wireless.ssid") or name,
                    "autoconnect": autoconnect == "yes",
                    "priority": int(priority) if priority.lstrip("-").isdigit() else 0,
                    "active": active == "yes",
                },
            )
        return sorted(networks, key=lambda n: (-n["priority"], n["ssid"].lower()))

    def _wifi_details(self, uuids: list[str]) -> dict[str, dict[str, str]]:
        """Return the Wi-Fi settings of the connections, by UUID."""
        args = [arg for uuid in uuids for arg in ("uuid", uuid)]
        output = self.nmcli(
            "-t",
            "-f",
            "connection.uuid,802-11-wireless.ssid,802-11-wireless.mode",
            "connection",
            "show",
            *args,
        )
        # one block of lines per connection, each starting with its UUID
        details: dict[str, dict[str, str]] = {}
        current: dict[str, str] = {}
        for line in output.splitlines():
            key, _, value = line.partition(":")
            if key == "connection.uuid":
                current = details.setdefault(value, {})
            elif key:
                current[key] = ":".join(split_terse(value))
        return details

    def hotspot_stations(self) -> list[int] | None:
        """Return how long each device on the hotspot has been inactive, in ms."""
        try:
            result = commands.run(["iw", "dev", self.interface, "station", "dump"], 5)
        except (OSError, subprocess.TimeoutExpired):
            return None
        stations: list[int] = []
        for line in result.stdout.splitlines():
            if line.startswith("Station"):
                stations.append(0)  # counts as active if iw leaves the time out
            elif stations and line.strip().startswith("inactive time:"):
                value = line.split(":", 1)[1].split()
                if value and value[0].isdigit():
                    stations[-1] = int(value[0])
        return stations

    def forwarding_blocked(self) -> bool:
        """Return whether setup-hotspot.sh blocks forwarding from the hotspot."""
        try:
            result = commands.run(
                ["nft", "list", "table", "inet", FORWARDING_BLOCK_TABLE],
                5,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return result.returncode == 0

    def status(self) -> dict[str, Any]:
        """Return the current status for the dashboard."""
        with self._lock:
            now = self._clock()
            if (
                self._status.mode != "unavailable"
                and (
                    self._networks_time is None
                    or now - self._networks_time > NETWORKS_CACHE_SECONDS
                )
                and not self._action_lock.locked()
            ):
                try:
                    self._status.networks = self.known_networks()
                    self._networks_time = now
                except WifiError as e:
                    logger.warning("Could not list the known networks: %s", e)
            status = self._status
            # read when asked, so the supervision doesn't run nmcli for it
            if status.mode == "client" and (
                self._signal_time is None
                or now - self._signal_time >= SIGNAL_CACHE_SECONDS
            ):
                status.signal = self._current_signal()
                self._signal_time = now
            deadline = status.hotspot_deadline
            return {
                "mode": status.mode,
                "ssid": status.ssid,
                "address": status.address,
                "signal": status.signal,
                "clients": status.clients,
                "hotspot_in": (
                    max(0, round(deadline - now)) if deadline is not None else None
                ),
                "hotspot_enabled": self.config.ENABLE_HOTSPOT,
                "hotspot_ssid": self.config.AP_SSID,
                "hotspot_address": HOTSPOT_ADDRESS,
                "action": status.action,
                "message": status.message,
                "error": status.error,
                "networks": status.networks,
            }

    # ------------------------------------------------------------- supervision

    def tick(self) -> None:
        """
        Update the status and open the hotspot when its deadline has passed.

        From an unused hotspot the known networks are tried again now and then, e.g.
        for a phone hotspot switched on as a backup when 4G fails.

        An open hotspot is closed if its forwarding block disappeared, e.g. because
        another firewall tool flushed the rules.
        """
        if self._action_lock.locked():
            return
        try:
            self._supervise()
        except WifiBusyError:
            # a dashboard request started a change since the check above
            logger.debug("A Wi-Fi change started meanwhile, skipping this check")

    def _supervise(self) -> None:
        device = self.device_state()
        blocked = True
        with self._lock:
            status = self._status
            self._show_device(device)
            if status.mode != "hotspot":
                self._block_check_failures = 0
                self._retry_at = None
            if device is None or device.state <= STATE_UNAVAILABLE:
                return
            if device.state == STATE_CONNECTED:
                status.hotspot_deadline = None
                self._had_first_deadline = True
                if status.mode == "client":
                    return
                blocked = not self._block_lost()
                if blocked and not self._retry_due(self._active_clients):
                    return
            elif not self._hotspot_due(device) or not self._can_open_hotspot():
                return
        if status.mode == "hotspot":
            self._leave_hotspot(blocked=blocked)
            return
        logger.info("No Wi-Fi network connected, opening the hotspot")

        def action() -> str:
            try:
                self._start_hotspot()
            except WifiError:
                self._restart_deadline()
                raise
            return "No known network connected, so the hotspot was opened."

        self._run_action("Opening the hotspot", action)

    def _show_device(self, device: DeviceState | None) -> None:
        """Update the status from the state of the interface."""
        status = self._status
        if device is None or device.state <= STATE_UNAVAILABLE:
            # STATE_UNAVAILABLE is e.g. the Wi-Fi switched off
            status.mode = "unavailable"
            status.ssid = status.address = status.signal = status.clients = None
            if device is not None:
                # e.g. the Wi-Fi switched off: once it's back, NetworkManager gets its
                # usual time to find a known network. Kept after a failed nmcli, which
                # is usually a brief hiccup.
                status.hotspot_deadline = None
        elif device.state != STATE_CONNECTED:
            status.mode = "searching"
            status.ssid = device.connection
            status.address = status.signal = status.clients = None
        elif device.connection == HOTSPOT_CONNECTION:
            status.mode = "hotspot"
            status.ssid = self.config.AP_SSID
            status.address = device.address
            status.signal = None
            stations = self.hotspot_stations()
            if stations is None:
                status.clients = self._active_clients = None
            else:
                status.clients = len(stations)
                self._active_clients = sum(
                    1 for ms in stations if ms < ACTIVE_CLIENT_SECONDS * 1000
                )
        else:
            status.mode = "client"
            status.ssid = device.connection
            status.address = device.address
            status.clients = None

    def _refresh_status(self) -> None:
        """Show the state right after an action, instead of after the next check."""
        try:
            device = self.device_state()
            with self._lock:
                self._show_device(device)
        except Exception:
            logger.exception("Could not read the Wi-Fi state")

    def _block_lost(self) -> bool:
        """Return whether the open hotspot's forwarding block is missing for good."""
        if self.forwarding_blocked():
            self._block_check_failures = 0
            return False
        self._block_check_failures += 1
        if self._block_check_failures < BLOCK_CHECK_FAILURES:
            logger.warning(
                "Forwarding block of the hotspot not found, checking again before "
                "closing it",
            )
            return False
        self._block_check_failures = 0
        return True

    def _can_open_hotspot(self) -> bool:
        """
        Return whether forwarding from the hotspot is blocked, so it can be opened.

        If not, e.g. on a git install without setup-hotspot.sh, it's logged once and
        checked again after the fallback delay.
        """
        if self.forwarding_blocked():
            self._reported_unblocked = False
            return True
        msg = (
            "The hotspot stays closed because forwarding from it to the internet "
            "isn't blocked. Run sudo scripts/setup-hotspot.sh."
        )
        if not self._reported_unblocked:
            self._reported_unblocked = True
            logger.warning(msg)
        self._status.error = msg
        self._restart_deadline()
        return False

    def _leave_hotspot(self, *, blocked: bool) -> None:
        if not blocked:
            logger.error(
                "Forwarding from the hotspot isn't blocked anymore, closing it",
            )
            self._run_action("Closing the hotspot", self._close_unblocked_hotspot)
        else:
            logger.info("The hotspot is unused, trying the known networks again")
            self._run_action("Looking for known networks", self._retry_known_networks)

    def _close_unblocked_hotspot(self) -> str:
        """Close the hotspot, whose forwarding block disappeared, e.g. by a flush."""
        self._restart_deadline()
        self.nmcli("connection", "down", HOTSPOT_CONNECTION)
        return (
            "The hotspot was closed because forwarding from it to the internet wasn't "
            "blocked anymore. Run sudo scripts/setup-hotspot.sh."
        )

    def _retry_due(self, active_clients: int | None) -> bool:
        """Return whether to try the known networks from the hotspot now."""
        now = self._clock()
        if self._retry_at is None or active_clients != 0:
            # not while a device uses the hotspot (or the count is unknown)
            self._retry_at = now + KNOWN_NETWORK_RETRY_SECONDS
            return False
        if now < self._retry_at:
            return False
        self._retry_at = None
        return True

    def _retry_known_networks(self) -> str | None:
        """
        Close the hotspot for a known network in reach, or open it again.

        Not finding one is the usual outcome, so a failed search is only logged.
        """
        try:
            self.nmcli("connection", "down", HOTSPOT_CONNECTION)
        except WifiError as e:
            logger.info("Could not close the hotspot: %s", e)
        connected = None
        try:
            connected = self._wait_for_hotspot_down()
            if connected is None:
                # NetworkManager doesn't scan while the hotspot runs, so its cached
                # list can be hours old
                connected = self._try_known_networks(cached=False)
        except WifiError as e:
            logger.info("Looking for known networks failed: %s", e)
        finally:
            if connected is None:
                self._start_hotspot()
        return connected

    def _wait_for_hotspot_down(self) -> str | None:
        """
        Wait until the interface left hotspot mode, so a scan works.

        Return a message if NetworkManager joined a known network by itself meanwhile.
        """
        deadline = time.monotonic() + LEAVE_HOTSPOT_TIMEOUT_SECONDS
        while True:
            device = self.device_state()
            if device is not None and device.connection != HOTSPOT_CONNECTION:
                if device.state == STATE_DISCONNECTED:
                    return None
                if device.state == STATE_CONNECTED:
                    return f"Connected to {device.connection}."
            if time.monotonic() >= deadline:
                msg = "The Wi-Fi didn't leave hotspot mode."
                raise WifiError(msg)
            time.sleep(0.5)

    def _hotspot_due(self, device: DeviceState) -> bool:
        """Return whether to open the hotspot, while no network is connected."""
        status = self._status
        if not self.config.ENABLE_HOTSPOT:
            status.hotspot_deadline = None
            return False
        now = self._clock()
        if status.hotspot_deadline is None:
            delay = (
                self.config.AP_FALLBACK_DELAY_SECONDS
                if self._had_first_deadline
                else self.config.AP_STARTUP_DELAY_SECONDS
            )
            self._had_first_deadline = True
            status.hotspot_deadline = now + delay
            logger.info("No Wi-Fi network connected, hotspot opens in %d s", delay)
            return False
        activating = STATE_DISCONNECTED < device.state < STATE_CONNECTED
        return now >= status.hotspot_deadline and not (
            activating and now < status.hotspot_deadline + ACTIVATION_GRACE_SECONDS
        )

    def _current_signal(self) -> int | None:
        try:
            networks = self.scan(rescan=False)
        except WifiError:
            return None
        return next((n["signal"] for n in networks if n["in_use"]), None)

    def run_forever(self, interval: float = SUPERVISION_INTERVAL_SECONDS) -> None:
        lower_priority()
        while True:
            try:
                self.tick()
            except Exception:
                logger.exception("Wi-Fi supervision failed")
            time.sleep(interval)

    # ---------------------------------------------------------------- actions

    def _run_action(
        self,
        description: str,
        action: Callable[[], str | None],
        *,
        locked: bool = False,
    ) -> None:
        """
        Run an action that changes the connection, one at a time.

        ``locked``: the caller already holds the action lock, which is released here.
        """
        if not locked and not self._action_lock.acquire(blocking=False):
            msg = "Another Wi-Fi change is still running."
            raise WifiBusyError(msg)
        try:
            with self._lock:
                self._status.action = description
                self._status.error = None
                self._status.message = None
            try:
                message = action()
            except WifiError as e:
                logger.warning("%s failed: %s", description, e)
                with self._lock:
                    self._status.error = f"{description} failed: {e}"
            except Exception:
                logger.exception("%s failed", description)
                with self._lock:
                    self._status.error = f"{description} failed."
            else:
                with self._lock:
                    self._status.message = message
        finally:
            with self._lock:
                self._status.action = None
                self._networks_time = None
            self._action_lock.release()
            self._refresh_status()

    def _in_background(
        self,
        description: str,
        action: Callable[[], str | None],
    ) -> None:
        # taken here, so a second request at the same moment gets a 409 instead of
        # being told it worked; the thread releases it
        if not self._action_lock.acquire(blocking=False):
            msg = "Another Wi-Fi change is still running."
            raise WifiBusyError(msg)

        def run() -> None:
            lower_priority()
            self._run_action(description, action, locked=True)

        try:
            threading.Thread(target=run, daemon=True).start()
        except BaseException:
            self._action_lock.release()
            raise

    def _ensure_hotspot_connection(self) -> None:
        ssid = self.config.AP_SSID
        # only an empty AP_PASSWORD opens an open hotspot, not a broken one
        password = self.config.AP_PASSWORD or None
        try:
            validate_ssid(ssid)
        except WifiError as e:
            msg = "AP_SSID in config.ini needs 1 to 32 characters."
            raise WifiError(msg) from e
        try:
            validate_password(password or "")
        except WifiError as e:
            msg = "AP_PASSWORD in config.ini needs 8 to 63 characters, or none."
            raise WifiError(msg) from e
        # Recreated only when the settings changed: every change is written to the SD
        # card, and the password is visible in the process list while nmcli runs.
        if self._hotspot_profile_matches(ssid, password):
            return
        try:
            self.nmcli("connection", "delete", HOTSPOT_CONNECTION)
        except WifiError:
            pass  # it didn't exist yet
        args = [
            "connection",
            "add",
            "type",
            "wifi",
            "ifname",
            self.interface,
            "con-name",
            HOTSPOT_CONNECTION,
            "ssid",
            ssid,
            "connection.autoconnect",
            "no",
            "802-11-wireless.mode",
            "ap",
            "802-11-wireless.band",
            "bg",
            "ipv4.method",
            "shared",
            "ipv4.addresses",
            f"{HOTSPOT_ADDRESS}/24",
            "ipv6.method",
            "disabled",
        ]
        if password:
            # WPA2 with CCMP only and without PMF, which the Pi's Wi-Fi chip needs
            args += [
                "wifi-sec.key-mgmt",
                "wpa-psk",
                "wifi-sec.proto",
                "rsn",
                "wifi-sec.pairwise",
                "ccmp",
                "wifi-sec.group",
                "ccmp",
                "wifi-sec.pmf",
                "disable",
                "wifi-sec.psk",
                password,
            ]
        self.nmcli(*args)

    def _hotspot_profile_matches(self, ssid: str, password: str | None) -> bool:
        fields = [
            "connection.interface-name",
            "802-11-wireless.ssid",
            "802-11-wireless-security.key-mgmt",
            "802-11-wireless-security.psk",
        ]
        try:
            output = self.nmcli(
                "--show-secrets",
                "--get-values",
                ",".join(fields),
                "connection",
                "show",
                HOTSPOT_CONNECTION,
            )
        except WifiError:
            return False  # not created yet
        values = [":".join(split_terse(line)) for line in output.splitlines()]
        while values and not values[-1]:
            values.pop()  # an open hotspot has no security values
        expected = [self.interface, ssid, *(["wpa-psk", password] if password else [])]
        return values == expected

    def _start_hotspot(self) -> None:
        if not self.forwarding_blocked():
            msg = (
                "Forwarding from the hotspot isn't blocked, so it would share the "
                "windbot's internet connection. Run sudo scripts/setup-hotspot.sh."
            )
            raise WifiError(msg)
        self._ensure_hotspot_connection()
        self.nmcli(
            "connection",
            "up",
            HOTSPOT_CONNECTION,
            timeout=CONNECT_TIMEOUT_SECONDS + 5,
        )
        with self._lock:
            self._status.hotspot_deadline = None

    def _try_known_networks(self, *, cached: bool = True) -> str | None:
        """
        Connect to the best known network in reach, if any.

        ``cached``: use NetworkManager's last results if a new scan isn't possible.
        """
        known = {n["ssid"]: n for n in self.known_networks()}
        candidates = [
            known[network["ssid"]]
            for network in self.scan(cached=cached)
            if network["ssid"] in known
        ]
        # highest priority first, scan order (signal) breaks ties
        candidates.sort(key=lambda n: -n["priority"])
        for network in candidates:
            try:
                self._connect(network["uuid"])
            except WifiError as e:
                logger.info("Could not connect to %s: %s", network["ssid"], e)
                continue
            return f"Connected to {network['ssid']}."
        return None

    def _connect(self, uuid: str) -> None:
        self.nmcli(
            "--wait",
            str(CONNECT_TIMEOUT_SECONDS),
            "connection",
            "up",
            "uuid",
            uuid,
            timeout=CONNECT_TIMEOUT_SECONDS + 5,
        )

    def connect(self, uuid: str) -> None:
        """Connect to a known network. Reopens the hotspot if that fails."""
        network = next((n for n in self.known_networks() if n["uuid"] == uuid), None)
        if network is None:
            msg = "Unknown network."
            raise WifiRequestError(msg)
        from_hotspot = self._status.mode == "hotspot"

        def action() -> str:
            self._restart_deadline()
            try:
                self._connect(uuid)
            except WifiError as e:
                if from_hotspot and self.config.ENABLE_HOTSPOT:
                    self._start_hotspot()
                msg = f"Could not connect to {network['ssid']}: {e}"
                raise WifiError(msg) from e
            return f"Connected to {network['ssid']}."

        self._in_background(f"Connecting to {network['ssid']}", action)

    def _restart_deadline(self) -> None:
        with self._lock:
            self._status.hotspot_deadline = (
                self._clock() + self.config.AP_FALLBACK_DELAY_SECONDS
            )

    def start_hotspot(self) -> None:
        self._in_background("Opening the hotspot", self._start_hotspot)

    def stop_hotspot(self) -> None:
        """Close the hotspot and look for a known network."""

        def action() -> str:
            self._restart_deadline()
            try:
                self.nmcli("connection", "down", HOTSPOT_CONNECTION)
            except WifiError as e:
                logger.info("Could not close the hotspot: %s", e)
            connected = self._try_known_networks()
            self._restart_deadline()
            if connected is not None:
                return connected
            return (
                "No known network connected. The hotspot opens again in "
                f"{self.config.AP_FALLBACK_DELAY_SECONDS} s."
            )

        self._in_background("Closing the hotspot", action)

    @contextlib.contextmanager
    def _changing_networks(self) -> Iterator[None]:
        """Hold the action lock while a request changes the known networks."""
        if not self._action_lock.acquire(blocking=False):
            msg = "Another Wi-Fi change is still running."
            raise WifiBusyError(msg)
        try:
            yield
        finally:
            with self._lock:
                self._networks_time = None
            self._action_lock.release()

    def save_network(  # noqa: PLR0913
        self,
        ssid: str,
        password: str | None,
        priority: int = 0,
        *,
        open_network: bool = False,
        autoconnect: bool = True,
        hidden: bool = False,
        uuid: str | None = None,
    ) -> None:
        """
        Add a network, or change a known one.

        :param password: new password, None or empty keeps the current one of a known
            network; a new network needs one unless ``open_network``
        :param open_network: save it without a password, also removing a known one
        :param uuid: the known network to change, None to add (or replace) by SSID
        """
        validate_ssid(ssid)
        if not MIN_PRIORITY <= priority <= MAX_PRIORITY:
            msg = f"The priority needs to be from {MIN_PRIORITY} to {MAX_PRIORITY}."
            raise WifiRequestError(msg)
        if open_network or not password:
            password = None
        else:
            validate_password(password)
        with self._changing_networks():
            if uuid is None:
                uuid = next(
                    (n["uuid"] for n in self.known_networks() if n["ssid"] == ssid),
                    None,
                )
            if uuid is None and password is None and not open_network:
                msg = "A new network needs a password, unless it is an open network."
                raise WifiRequestError(msg)
            self._save_network(
                uuid,
                ssid,
                password,
                [
                    "connection.autoconnect",
                    "yes" if autoconnect else "no",
                    "connection.autoconnect-priority",
                    str(priority),
                    "802-11-wireless.hidden",
                    "yes" if hidden else "no",
                ],
                open_network=open_network,
            )

    def _save_network(
        self,
        uuid: str | None,
        ssid: str,
        password: str | None,
        settings: list[str],
        *,
        open_network: bool,
    ) -> None:
        if password:
            settings = [
                *settings,
                "wifi-sec.key-mgmt",
                "wpa-psk",
                "wifi-sec.psk",
                password,
            ]
        if uuid is None:
            self.nmcli(
                "connection",
                "add",
                "type",
                "wifi",
                "ifname",
                self.interface,
                "con-name",
                ssid,
                "ssid",
                ssid,
                *settings,
            )
            return
        if open_network:
            # remove the security settings to make it an open network
            self.nmcli(
                "connection",
                "modify",
                "uuid",
                uuid,
                "remove",
                "802-11-wireless-security",
            )
        self.nmcli("connection", "modify", "uuid", uuid, "ssid", ssid, *settings)

    def delete_network(self, uuid: str) -> None:
        with self._changing_networks():
            if not any(n["uuid"] == uuid for n in self.known_networks()):
                msg = "Unknown network."
                raise WifiRequestError(msg)
            self.nmcli("connection", "delete", "uuid", uuid)


def init(network_config: NetworkConfig) -> WifiManager | None:
    """Start watching the Wi-Fi, or return None if NetworkManager isn't available."""
    if shutil.which("nmcli") is None:
        logger.warning("nmcli not found, Wi-Fi management is disabled")
        return None
    if network_config.AP_PASSWORD == DEFAULT_AP_PASSWORD:
        logger.warning(
            "AP_PASSWORD is the public default, anyone in range can join the "
            "hotspot. Set your own in config.ini.",
        )
    if network_config.ADMIN_PASSWORD is None:
        logger.warning(
            "ADMIN_PASSWORD isn't set, anyone who can reach the dashboard can change "
            "the Wi-Fi",
        )
    manager = WifiManager(network_config)
    threading.Thread(target=manager.run_forever, daemon=True).start()
    return manager
