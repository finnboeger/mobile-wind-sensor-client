"""
Wi-Fi status and changes through NetworkManager (``nmcli``).

:class:`WifiManager` reads the state of the Wi-Fi interface when the dashboard asks
for it, so it can show which network it is connected to, the known networks and the
networks in reach. Nothing runs while nobody looks, since every ``nmcli`` costs
noticeable CPU on a Pi Zero. Changes, e.g. connecting to a known network or opening
an access point ("hotspot") serving the local dashboard, run one at a time in a
background thread.

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
from collections.abc import Callable
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
STATE_CONNECTED = 100
#: seconds the state of the interface is cached
STATE_CACHE_SECONDS = 3
#: seconds the list of known networks is cached
NETWORKS_CACHE_SECONDS = 10
#: seconds nmcli waits for a connection to come up
CONNECT_TIMEOUT_SECONDS = 30
#: nice value of the Wi-Fi threads, so they don't slow down the wind processing
BACKGROUND_NICE = 10
WIFI_TYPE = "802-11-wireless"
MAX_SSID_LENGTH = 32
MIN_PSK_LENGTH = 8
MAX_PSK_LENGTH = 63


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
        self._state_time: float | None = None
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

    def scan(self, *, rescan: bool = True) -> list[dict[str, Any]]:
        """Return the networks in reach, strongest first, one entry per SSID."""
        fields = ["-t", "-f", "IN-USE,SSID,SIGNAL,SECURITY", "device", "wifi"]
        args = [*fields, "list", "ifname", self.interface, "--rescan"]
        try:
            output = self.nmcli(*args, "yes" if rescan else "no", timeout=30)
        except WifiError:
            if not rescan:
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

    def hotspot_clients(self) -> int | None:
        try:
            result = commands.run(["iw", "dev", self.interface, "station", "dump"], 5)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return sum(
            1 for line in result.stdout.splitlines() if line.startswith("Station")
        )

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
                self._state_time is None
                or now - self._state_time >= STATE_CACHE_SECONDS
            ):
                self.tick()
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
            return {
                "mode": status.mode,
                "ssid": status.ssid,
                "address": status.address,
                "signal": status.signal,
                "clients": status.clients,
                "hotspot_ssid": self.config.AP_SSID,
                "hotspot_address": HOTSPOT_ADDRESS,
                "action": status.action,
                "message": status.message,
                "error": status.error,
                "networks": status.networks,
            }

    # ------------------------------------------------------------- supervision

    def tick(self) -> None:
        """Update the status."""
        device = self.device_state()
        with self._lock:
            self._state_time = self._clock()
            status = self._status
            # STATE_UNAVAILABLE is e.g. the Wi-Fi switched off
            if device is None or device.state <= STATE_UNAVAILABLE:
                status.mode = "unavailable"
                status.ssid = status.address = status.signal = None
            elif device.state == STATE_CONNECTED:
                status.address = device.address
                if device.connection == HOTSPOT_CONNECTION:
                    status.mode = "hotspot"
                    status.ssid = self.config.AP_SSID
                    status.signal = None
                    status.clients = self.hotspot_clients()
                else:
                    status.mode = "client"
                    status.ssid = device.connection
                    status.signal = self._current_signal()
                    status.clients = None
            else:
                status.mode = "searching"
                status.ssid = device.connection
                status.address = status.signal = status.clients = None

    def _current_signal(self) -> int | None:
        try:
            networks = self.scan(rescan=False)
        except WifiError:
            return None
        return next((n["signal"] for n in networks if n["in_use"]), None)

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
                self._state_time = self._networks_time = None
            self._action_lock.release()

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

    def _try_known_networks(self) -> str | None:
        """Connect to the best known network in reach, if any."""
        known = {n["ssid"]: n for n in self.known_networks()}
        candidates = [
            known[network["ssid"]]
            for network in self.scan()
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
        """Connect to a known network."""
        network = next((n for n in self.known_networks() if n["uuid"] == uuid), None)
        if network is None:
            msg = "Unknown network."
            raise WifiRequestError(msg)

        def action() -> str:
            try:
                self._connect(uuid)
            except WifiError as e:
                msg = f"Could not connect to {network['ssid']}: {e}"
                raise WifiError(msg) from e
            return f"Connected to {network['ssid']}."

        self._in_background(f"Connecting to {network['ssid']}", action)

    def start_hotspot(self) -> None:
        self._in_background("Opening the hotspot", self._start_hotspot)

    def stop_hotspot(self) -> None:
        """Close the hotspot for a known network in reach, or open it again."""

        def action() -> str:
            try:
                self.nmcli("connection", "down", HOTSPOT_CONNECTION)
            except WifiError as e:
                logger.info("Could not close the hotspot: %s", e)
            connected = None
            try:
                connected = self._try_known_networks()
            finally:
                if connected is None:
                    self._start_hotspot()
            return (
                connected or "No known network connected, so the hotspot is open again."
            )

        self._in_background("Closing the hotspot", action)


def init(network_config: NetworkConfig) -> WifiManager | None:
    """Return the Wi-Fi manager, or None if NetworkManager isn't available."""
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
    return WifiManager(network_config)
