"""
Wi-Fi management through NetworkManager (``nmcli``).

The Wi-Fi interface is either connected to a known network or, if none is in reach,
runs an access point ("hotspot") serving the local dashboard. :class:`WifiManager`
watches the interface and opens the hotspot once no network was connected for a
while. The dashboard uses it to switch between both and to manage the known
networks.

The hotspot uses NetworkManager's ``shared`` mode, which runs a DHCP server on
10.42.0.1/24. ``scripts/setup-hotspot.sh`` stops it from forwarding traffic to the
internet.
"""

import logging
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
#: NetworkManager device states, see `nmcli device status`
STATE_UNAVAILABLE = 20
STATE_DISCONNECTED = 30
STATE_CONNECTED = 100
#: seconds an activating connection may hold off the hotspot past its deadline
ACTIVATION_GRACE_SECONDS = 60
#: seconds nmcli waits for a connection to come up
CONNECT_TIMEOUT_SECONDS = 30
#: seconds the list of known networks is cached
NETWORKS_CACHE_SECONDS = 10
WIFI_TYPE = "802-11-wireless"
MAX_SSID_LENGTH = 32
MIN_PSK_LENGTH = 8
MAX_PSK_LENGTH = 63


class WifiError(Exception):
    """A NetworkManager command failed or its input was invalid."""


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


def validate_ssid(ssid: str) -> str:
    if not 0 < len(ssid.encode()) <= MAX_SSID_LENGTH:
        msg = "The network name needs 1 to 32 characters."
        raise WifiError(msg)
    return ssid


def validate_password(password: str) -> str:
    if password and not MIN_PSK_LENGTH <= len(password) <= MAX_PSK_LENGTH:
        msg = "The password needs 8 to 63 characters, or none for an open network."
        raise WifiError(msg)
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
        networks = []
        for row in parse_terse(output):
            if len(row) < 6 or row[2] != WIFI_TYPE or row[0] == HOTSPOT_CONNECTION:  # noqa: PLR2004
                continue
            name, uuid, _, autoconnect, priority, active = row[:6]
            details = parse_properties(
                self.nmcli(
                    "-t",
                    "-f",
                    "802-11-wireless.ssid,802-11-wireless.mode",
                    "connection",
                    "show",
                    "uuid",
                    uuid,
                ),
            )
            if details.get("802-11-wireless.mode") == "ap":
                continue
            networks.append(
                {
                    "uuid": uuid,
                    "name": name,
                    "ssid": details.get("802-11-wireless.ssid") or name,
                    "autoconnect": autoconnect == "yes",
                    "priority": int(priority) if priority.lstrip("-").isdigit() else 0,
                    "active": active == "yes",
                },
            )
        return sorted(networks, key=lambda n: (-n["priority"], n["ssid"].lower()))

    def hotspot_clients(self) -> int | None:
        try:
            result = commands.run(["iw", "dev", self.interface, "station", "dump"], 5)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return sum(
            1 for line in result.stdout.splitlines() if line.startswith("Station")
        )

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
        """Update the status and open the hotspot when its deadline has passed."""
        if self._action_lock.locked():
            return
        device = self.device_state()
        with self._lock:
            status = self._status
            if device is None or device.state < STATE_UNAVAILABLE:
                status.mode = "unavailable"
                status.ssid = status.address = status.signal = None
                return

            if device.state == STATE_CONNECTED:
                status.ssid = device.connection
                status.address = device.address
                status.hotspot_deadline = None
                self._had_first_deadline = True
                if device.connection == HOTSPOT_CONNECTION:
                    status.mode = "hotspot"
                    status.ssid = self.config.AP_SSID
                    status.signal = None
                    status.clients = self.hotspot_clients()
                else:
                    status.mode = "client"
                    status.clients = None
                    status.signal = self._current_signal()
                return

            status.mode = "searching"
            status.ssid = device.connection
            status.address = status.signal = status.clients = None
            if not self.config.ENABLE_HOTSPOT:
                status.hotspot_deadline = None
                return
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
                return
            activating = STATE_DISCONNECTED < device.state < STATE_CONNECTED
            if now < status.hotspot_deadline or (
                activating and now < status.hotspot_deadline + ACTIVATION_GRACE_SECONDS
            ):
                return
        logger.info("No Wi-Fi network connected, opening the hotspot")

        def action() -> str:
            self._start_hotspot()
            return "No known network connected, so the hotspot was opened."

        self._run_action("Opening the hotspot", action)

    def _current_signal(self) -> int | None:
        try:
            networks = self.scan(rescan=False)
        except WifiError:
            return None
        return next((n["signal"] for n in networks if n["in_use"]), None)

    def run_forever(self, interval: float = 5) -> None:
        while True:
            try:
                self.tick()
            except Exception:
                logger.exception("Wi-Fi supervision failed")
            time.sleep(interval)

    # ---------------------------------------------------------------- actions

    def _run_action(self, description: str, action: Callable[[], str | None]) -> None:
        """Run an action that changes the connection, one at a time."""
        if not self._action_lock.acquire(blocking=False):
            msg = "Another Wi-Fi change is still running."
            raise WifiError(msg)
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
            else:
                with self._lock:
                    self._status.message = message
        finally:
            with self._lock:
                self._status.action = None
                self._networks_time = None
            self._action_lock.release()

    def _in_background(
        self,
        description: str,
        action: Callable[[], str | None],
    ) -> None:
        if self._action_lock.locked():
            msg = "Another Wi-Fi change is still running."
            raise WifiError(msg)
        threading.Thread(
            target=self._run_action,
            args=(description, action),
            daemon=True,
        ).start()

    def _ensure_hotspot_connection(self) -> None:
        ssid = validate_ssid(self.config.AP_SSID)
        password = self.config.AP_PASSWORD
        try:
            validate_password(password or "")
        except WifiError:
            logger.exception("AP_PASSWORD is invalid, opening an open hotspot instead")
            password = None
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

    def _start_hotspot(self) -> None:
        self._ensure_hotspot_connection()
        self.nmcli(
            "connection",
            "up",
            HOTSPOT_CONNECTION,
            timeout=CONNECT_TIMEOUT_SECONDS + 5,
        )
        with self._lock:
            self._status.hotspot_deadline = None

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

    def connect(self, uuid: str) -> None:
        """Connect to a known network. Reopens the hotspot if that fails."""
        network = next((n for n in self.known_networks() if n["uuid"] == uuid), None)
        if network is None:
            msg = "Unknown network."
            raise WifiError(msg)
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

    def save_network(  # noqa: PLR0913
        self,
        ssid: str,
        password: str | None,
        priority: int = 0,
        *,
        autoconnect: bool = True,
        hidden: bool = False,
        uuid: str | None = None,
    ) -> None:
        """
        Add a network, or change a known one.

        :param password: new password, None keeps the current one of a known network
        :param uuid: the known network to change, None to add (or replace) by SSID
        """
        validate_ssid(ssid)
        if password is not None:
            validate_password(password)
        if uuid is None:
            uuid = next(
                (n["uuid"] for n in self.known_networks() if n["ssid"] == ssid),
                None,
            )
        settings = [
            "connection.autoconnect",
            "yes" if autoconnect else "no",
            "connection.autoconnect-priority",
            str(int(priority)),
            "802-11-wireless.hidden",
            "yes" if hidden else "no",
        ]
        if password:
            settings += ["wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", password]
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
        else:
            if password is not None and not password:
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
        with self._lock:
            self._networks_time = None

    def delete_network(self, uuid: str) -> None:
        if not any(n["uuid"] == uuid for n in self.known_networks()):
            msg = "Unknown network."
            raise WifiError(msg)
        self.nmcli("connection", "delete", "uuid", uuid)
        with self._lock:
            self._networks_time = None


def init(network_config: NetworkConfig) -> WifiManager | None:
    """Start watching the Wi-Fi, or return None if NetworkManager isn't available."""
    if shutil.which("nmcli") is None:
        logger.warning("nmcli not found, Wi-Fi management is disabled")
        return None
    manager = WifiManager(network_config)
    threading.Thread(target=manager.run_forever, daemon=True).start()
    return manager
