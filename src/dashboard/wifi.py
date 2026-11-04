"""
Wi-Fi status through NetworkManager (``nmcli``).

:class:`WifiManager` reads the state of the Wi-Fi interface when the dashboard asks
for it, so it can show which network it is connected to, the known networks and the
networks in reach. Nothing runs while nobody looks, since every ``nmcli`` costs
noticeable CPU on a Pi Zero.
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

#: NetworkManager device states, see `nmcli device status`
STATE_UNAVAILABLE = 20
STATE_CONNECTED = 100
#: seconds the state of the interface is cached
STATE_CACHE_SECONDS = 3
#: seconds the list of known networks is cached
NETWORKS_CACHE_SECONDS = 10
WIFI_TYPE = "802-11-wireless"


class WifiError(Exception):
    """A NetworkManager command failed."""


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


@dataclass
class DeviceState:
    state: int
    connection: str | None
    address: str | None


@dataclass
class WifiStatus:
    #: "client", "searching" or "unavailable"
    mode: str = "unavailable"
    ssid: str | None = None
    address: str | None = None
    signal: int | None = None
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
            if len(row) >= 6 and row[2] == WIFI_TYPE  # noqa: PLR2004
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

    def status(self) -> dict[str, Any]:
        """Return the current status for the dashboard."""
        with self._lock:
            now = self._clock()
            if (
                self._state_time is None
                or now - self._state_time >= STATE_CACHE_SECONDS
            ):
                self.tick()
            if self._status.mode != "unavailable" and (
                self._networks_time is None
                or now - self._networks_time > NETWORKS_CACHE_SECONDS
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
                status.mode = "client"
                status.ssid = device.connection
                status.address = device.address
                status.signal = self._current_signal()
            else:
                status.mode = "searching"
                status.ssid = device.connection
                status.address = status.signal = None

    def _current_signal(self) -> int | None:
        try:
            networks = self.scan(rescan=False)
        except WifiError:
            return None
        return next((n["signal"] for n in networks if n["in_use"]), None)


def init(network_config: NetworkConfig) -> WifiManager | None:
    """Return the Wi-Fi manager, or None if NetworkManager isn't available."""
    if shutil.which("nmcli") is None:
        logger.warning("nmcli not found, Wi-Fi management is disabled")
        return None
    return WifiManager(network_config)
