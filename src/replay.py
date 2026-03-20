"""
Replay previously recorded data logs into the processing pipeline.

This tool is designed to make it easy to analyze and improve the wind
calculation logic without collecting live data.

Controls (TUI):
- Space: play/pause
- n: step a single message (only while paused)
- +/-: decrease/increase playback speed
- q: quit
"""

from __future__ import annotations

import argparse
import curses
import logging
import math
import textwrap
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from queue import Queue
from typing import Any, Final

import jsonpickle
import n2k
from pyubx2 import UBXMessage

import wind
from config import Config
from log import DATA_SEPARATOR
from structs import (
    ApparentWindData,
    CorrectedApparentWindData,
    HeadingData,
    PositionData,
    TrueWindData,
    WindOutputQueue,
)

EXPECTED_LINE_PARTS: Final[int] = 2
MIN_BOX_HEIGHT: Final[int] = 3
MIN_BOX_WIDTH: Final[int] = 4
MIN_TITLE_WIDTH: Final[int] = 6
UI_REFRESH_MS: Final[int] = 50
MAX_SPEED: Final[float] = 64.0
MIN_SPEED: Final[float] = 0.1
VISIBLE_MESSAGES: Final[int] = 10
GPS_MAX_LINES_PER_MESSAGE: Final[int] = 4


@dataclass(frozen=True, kw_only=True)
class ReplayEvent:
    log_time: float
    consumer: str
    payload: object
    source: str


def _discover_default_log_paths() -> list[Path]:
    cfg = Config()

    if cfg.LOGGING.DATA_LOG_DIR:
        log_dir = Path(cfg.LOGGING.DATA_LOG_DIR)
        if log_dir.exists() and log_dir.is_dir():
            return sorted(p for p in log_dir.glob("*.log*") if p.is_file())
        return []

    return []


def _expand_input_paths(paths: list[str]) -> list[Path]:
    expanded: list[Path] = []
    for raw in paths:
        p = Path(raw)
        if p.is_dir():
            expanded.extend(
                sorted(child for child in p.glob("*.log*") if child.is_file()),
            )
        else:
            expanded.append(p)
    # de-dup while keeping order
    unique: list[Path] = []
    seen: set[Path] = set()
    for p in expanded:
        resolved = p.resolve() if p.exists() else p
        if resolved in seen:
            continue
        seen.add(resolved)
        unique.append(p)
    return unique


class UBXMessageHandler(jsonpickle.handlers.BaseHandler):
    def flatten(self, obj: UBXMessage, data: dict[str, Any]) -> dict[str, Any]:
        # Preserve standard jsonpickle flattening behavior
        data.update(self.context.flatten(obj.__dict__, reset=False))
        return data

    def restore(self, obj: dict[str, Any]) -> UBXMessage:
        instance = UBXMessage.__new__(UBXMessage)

        object.__setattr__(instance, "_immutable", False)

        # jsonpickle puts attributes in 'py/state' if __getstate__ is used,
        # otherwise directly in the obj dict
        state = obj.get("py/state", obj)

        for key, value in state.items():
            if key not in (
                # This is a special key used by jsonpickle to track the class
                "py/object",
                # We have already extracted this data above
                "py/state",
                # We only want to enable immutability after all attributes have been set
                "_immutable",
            ):
                # Properly deserialize any nested objects using the context
                restored_value = self.context.restore(value, reset=False)

                # Because _immutable is False, standard setattr works safely here
                setattr(instance, key, restored_value)

        # 5. Lock the object to restore immutability after deserialization
        object.__setattr__(instance, "_immutable", True)

        return instance


def _parse_line(line: str, source: str) -> ReplayEvent | None:
    stripped = line.strip()
    if len(stripped) == 0:
        return None

    parts = stripped.split(DATA_SEPARATOR, 1)
    if len(parts) != EXPECTED_LINE_PARTS:
        return None

    raw_time, raw_payload = parts
    try:
        log_time = float(raw_time)
    except ValueError:
        return None

    try:
        payload = jsonpickle.decode(raw_payload)  # noqa: S301 # nosec
    except Exception:  # noqa: BLE001
        # Corrupted line or incompatible payload; skip.
        return None

    return ReplayEvent(
        log_time=log_time,
        consumer=source.split("/")[-1].split(".")[0],
        payload=payload,
        source=source,
    )


def load_events(files: list[Path]) -> list[ReplayEvent]:
    UBXMessageHandler.handles(UBXMessage)
    events: list[ReplayEvent] = []
    for file_path in files:
        if not file_path.exists() or not file_path.is_file():
            continue
        try:
            with file_path.open("r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    evt = _parse_line(line, source=str(file_path))
                    if evt is not None:
                        events.append(evt)
        except OSError:
            continue

    events.sort(key=lambda e: e.log_time)
    return events


def _fmt_angle(rad: float | None) -> str:
    if rad is None:
        return "-"
    deg = (math.degrees(rad) % 360.0 + 360.0) % 360.0
    return f"{deg:6.1f}°"


def _fmt_knots_from_mps(mps: float | None) -> str:
    if mps is None:
        return "-"
    return f"{n2k.utils.meters_per_second_to_knots(mps):5.2f} kt"


def _fmt_mps(mps: float | None) -> str:
    if mps is None:
        return "-"
    return f"{mps:5.2f} m/s"


def _fmt_knots(knots: float | None) -> str:
    if knots is None:
        return "-"
    return f"{knots:5.2f} kt"


def _shortest_angle_diff_deg(a_deg: float, b_deg: float) -> float:
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True, kw_only=True)
class Rect:
    y: int
    x: int
    h: int
    w: int


def _draw_box(stdscr: curses.window, rect: Rect, title: str) -> None:
    if rect.h < MIN_BOX_HEIGHT or rect.w < MIN_BOX_WIDTH:
        return

    stdscr.addstr(rect.y, rect.x, "+" + "-" * (rect.w - 2) + "+")
    for i in range(1, rect.h - 1):
        stdscr.addstr(rect.y + i, rect.x, "|")
        stdscr.addstr(rect.y + i, rect.x + rect.w - 1, "|")
    stdscr.addstr(
        rect.y + rect.h - 1,
        rect.x,
        "+" + "-" * (rect.w - 2) + "+",
    )
    if title and rect.w > MIN_TITLE_WIDTH:
        trimmed = title[: rect.w - 4]
        stdscr.addstr(rect.y, rect.x + 2, trimmed)


def _draw_lines(stdscr: curses.window, rect: Rect, lines: list[str]) -> None:
    if rect.h <= 0 or rect.w <= 0:
        return
    max_lines = max(0, rect.h)
    for i in range(min(len(lines), max_lines)):
        stdscr.addnstr(rect.y + i, rect.x, lines[i], rect.w)


def _wrap_with_prefix(prefix: str, text: str, *, width: int) -> list[str]:
    if width <= 0:
        return []

    if len(prefix) >= width:
        return [prefix[:width]]

    available = max(1, width - len(prefix))
    wrapped = textwrap.wrap(
        text,
        width=available,
        break_long_words=True,
        break_on_hyphens=False,
        drop_whitespace=False,
    )
    if not wrapped:
        return [prefix]

    continuation = " " * len(prefix)
    return [prefix + wrapped[0], *[continuation + part for part in wrapped[1:]]]


def run_tui(  # noqa: C901, PLR0912, PLR0915
    stdscr: curses.window,
    *,
    events: list[ReplayEvent],
    speed: float,
    tail: int,
    start_paused: bool,
) -> None:
    # Reduce logging noise during curses operation.
    logging.getLogger().setLevel(logging.CRITICAL)

    curses.start_color()
    curses.use_default_colors()
    curses.curs_set(0)
    stdscr.nodelay(True)  # noqa: FBT003
    stdscr.timeout(UI_REFRESH_MS)

    position_queue: Queue[PositionData] = Queue()
    heading_queue: Queue[HeadingData] = Queue()
    wind_queue: Queue[ApparentWindData] = Queue()
    calc_out_queue: WindOutputQueue = Queue()
    # TODO: add mqtt to output queue as an option (via cli flag?).
    # Same for LoraWAN etc. later

    threading.Thread(
        target=wind.worker,
        args=(position_queue, heading_queue, wind_queue, [calc_out_queue]),
        daemon=True,
    ).start()

    input_positions: deque[tuple[float, PositionData]] = deque(maxlen=tail)
    input_headings: deque[tuple[float, HeadingData]] = deque(maxlen=tail)
    input_winds: deque[tuple[float, ApparentWindData]] = deque(maxlen=tail)
    input_gps: deque[tuple[float, UBXMessage]] = deque(maxlen=tail)

    logged_true_wind: deque[tuple[float, TrueWindData]] = deque(maxlen=tail)
    logged_corr_app: deque[tuple[float, CorrectedApparentWindData]] = deque(maxlen=tail)
    calc_true_wind: deque[tuple[float, TrueWindData]] = deque(maxlen=tail)
    calc_corr_app: deque[tuple[float, CorrectedApparentWindData]] = deque(maxlen=tail)

    if not events:
        stdscr.addstr(0, 0, "No events loaded.")
        stdscr.refresh()
        time.sleep(1.0)
        return

    idx = 0
    replay_time = events[0].log_time
    last_real = time.time()
    paused = start_paused

    def process_event(evt: ReplayEvent) -> None:
        nonlocal replay_time
        replay_time = max(replay_time, evt.log_time)
        if evt.consumer == "position" and isinstance(evt.payload, PositionData):
            input_positions.append((evt.log_time, evt.payload))
            position_queue.put(evt.payload)
            # TODO: also add to mqtt queue if enabled
        elif evt.consumer == "heading" and isinstance(evt.payload, HeadingData):
            input_headings.append((evt.log_time, evt.payload))
            heading_queue.put(evt.payload)
        elif evt.consumer == "wind" and isinstance(evt.payload, ApparentWindData):
            input_winds.append((evt.log_time, evt.payload))
            wind_queue.put(evt.payload)
        elif evt.consumer == "gps" and isinstance(evt.payload, UBXMessage):
            input_gps.append((evt.log_time, evt.payload))
        elif evt.consumer == "true_wind" and isinstance(evt.payload, TrueWindData):
            logged_true_wind.append((evt.log_time, evt.payload))
        elif evt.consumer == "apparent_wind_corrected" and isinstance(
            evt.payload,
            CorrectedApparentWindData,
        ):
            logged_corr_app.append((evt.log_time, evt.payload))

    def drain_calc_outputs(now_replay_time: float) -> None:
        while not calc_out_queue.empty():
            corr, tw = calc_out_queue.get()
            calc_corr_app.append((now_replay_time, corr))
            calc_true_wind.append((now_replay_time, tw))

    while True:
        ch = stdscr.getch()
        if ch != -1:
            if ch in (ord("q"), ord("Q")):
                return
            if ch == ord(" "):
                paused = not paused
                last_real = time.time()
            if ch in (ord("+"), ord("=")):
                speed = min(MAX_SPEED, speed * 2.0)
            if ch == ord("-"):
                speed = max(MIN_SPEED, speed / 2.0)
            if ch in (ord("n"), ord("N")) and paused and idx < len(events):
                process_event(events[idx])
                idx += 1
                drain_calc_outputs(replay_time)

        if not paused:
            now_real = time.time()
            replay_time += (now_real - last_real) * speed
            last_real = now_real

            while idx < len(events) and events[idx].log_time <= replay_time:
                process_event(events[idx])
                idx += 1
            drain_calc_outputs(replay_time)
        else:
            drain_calc_outputs(replay_time)

        # Draw
        stdscr.erase()
        max_y, max_x = stdscr.getmaxyx()

        status = (
            f"events {idx}/{len(events)}  "
            f"t={replay_time - events[0].log_time:8.1f}s  "
            f"speed={speed:4.1f}x  "
            f"{'PAUSED' if paused else 'PLAY'}  "
            "[space]=play/pause  n=step  +/-=speed  q=quit"
        )
        stdscr.addnstr(0, 0, status, max_x - 1)

        # Layout
        top = 1
        bottom_box_h = 11
        top_box_h = max(0, max_y - 2 - bottom_box_h)
        body_w = max(0, max_x - 2)
        left_x = 0

        top_rect = Rect(y=top, x=left_x, h=top_box_h, w=body_w)
        bottom_rect = Rect(y=top + top_box_h, x=left_x, h=bottom_box_h, w=body_w)

        _draw_box(stdscr, top_rect, "Recent Inputs")
        _draw_box(stdscr, bottom_rect, "Outputs (logged vs recalculated)")

        # Inputs
        lines: list[str] = []
        lines.append("Position (last)")
        for t, p in list(input_positions)[-min(tail, VISIBLE_MESSAGES) :][::-1]:
            cog = p.true_course if p.true_course is not None else "-"
            lat = p.latitude if p.latitude is not None else "-"
            lon = p.longitude if p.longitude is not None else "-"
            line = f"  {t - events[0].log_time:7.1f}s  "
            line += f"lat={lat:>10}  lon={lon:>11}  "
            speed_kts = n2k.utils.meters_per_second_to_knots(p.speed)
            line += f"sog={_fmt_knots(speed_kts)}  cog={cog}"
            lines.append(line)

        lines.append("")
        lines.append("Heading (last)")
        for t, h in list(input_headings)[-min(tail, VISIBLE_MESSAGES) :][::-1]:
            lines.append(
                f"  {t - events[0].log_time:7.1f}s  hdg={_fmt_angle(h.heading)}",
            )

        lines.append("")
        lines.append("Apparent wind (last)")
        for t, w_evt in list(input_winds)[-min(tail, VISIBLE_MESSAGES) :][::-1]:
            aws_mps = _fmt_mps(w_evt.wind_speed)
            aws_kt = _fmt_knots_from_mps(w_evt.wind_speed)
            line = f"  {t - events[0].log_time:7.1f}s  "
            line += f"aws={aws_mps} ({aws_kt})  "
            line += f"awa={_fmt_angle(w_evt.wind_angle)}"
            lines.append(line)

        lines.append("")
        lines.append(f"Raw GPS messages buffered: {len(input_gps)}")
        gps_rect_w = max(0, body_w - 2)
        for t, gps_msg in list(input_gps)[-min(tail, VISIBLE_MESSAGES) :][::-1]:
            prefix = f"  {t - events[0].log_time:7.1f}s  "
            wrapped_lines = _wrap_with_prefix(prefix, str(gps_msg), width=gps_rect_w)
            if GPS_MAX_LINES_PER_MESSAGE == 0:
                lines.extend(wrapped_lines)
            else:
                lines.extend(wrapped_lines[:GPS_MAX_LINES_PER_MESSAGE])

        _draw_lines(
            stdscr,
            Rect(y=top + 1, x=left_x + 1, h=top_box_h - 2, w=body_w - 2),
            lines,
        )

        # Outputs
        out_lines: list[str] = []

        def fmt_wind_line(prefix: str, tw: TrueWindData | None) -> str:
            if tw is None:
                return f"{prefix}: -"
            tws_mps = _fmt_mps(tw.wind_speed)
            tws_kt = _fmt_knots_from_mps(tw.wind_speed)
            twd = _fmt_angle(tw.wind_angle)
            return f"{prefix}: tws={tws_mps} ({tws_kt})  twd={twd}"

        latest_logged_tw = logged_true_wind[-1][1] if logged_true_wind else None
        latest_calc_tw = calc_true_wind[-1][1] if calc_true_wind else None

        out_lines.append("True wind")
        out_lines.append(fmt_wind_line("  logged", latest_logged_tw))
        out_lines.append(fmt_wind_line("  calc  ", latest_calc_tw))

        if latest_logged_tw is not None and latest_calc_tw is not None:
            logged_deg = (
                math.degrees(latest_logged_tw.wind_angle) % 360.0 + 360.0
            ) % 360.0
            calc_deg = (math.degrees(latest_calc_tw.wind_angle) % 360.0 + 360.0) % 360.0
            d_deg = _shortest_angle_diff_deg(calc_deg, logged_deg)
            d_spd = latest_calc_tw.wind_speed - latest_logged_tw.wind_speed
            out_lines.append(f"  diff  : Δtwd={d_deg:+6.1f}°  Δtws={d_spd:+6.2f} m/s")

        out_lines.append("")
        out_lines.append("Corrected apparent wind")
        latest_logged_ca = logged_corr_app[-1][1] if logged_corr_app else None
        latest_calc_ca = calc_corr_app[-1][1] if calc_corr_app else None
        if latest_logged_ca is not None:
            aws_mps = _fmt_mps(latest_logged_ca.wind_speed)
            aws_kt = _fmt_knots_from_mps(latest_logged_ca.wind_speed)
            out_lines.append(
                f"  logged: aws={aws_mps} ({aws_kt})  "
                f"awd={_fmt_angle(latest_logged_ca.wind_angle)}",
            )
        else:
            out_lines.append("  logged: -")
        if latest_calc_ca is not None:
            aws_mps = _fmt_mps(latest_calc_ca.wind_speed)
            aws_kt = _fmt_knots_from_mps(latest_calc_ca.wind_speed)
            out_lines.append(
                f"  calc  : aws={aws_mps} ({aws_kt})  "
                f"awd={_fmt_angle(latest_calc_ca.wind_angle)}",
            )
        else:
            out_lines.append("  calc  : -")

        _draw_lines(
            stdscr,
            Rect(y=top + top_box_h + 1, x=left_x + 1, h=bottom_box_h - 2, w=body_w - 2),
            out_lines,
        )

        stdscr.refresh()

        # Stop automatically once we're done (stay paused on last frame).
        if idx >= len(events) and not paused:
            paused = True


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay recorded data logs (TUI).")
    parser.add_argument(
        "paths",
        nargs="*",
        help=(
            "Log file(s) or a directory containing per-consumer logs. "
            "If omitted, uses LOGGING.DATA_LOG_DIR or LOGGING.DATA_LOG_FILE from "
            "config.ini."
        ),
    )
    parser.add_argument(
        "-s",
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed (1.0 = realtime).",
    )
    parser.add_argument(
        "--tail",
        type=int,
        default=10,
        help="How many recent datapoints to keep for display.",
    )
    parser.add_argument(
        "--paused",
        action="store_true",
        help="Start paused.",
    )

    args = parser.parse_args()

    if args.paths:
        files = _expand_input_paths(list(args.paths))
    else:
        files = _discover_default_log_paths()

    events = load_events(files)

    if not events:
        joined = ", ".join(str(p) for p in files) if files else "(none)"
        msg = f"No events loaded from: {joined}"
        raise SystemExit(msg)

    curses.wrapper(
        run_tui,
        events=events,
        speed=float(args.speed),
        tail=int(args.tail),
        start_paused=bool(args.paused),
    )


if __name__ == "__main__":
    main()
