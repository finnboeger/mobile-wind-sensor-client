"""
Latest live values of the processing pipeline, for the local dashboard.

The workers publish what they are doing with :func:`update`, the dashboard reads a
copy of everything with :func:`snapshot`. Each section records when it was last
updated, so the dashboard can show how old a value is.
"""

import threading
import time
from typing import Any

_lock = threading.Lock()
_sections: dict[str, dict[str, Any]] = {}


def update(section: str, **values: Any) -> None:  # noqa: ANN401
    """Set values in a section and mark it as updated now."""
    with _lock:
        current = _sections.setdefault(section, {})
        current.update(values)
        current["updated"] = time.time()


def increment(section: str, key: str, amount: int = 1) -> None:
    """Increase a counter in a section without changing its update time."""
    with _lock:
        current = _sections.setdefault(section, {})
        current[key] = current.get(key, 0) + amount


def snapshot() -> dict[str, dict[str, Any]]:
    """Return a copy of all sections."""
    with _lock:
        return {name: dict(values) for name, values in _sections.items()}


def clear() -> None:
    with _lock:
        _sections.clear()
