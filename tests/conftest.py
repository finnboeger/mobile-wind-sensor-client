import os
import tempfile
from pathlib import Path

# The modules under test read config.ini from the working directory on import.
# Point them at a minimal config in a temporary directory instead of the
# developer's own config.ini, with data logging disabled.
_CONFIG = """
[SENSOR]
COMPASS_OFFSET_DEGREES = 0

[LOGGING]
LOG_LEVEL = WARNING
"""


def pytest_configure() -> None:
    directory = Path(tempfile.mkdtemp(prefix="windbot-tests-"))
    (directory / "config.ini").write_text(_CONFIG, encoding="utf-8")
    os.chdir(directory)
