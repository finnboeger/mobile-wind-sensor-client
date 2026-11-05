"""Running the system tools the dashboard reads its state from."""

import shutil
import subprocess  # nosec B404


def run(args: list[str], timeout: float) -> subprocess.CompletedProcess[str]:
    """
    Run a command without a shell and capture its output.

    :raises OSError: if the program isn't installed or can't be started
    :raises subprocess.TimeoutExpired: if it takes longer than ``timeout`` seconds
    """
    executable = shutil.which(args[0])
    if executable is None:
        msg = f"{args[0]} is not installed"
        raise FileNotFoundError(msg)
    return subprocess.run(  # noqa: S603 # nosec B603
        [executable, *args[1:]],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )
