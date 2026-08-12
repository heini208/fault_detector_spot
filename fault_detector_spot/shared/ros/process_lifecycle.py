import os
import signal
import subprocess


def terminate_process_group(
    process,
    interrupt_timeout_sec: float = 3.0,
    terminate_timeout_sec: float = 2.0,
    kill_timeout_sec: float = 1.0,
) -> bool:
    if process is None or process.poll() is not None:
        return True

    stages = (
        (signal.SIGINT, interrupt_timeout_sec),
        (signal.SIGTERM, terminate_timeout_sec),
        (signal.SIGKILL, kill_timeout_sec),
    )

    for sig, timeout_sec in stages:
        if process.poll() is not None:
            return True
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except ProcessLookupError:
            return process.poll() is not None

        try:
            process.wait(timeout=timeout_sec)
            return True
        except subprocess.TimeoutExpired:
            continue

    return process.poll() is not None
