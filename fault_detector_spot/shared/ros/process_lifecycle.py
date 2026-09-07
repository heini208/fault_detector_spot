import os
import signal
import time


def is_process_group_running(process) -> bool:
    """Return whether any process remains in an owned process group.

    Callers must create ``process`` with ``start_new_session=True`` so its
    process id is also the id of the process group that it owns.
    """
    if process is None:
        return False

    try:
        os.killpg(process.pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _wait_for_process_group(
    process,
    timeout_sec: float,
    poll_interval_sec: float = 0.05,
) -> bool:
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while is_process_group_running(process):
        process.poll()
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        time.sleep(min(poll_interval_sec, remaining))

    process.poll()
    return True


def terminate_process_group(
    process,
    interrupt_timeout_sec: float = 3.0,
    terminate_timeout_sec: float = 2.0,
    kill_timeout_sec: float = 1.0,
) -> bool:
    if process is None:
        return True

    stages = (
        (signal.SIGINT, interrupt_timeout_sec),
        (signal.SIGTERM, terminate_timeout_sec),
        (signal.SIGKILL, kill_timeout_sec),
    )

    for sig, timeout_sec in stages:
        if not is_process_group_running(process):
            process.poll()
            return True
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            process.poll()
            return True

        if _wait_for_process_group(process, timeout_sec):
            return True

    return not is_process_group_running(process)
