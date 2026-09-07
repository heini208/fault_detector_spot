"""Tests for owned subprocess-group shutdown."""

import os
import signal
import subprocess
import sys

from fault_detector_spot.shared.ros.process_lifecycle import (
    is_process_group_running,
    terminate_process_group,
)


def test_termination_waits_for_descendants_after_launcher_exits():
    child_code = (
        'import signal,time; '
        'signal.signal(signal.SIGINT, signal.SIG_IGN); '
        'time.sleep(60)'
    )
    launcher_code = (
        'import subprocess,sys,time; '
        f'subprocess.Popen([sys.executable, "-c", {child_code!r}]); '
        'print("ready", flush=True); '
        'time.sleep(60)'
    )
    process = subprocess.Popen(
        [sys.executable, '-c', launcher_code],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        start_new_session=True,
    )

    try:
        assert process.stdout.readline().strip() == 'ready'
        assert is_process_group_running(process)

        assert terminate_process_group(
            process,
            interrupt_timeout_sec=0.2,
            terminate_timeout_sec=1.0,
            kill_timeout_sec=1.0,
        )
        assert not is_process_group_running(process)
    finally:
        if is_process_group_running(process):
            os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=2.0)
        process.stdout.close()
