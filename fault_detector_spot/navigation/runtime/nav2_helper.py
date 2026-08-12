import os
import subprocess
import time

import py_trees
from ament_index_python.packages import get_package_share_directory

from fault_detector_spot.shared.ros.process_lifecycle import (
    terminate_process_group,
)


class Nav2Helper:
    """Helper for managing Nav2 lifecycle and nested launch processes."""

    def __init__(
        self,
        node,
        blackboard,
        launch_file="nav2_sim_launch.py",
        params_file=None,
    ):
        self.node = node
        self.bb = blackboard
        self.launch_file = launch_file
        self.params_file = params_file

        self.bb.register_key(
            "nav2_launch_process",
            access=py_trees.common.Access.WRITE,
        )
        if not self.bb.exists("nav2_launch_process"):
            self.bb.nav2_launch_process = None

    def set_launch_file(self, launch_file: str):
        self.launch_file = launch_file

    def set_params_file(self, params_file: str):
        self.params_file = params_file

    def _use_sim_time(self) -> bool:
        try:
            return bool(
                self.node.get_parameter("use_sim_time").value
            )
        except Exception:
            return False

    def _use_sim_time_launch_arg(self) -> str:
        return "true" if self._use_sim_time() else "false"

    def start(
        self,
        map_file: str = None,
        extra_args: list = None,
    ) -> subprocess.Popen:
        args = [
            "ros2",
            "launch",
            "fault_detector_spot",
            self.launch_file,
        ]

        if self.params_file:
            params_file = self.params_file
            if not os.path.isabs(params_file):
                params_file = os.path.join(
                    get_package_share_directory(
                        "fault_detector_spot"
                    ),
                    "config",
                    params_file,
                )
            args.append(f"params_file:={params_file}")

        if map_file:
            args.append(f"map:={map_file}")

        launch_args = list(extra_args or [])
        if not any(
            item.startswith("use_sim_time:=")
            for item in launch_args
        ):
            launch_args.append(
                "use_sim_time:="
                f"{self._use_sim_time_launch_arg()}"
            )
        args.extend(launch_args)

        proc = subprocess.Popen(args, preexec_fn=os.setsid)
        self.bb.nav2_launch_process = proc
        self.node.get_logger().info(
            f"[Nav2Helper] Started Nav2 with PID {proc.pid}"
        )
        return proc

    def stop(self) -> bool:
        proc = getattr(
            self.bb,
            "nav2_launch_process",
            None,
        )
        if proc is None:
            return True

        if not terminate_process_group(
            proc,
            interrupt_timeout_sec=5.0,
            terminate_timeout_sec=2.0,
            kill_timeout_sec=1.0,
        ):
            self.node.get_logger().error(
                "[Nav2Helper] Nav2 process did not terminate"
            )
            return False

        self.bb.nav2_launch_process = None
        self.node.get_logger().info("[Nav2Helper] Stopped Nav2")
        return True

    def is_running(self) -> bool:
        proc = getattr(
            self.bb,
            "nav2_launch_process",
            None,
        )
        if proc is None:
            return False
        return proc.poll() is None

    def wait_until_active(self, timeout_sec=10):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.is_running():
                return True
            time.sleep(0.2)
        return False
