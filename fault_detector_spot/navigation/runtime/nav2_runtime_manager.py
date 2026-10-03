"""Manage the Nav2 launch process and its configuration."""

import os
from ament_index_python.packages import get_package_share_directory

from fault_detector_spot.shared.ros.runtime_manager import RuntimeManager


class Nav2RuntimeManager(RuntimeManager):
    PROCESS_KEY = "nav2_launch_process"
    RUNTIME_NAME = "Nav2"
    INTERRUPT_TIMEOUT_SEC = 5.0

    def __init__(self, node, blackboard, launch_file="nav2_lidar_launch.py",
                 params_file=None):
        super().__init__(node, blackboard)
        self.launch_file = launch_file
        self.params_file = params_file

    def start(self, map_file=None, extra_args=None):
        args = []
        if self.params_file:
            params_file = self.params_file
            if not os.path.isabs(params_file):
                params_file = os.path.join(
                    get_package_share_directory("fault_detector_spot"),
                    "config", params_file,
                )
            args.append(f"params_file:={params_file}")
        if map_file:
            args.append(f"map:={map_file}")
        args.extend(extra_args or [])
        return self._launch(self.launch_file, args)
