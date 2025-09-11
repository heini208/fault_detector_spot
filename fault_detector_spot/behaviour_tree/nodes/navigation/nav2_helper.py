import os
import signal
import subprocess
import time

import py_trees
from geometry_msgs.msg import PoseWithCovarianceStamped


class Nav2Helper:
    """
    Helper for managing Nav2 (Navigation2) lifecycle.
    Handles starting/stopping Nav2 with custom launch files and parameters.
    Tracks the Nav2 process on the blackboard.
    """

    def __init__(self, node, blackboard, launch_file="nav2_sim_launch.py", params_file=None):
        self.node = node
        self.bb = blackboard
        self.launch_file = launch_file
        self.params_file = params_file
        self.amcl_pose = None

        # Register blackboard key for nav2 process
        self.bb.register_key("nav2_launch_process", access=py_trees.common.Access.WRITE)
        if not self.bb.exists("nav2_launch_process"):
            self.bb.nav2_launch_process = None

    def set_launch_file(self, launch_file: str):
        """
        Set the launch file to use for starting Nav2.
        """
        self.launch_file = launch_file

    def set_params_file(self, params_file: str):
        """
        Set the parameters file to use for starting Nav2.
        """
        self.params_file = params_file

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.amcl_pose = msg

    def start(self, map_file: str = None, initial_pose=[0, 0, 0], extra_args: list = None, ) -> subprocess.Popen:
        """
        Launch Nav2 using the specified launch file and parameters.
        """
        if initial_pose is None:
            initial_pose = [0, 0, 0]
        x, y, theta = initial_pose

        args = ["ros2", "launch", "fault_detector_spot", self.launch_file]

        # Pass params file if specified
        if self.params_file:
            args.append(f"params_file:={self.params_file}")

        if map_file:
            args.append(f"map:={map_file}")

        args.append(f"initial_pose_x:={x}")
        args.append(f"initial_pose_y:={y}")
        args.append(f"initial_pose_z:=0.0")  # fixed z, can be parameterized if needed
        args.append(f"initial_pose_theta:={theta}")
        args.append(f"set_initial_pose:=True")

        # Extra launch arguments (e.g., use_sim_time)
        if extra_args is None:
            extra_args = []
        if hasattr(self.node, "use_sim_time"):
            extra_args.append(f"use_sim_time:={self.node.use_sim_time}")

        args.extend(extra_args)

        proc = subprocess.Popen(args, preexec_fn=os.setsid)
        self.bb.nav2_launch_process = proc
        self.node.get_logger().info(f"[Nav2Helper] Started Nav2 with PID {proc.pid}")
        return proc

    def stop(self):
        """
        Stop the Nav2 process if it is running.
        """
        proc = getattr(self.bb, "nav2_launch_process", None)
        if proc:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=5)
                self.node.get_logger().info("[Nav2Helper] Stopped Nav2")
            except Exception as e:
                self.node.get_logger().warn(f"[Nav2Helper] Failed to stop Nav2: {e}")
            finally:
                self.bb.nav2_launch_process = None

    def is_running(self) -> bool:
        """
        Return True if Nav2 process is running.
        """
        proc = getattr(self.bb, "nav2_launch_process", None)
        if proc is None:
            return False
        return proc.poll() is None

    def wait_until_active(self, timeout_sec=10):
        """
        Wait until Nav2 process is running and ready (approximate).
        Returns True if ready within timeout, else False.
        """
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.is_running():
                return True
            time.sleep(0.2)
        return False