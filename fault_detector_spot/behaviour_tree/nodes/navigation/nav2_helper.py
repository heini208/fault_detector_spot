import os
import signal
import subprocess
import time

import py_trees
import rclpy
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler


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

    def publish_initial_pose(self, pose: list[float], max_wait: float = 5.0, republish_count: int = 3,
                             republish_interval: float = 0.5):
        """
        Robustly publish initial pose to /initialpose for AMCL.
        pose: [x, y, theta] in map frame
        max_wait: seconds to wait for subscriber
        republish_count: how many times to republish
        republish_interval: seconds between republish attempts
        """
        if len(pose) != 3:
            raise ValueError("Initial pose must be [x, y, theta]")

        x, y, theta = pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', LATCHED_QOS)

        # Wait until at least one subscriber connects
        start_time = self.node.get_clock().now().nanoseconds / 1e9
        while pub.get_subscription_count() == 0:
            now = self.node.get_clock().now().nanoseconds / 1e9
            if now - start_time > max_wait:
                self.node.get_logger().warn("No /initialpose subscriber detected within timeout, publishing anyway")
                break
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Publish multiple times to ensure receiver gets it
        for _ in range(republish_count):
            msg.header.stamp = self.node.get_clock().now().to_msg()  # refresh timestamp
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(republish_interval)

        self.node.get_logger().info(f"Initial pose published: [x={x:.2f}, y={y:.2f}, theta={theta:.2f}]")

    def start(self, map_file: str = None, extra_args: list = None, initial_pose = [0,0,0]) -> subprocess.Popen:
        """
        Launch Nav2 using the specified launch file and parameters.
        Stops any running instance before starting a new one.
        """
        self.stop()

        args = ["ros2", "launch", "fault_detector_spot", self.launch_file]

        # Pass params file if specified
        if self.params_file:
            args.append(f"params_file:={self.params_file}")

        if map_file:
            args.append(f"map:={map_file}")

        # Extra launch arguments (e.g., use_sim_time)
        if extra_args is None:
            extra_args = []
        if hasattr(self.node, "use_sim_time"):
            extra_args.append(f"use_sim_time:={self.node.use_sim_time}")

        args.extend(extra_args)

        proc = subprocess.Popen(args, preexec_fn=os.setsid)
        self.bb.nav2_launch_process = proc
        self.publish_initial_pose(initial_pose)
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
