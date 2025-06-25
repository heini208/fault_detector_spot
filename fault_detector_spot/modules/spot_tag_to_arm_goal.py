#!/usr/bin/env python3
import threading
import re
import signal

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from synchros2.tf_listener_wrapper import TFListenerWrapper
import synchros2.process as ros_process
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from rclpy.duration import Duration
from rclpy.time import Time


# Handle Ctrl+C in the CLI
def _handle_sigint(signum, frame):
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    raise KeyboardInterrupt

signal.signal(signal.SIGINT, _handle_sigint)

class TagToArmGoalNode(Node):
    def __init__(self):
        super().__init__('tag_to_arm_goal_node')

        # State + lock for thread safety
        self._lock = threading.Lock()
        self.currently_visible_ids = set()
        self.tag_positions = {}

        # TF listener
        self.tf = TFListenerWrapper(self)

        # Publisher for arm_goal
        self._goal_pub = self.create_publisher(PoseStamped, 'arm_goal', 10)

        # Periodic TF scan
        self.create_timer(0.2, self._scan_tf)

        # Start keyboard input thread
        thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        thread.start()

        self.get_logger().info('Initialized. Scanning tf tree and waiting for keyboard input...')

    def tag_found(self, tag_id: int):
        self.get_logger().info(f"apriltag_{tag_id} found")

    def tag_lost(self, tag_id: int):
        self.get_logger().info(f"apriltag_{tag_id} lost")

    def _scan_tf(self):
        try:
            yaml = self.tf.buffer.all_frames_as_yaml()
        except Exception as e:
            self.get_logger().warn(f"Couldn’t list TF frames: {e}")
            return

        fid_re = re.compile(r"['\"]?(filtered_fiducial_(\d+))['\"]?:?")
        new_seen = set()
        new_positions = {}

        for line in yaml.splitlines():
            for m in fid_re.finditer(line):
                frame_name, tag_str = m.group(1), m.group(2)
                tag_id = int(tag_str)
                try:
                    if not self.tf.buffer.can_transform(
                            GRAV_ALIGNED_BODY_FRAME_NAME,
                            frame_name,
                            Time(),
                            Duration(seconds=0)
                    ):
                        continue
                    tfst = self.tf.lookup_a_tform_b(
                        GRAV_ALIGNED_BODY_FRAME_NAME,
                        frame_name
                    )
                    t = tfst.transform.translation

                    t = tfst.transform.translation
                except Exception:
                    # Skip if no transform available
                    self.get_logger().info(f"skipping {frame_name}")
                    continue
                new_seen.add(tag_id)
                new_positions[tag_id] = (t.x, t.y, t.z)

        with self._lock:
            # Report found and lost
            for tid in new_seen - self.currently_visible_ids:
                self.tag_found(tid)
            for tid in self.currently_visible_ids - new_seen:
                self.tag_lost(tid)

            self.currently_visible_ids = new_seen
            self.tag_positions = new_positions

            # Log visible tags
            if new_seen:
                tags = ", ".join(str(t) for t in sorted(new_seen))
                self.get_logger().info(f"Currently visible: [{tags}]")
            else:
                self.get_logger().info("Currently visible: []")

    def keyboard_loop(self):
        """Runs in its own thread; never blocks ROS callbacks."""
        while rclpy.ok():
            try:
                tag_str = input("Enter tag ID (or 'q' to quit): ").strip().lower()
            except EOFError:
                break

            if tag_str == 'q':
                rclpy.shutdown()
                break

            if not tag_str.isdigit():
                print("❌ Invalid input—please enter a number.", flush=True)
                continue

            tag_id = int(tag_str)
            with self._lock:
                if tag_id not in self.currently_visible_ids:
                    print(f"apriltag_{tag_id} not found", flush=True)
                    continue
                pos = self.tag_positions.get(tag_id)

            if pos is None:
                print(f"apriltag_{tag_id} position unknown; it may have been lost.", flush=True)
                continue

            x, y, z = pos
            print(f"apriltag_{tag_id} at (x={x:.3f}, y={y:.3f}, z={z:.3f})", flush=True)

            # wait for confirmation
            resp = input("Press 'y' to publish goal: ").strip().lower()
            if resp == 'y':
                goal = PoseStamped()
                goal.header.frame_id = GRAV_ALIGNED_BODY_FRAME_NAME
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z
                self._goal_pub.publish(goal)
                print(f"Published arm goal for apriltag_{tag_id}.", flush=True)

@ros_process.main()
def main():
    node = TagToArmGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
