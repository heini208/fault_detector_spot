#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Int32
import synchros2.process as ros_process

class TagToArmGoalNode(Node):
    def __init__(self):
        super().__init__('tag_to_arm_goal_node')

        # --- State + lock for thread safety ---
        self._lock = threading.Lock()
        self.currently_visible_ids = set()
        self.detection_centers = {}

        # --- Subscriptions & threads ---
        self.create_subscription(
            AprilTagDetectionArray, '/detections',
            self.detection_callback, 10
        )

        thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        thread.start()

        self.get_logger().info('Initialized. Showing found/lost and waiting for keyboard input…')

    def tag_found(self, tag_id: int):
        self.get_logger().info(f"apriltag_{tag_id} found")

    def tag_lost(self, tag_id: int):
        self.get_logger().info(f"apriltag_{tag_id} lost")

    def detection_callback(self, msg: AprilTagDetectionArray):
        new_seen = set()
        new_centers = {}

        for det in msg.detections:
            new_seen.add(det.id)
            new_centers[det.id] = (det.centre.x, det.centre.y)

        with self._lock:
            # Found
            for tag_id in new_seen - self.currently_visible_ids:
                self.tag_found(tag_id)
            # Lost
            for tag_id in self.currently_visible_ids - new_seen:
                self.tag_lost(tag_id)

            self.currently_visible_ids = new_seen
            self.detection_centers = new_centers

            # Log all visible tags
            if new_seen:
                tags = ", ".join(str(t) for t in sorted(new_seen))
                self.get_logger().info(f"Currently visible: [{tags}]")
            else:
                self.get_logger().info("Currently visible: []")

    def keyboard_loop(self):
        """Runs in its own thread; never blocks ROS callbacks."""
        while rclpy.ok():
            tag_str = input("Enter tag ID (1–586, or 'q' to quit): ").strip().lower()
            if tag_str == 'q':
                rclpy.shutdown()
                break

            if not tag_str.isdigit():
                print("❌ Invalid input—please enter a number.", flush=True)
                continue

            tag_id = int(tag_str)
            if tag_id not in range(1, 587):
                print("❌ Tag ID out of range. Must be 1–586.", flush=True)
                continue

            with self._lock:
                if tag_id not in self.currently_visible_ids:
                    print(f"apriltag_{tag_id} not found", flush=True)
                    continue
                x, y = self.detection_centers[tag_id]

            print(f"apriltag_{tag_id} center at (x={x:.2f}, y={y:.2f})", flush=True)

            # wait for confirmation
            while True:
                resp = input("Press 'y' to continue: ").strip().lower()
                if resp == 'y':
                    break


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