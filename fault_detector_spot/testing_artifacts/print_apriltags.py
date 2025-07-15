import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray

class AprilTagPrinter(Node):
    def __init__(self):
        super().__init__('apriltag_printer')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10
        )
        self.get_logger().info("Listening for AprilTag detections on /tag_detections...")

    def tag_callback(self, msg):
        if not msg.detections:
            self.get_logger().info("No tags detected.")
            return

        for detection in msg.detections:
            tag_id = detection.id[0] if detection.id else "Unknown"
            self.get_logger().info(f"Detected tag ID: {tag_id}")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
