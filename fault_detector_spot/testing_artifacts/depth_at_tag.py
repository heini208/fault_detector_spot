import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from apriltag_msgs.msg import AprilTagDetectionArray
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import struct


from sensor_msgs.msg import Image
import numpy as np

class DepthAtTagNode(Node):
    def __init__(self):
        super().__init__('depth_at_tag_node')

        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_detection_callback,
            10
        )

        self.create_subscription(
            Image,
            '/depth_registered/hand/image',
            self.depth_image_callback,
            10
        )

        self.depth_image = None

    def depth_image_callback(self, msg):
        self.depth_image = msg

    def tag_detection_callback(self, msg):

        if self.depth_image is None:
            self.get_logger().warn("No depth image available.")
            return
        for detection in msg.detections:
            u = int(detection.centre.x)
            v = int(detection.centre.y)
            depth = self.get_pixel_from_image(self.depth_image, u, v)
            range = 1
            while depth == 0:
                print("range: ", range)
                x = u + range
                y = v
                depth = self.get_pixel_from_image(self.depth_image, x, y)
                if depth == 0:
                    y = v + range
                    x = u
                    depth = self.get_pixel_from_image(self.depth_image, x, y)
                if depth == 0:
                    x = u - range
                    y = v
                    depth = self.get_pixel_from_image(self.depth_image, x, y)
                if depth == 0:
                    x = u
                    y = v - range
                    depth = self.get_pixel_from_image(self.depth_image, x, y)
                range+= 1

            if depth is not None:
                self.get_logger().info(f"Depth at tag center ({u},{v}): {depth}")
            else:
                self.get_logger().warn(f"Could not find valid depth at ({u}, {v})")

    def get_pixel_from_image(self, msg: Image, x: int, y: int) -> int:
        # Convert raw data to numpy array
        dtype = np.uint16  # 16-bit unsigned
        img_array = np.frombuffer(msg.data, dtype=dtype)

        # Reshape the array to 2D (height x width)
        img_array = img_array.reshape((msg.height, msg.width))

        # Access the pixel value at (y, x)
        return img_array[y, x]


def main(args=None):
    rclpy.init(args=args)
    node = DepthAtTagNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
