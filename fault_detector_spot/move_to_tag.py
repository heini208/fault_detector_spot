import threading
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import numpy as np
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
import synchros2.process as ros_process
from synchros2.tf_listener_wrapper import TFListenerWrapper
from bosdyn.client.math_helpers import SE3Pose, Quat
from bosdyn.api import geometry_pb2


class TagToArmGoalNode(Node):
    def __init__(self):
        super().__init__('tag_to_arm_goal_node')

        self.tag_id = 0
        self.z_offset = 0.1  # meters in front of the tag
        self.camera_info = None
        self.depth_image = None
        self.latest_pose = None
        self.grav_aligned_body_frame = GRAV_ALIGNED_BODY_FRAME_NAME
        self.odom_frame = ODOM_FRAME_NAME
        self.hand_camera_frame = "hand_color_image_sensor"

        self.bridge = CvBridge()
        self.tf_listener = TFListenerWrapper(self)

        self.declare_parameter('tag_id', 0)
        self.tag_id = self.get_parameter('tag_id').value

        self.create_subscription(CameraInfo, '/depth_registered/hand/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/depth_registered/hand/image', self.depth_callback, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.detection_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, 'arm_goal', 10)

        # Start the keypress thread
        thread = threading.Thread(target=self.listen_for_input)
        thread.daemon = True
        thread.start()

        self.get_logger().info('Tag to arm goal node initialized. Press "T" to send arm goal.')

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert depth image: {e}")

    def detection_callback(self, msg: AprilTagDetectionArray):
        if self.camera_info is None or self.depth_image is None:
            self.get_logger().warn("Waiting for camera info or depth image.")
            return

        for detection in msg.detections:
            if detection.id != self.tag_id:
                continue

            pixel_center_x = int(detection.centre.x)
            pixel_center_y = int(detection.centre.y)

            if 0 <= pixel_center_y < self.depth_image.shape[0] and 0 <= pixel_center_x < self.depth_image.shape[1]:
                depth_raw = self.depth_image[pixel_center_y, pixel_center_x]
                depth = float(depth_raw) / 1000.0
            else:
                self.get_logger().warn("Tag center out of depth image bounds.")
                return

            fx = self.camera_info.k[0]  # Focal length in x (from CameraInfo)
            fy = self.camera_info.k[4]  # Focal length in y (from CameraInfo)
            cx = self.camera_info.k[2]  # Principal point x (from CameraInfo)
            cy = self.camera_info.k[5]  # Principal point y (from CameraInfo)

            # Convert pixel coordinates to camera frame coordinates
            z = (pixel_center_x - cx) * depth / fx
            y = (pixel_center_y - cy) * depth / fy
            x = depth  # Depth is the X coordinate in the camera frame

            qw = 1
            qx = 0
            qy = 0
            qz = 0
            flat_body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)
            hand_ewrt_flat_body = geometry_pb2.Vec3(x=x, y=y, z=z)
            flat_body_T_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body, rotation=flat_body_Q_hand)

            goal_pose = PoseStamped()
            goal_pose.header.stamp = msg.header.stamp
            goal_pose.header.frame_id = self.odom_frame

            odom_T_flat_body = self.tf_listener.lookup_a_tform_b(self.grav_aligned_body_frame, self.hand_camera_frame)
            odom_T_flat_body_se3 = SE3Pose(
                odom_T_flat_body.transform.translation.x,
                odom_T_flat_body.transform.translation.y,
                odom_T_flat_body.transform.translation.z,
                Quat(
                    odom_T_flat_body.transform.rotation.w,
                    odom_T_flat_body.transform.rotation.x,
                    odom_T_flat_body.transform.rotation.y,
                    odom_T_flat_body.transform.rotation.z,
                ),
            )
            print("just_y:", y, " hand_ewrt_flat_body:", hand_ewrt_flat_body.y, "depth: ", depth)

            odom_T_hand = odom_T_flat_body_se3 * SE3Pose.from_obj(flat_body_T_hand)

            goal_pose = PoseStamped()
            goal_pose.header.stamp = msg.header.stamp
            goal_pose.header.frame_id = self.grav_aligned_body_frame
            goal_pose.pose.position.x = 0.0#float(odom_T_hand.x)
            goal_pose.pose.position.y = float(odom_T_hand.y)
            goal_pose.pose.position.z = float(odom_T_hand.z)
            goal_pose.pose.orientation.x = 1.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 0.0
            # Update latest pose
            self.latest_pose = goal_pose

            # Log the tag's position in the same frame as the arm's position
            self.get_logger().info(
                f"Tag {self.tag_id} detected at x={odom_T_hand.x:.2f}, y={odom_T_hand.y:.2f}, z={odom_T_hand.z:.2f}. Press 'T' to move arm.")

    def listen_for_input(self):
        while rclpy.ok():
            key = self.getch_nonblocking()
            if key:
                if key.lower() == 't' and self.latest_pose is not None:
                    self.publisher.publish(self.latest_pose)
                    self.get_logger().info("Published arm goal.")
                else:
                    self.get_logger().info(f"Ignored key: {key}")

    def getch_nonblocking(self):
        """Reads a single character from stdin if available, non-blocking."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            rlist, _, _ = select.select([fd], [], [], 0)
            if rlist:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


@ros_process.main()
def main() -> None:
    node = TagToArmGoalNode()
    node.get_logger().info("Spinning node. Waiting for AprilTag detections...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt.")
    finally:
        node.get_logger().info("Destroying node and shutting down.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
