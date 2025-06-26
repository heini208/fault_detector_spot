import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from synchros2.tf_listener_wrapper import TFListenerWrapper
from bosdyn.client.math_helpers import SE3Pose, Quat
from bosdyn.api import geometry_pb2


class TagTransformer:
    """
    Compute depth and 3D poses for a single AprilTag given its image pixel center.
    """
    def __init__(
        self,
        node: Node,
        camera_info_topic: str,
        depth_image_topic: str,
        camera_frame: str = 'hand_color_image_sensor',
        body_frame: str = GRAV_ALIGNED_BODY_FRAME_NAME,
        target_frame: str = ODOM_FRAME_NAME,
    ):
        self.node = node
        self.lock = threading.Lock()
        self.camera_frame = camera_frame
        self.body_frame = body_frame
        self.target_frame = target_frame
        self.camera_info = None
        self.depth_image = None
        self.tf_listener = TFListenerWrapper(node)
        self._init_subscribers(camera_info_topic, depth_image_topic)

    def _init_subscribers(self, cam_info_topic, depth_topic):
        self.node.create_subscription(
            CameraInfo, cam_info_topic,
            self._camera_info_cb, 10
        )
        self.node.create_subscription(
            Image, depth_topic,
            self._depth_image_cb, 10
        )

    def _camera_info_cb(self, msg: CameraInfo):
        with self.lock:
            self.camera_info = msg

    def _depth_image_cb(self, msg: Image):
        with self.lock:
            self.depth_image = msg

    def get_depth_at_pixel(self, u: int, v: int, search_radius: int = 5) -> float:
        """
        Return depth in meters at pixel (u, v), searching neighborhood if needed.
        """
        depth_mat = self._get_depth_matrix()
        return self._search_depth(depth_mat, u, v, search_radius)

    def pixel_to_camera_pose(self, u: int, v: int) -> SE3Pose:
        """
        Convert pixel (u, v) plus depth into a camera-frame SE3Pose.
        """
        cam_info = self._require_camera_info()
        z = self.get_depth_at_pixel(u, v)
        x, y = self._unproject_pixel(u, v, z, cam_info)
        return self._build_camera_se3(x, y, z)

    def get_pose_stamped(self, u: int, v: int, frame: str = None) -> PoseStamped:
        """
        Return a PoseStamped of the point at pixel (u, v) in the specified frame.
        """
        frame = frame or self.target_frame
        cam_pose = self.pixel_to_camera_pose(u, v)
        tf_cam = self._lookup_transform(frame, self.camera_frame)
        world_cam = self._se3_from_tf(tf_cam)
        world_point = world_cam * cam_pose
        return self._build_pose_stamped(world_point, frame)

    # ------ Internal helpers ------
    def _require_camera_info(self) -> CameraInfo:
        with self.lock:
            if not self.camera_info:
                raise RuntimeError("CameraInfo not received yet")
            return self.camera_info

    def _require_depth_image(self) -> Image:
        with self.lock:
            if self.depth_image is None:
                raise RuntimeError("Depth image not received yet")
            return self.depth_image

    def _get_depth_matrix(self) -> np.ndarray:
        depth_msg = self._require_depth_image()
        buf = np.frombuffer(depth_msg.data, dtype=np.uint16)
        return buf.reshape((depth_msg.height, depth_msg.width))

    def _search_depth(self, mat: np.ndarray, u: int, v: int, radius: int) -> float:
        h, w = mat.shape
        for r in range(radius + 1):
            for du, dv in [(0,0), (r,0), (0,r), (-r,0), (0,-r)]:
                x, y = u + du, v + dv
                if 0 <= x < w and 0 <= y < h:
                    val = mat[y, x]
                    if val != 0:
                        return float(val) * 1e-3
        raise RuntimeError(f"No valid depth around pixel ({u}, {v})")

    def _unproject_pixel(self, u, v, z, cam_info: CameraInfo):
        fx, fy = cam_info.k[0], cam_info.k[4]
        cx, cy = cam_info.k[2], cam_info.k[5]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y

    def _build_camera_se3(self, x, y, z) -> SE3Pose:
        vec = geometry_pb2.Vec3(x=x, y=y, z=z)
        quat = geometry_pb2.Quaternion(w=1, x=0, y=0, z=0)
        return SE3Pose.from_obj(geometry_pb2.SE3Pose(position=vec, rotation=quat))

    def _lookup_transform(self, from_frame: str, to_frame: str):
        return self.tf_listener.lookup_a_tform_b(from_frame, to_frame)

    def _se3_from_tf(self, tf) -> SE3Pose:
        t = tf.transform.translation
        r = tf.transform.rotation
        return SE3Pose(
            t.x, t.y, t.z,
            Quat(r.w, r.x, r.y, r.z)
        )

    def _build_pose_stamped(self, se3: SE3Pose, frame: str) -> PoseStamped:
        ps = PoseStamped()
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.header.frame_id = frame
        ps.pose.position.x = se3.x
        ps.pose.position.y = se3.y
        ps.pose.position.z = se3.z
        ps.pose.orientation.x = se3.rot.x
        ps.pose.orientation.y = se3.rot.y
        ps.pose.orientation.z = se3.rot.z
        ps.pose.orientation.w = se3.rot.w
        return ps
