import py_trees
import py_trees_ros
import rclpy
from rclpy.node import Node
from typing import Dict, Optional
import numpy as np

from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from fault_detector_msgs.msg import TagElement, TagElementArray

from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME
from bosdyn.client.math_helpers import SE3Pose, Quat
from synchros2.tf_listener_wrapper import TFListenerWrapper


class HandCameraTagDetection(py_trees.behaviour.Behaviour):
    """
    Detect AprilTags using depth
    Positions are transformed to GRAV_ALIGNED_BODY_FRAME_NAME.
    """

    def __init__(self, name: str = "HandCameraTagDetection"):
        super().__init__(name)
        self.node: Optional[Node] = None
        self.blackboard = self.attach_blackboard_client()

        self.depth_image: Optional[Image] = None
        self.camera_info: Optional[CameraInfo] = None
        self.latest_detections: Optional[AprilTagDetectionArray] = None
        self.tf_listener: Optional[TFListenerWrapper] = None

        self.camera_frame: Optional[str] = None
        self.target_frame: str = GRAV_ALIGNED_BODY_FRAME_NAME

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]

            self.node.create_subscription(
                Image,
                "/depth_registered/hand/image",
                self._depth_cb,
                10
            )
            self.node.create_subscription(
                CameraInfo,
                "/depth_registered/hand/camera_info",
                self._camera_info_cb,
                10
            )
            self.node.create_subscription(
                AprilTagDetectionArray,
                "/detections",
                self._detections_cb,
                10
            )
            self.tf_listener = TFListenerWrapper(self.node)

            self.blackboard.register_key("visible_tags", access=py_trees.common.Access.READ)
            self.logger.info("DetectNewTagsWithDepth node initialized.")
        except KeyError as e:
            self.logger.error(f"Missing required setup argument: {e}")

    def _depth_cb(self, msg: Image):
        self.depth_image = msg
        self.camera_frame = msg.header.frame_id

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def _detections_cb(self, msg: AprilTagDetectionArray):
        self.latest_detections = msg

    def update(self) -> py_trees.common.Status:
        if self.depth_image is None or self.camera_info is None or self.latest_detections is None:
            return py_trees.common.Status.SUCCESS

        visible_tags: Dict[int, TagElement] = getattr(self.blackboard, "visible_tags", {})
        new_elements = []

        for detection in self.latest_detections.detections:
            tag_id = detection.id
            if tag_id in visible_tags:
                continue  # Skip already known tags

            u, v = int(detection.centre.x), int(detection.centre.y)
            depth = self._get_valid_depth(u, v)

            if depth is None:
                self.logger.debug(f"No valid depth for tag {tag_id} at ({u},{v})")
                continue

            x, y = self._unproject_pixel(u, v, depth)
            local_pose = SE3Pose(x, y, depth, Quat())

            tf_msg = self.tf_listener.lookup_a_tform_b(
                frame_a=self.target_frame,
                frame_b=self.camera_frame,
                timeout_sec=2.0
            )
            t = tf_msg.transform.translation
            r = tf_msg.transform.rotation
            world_tf = SE3Pose(t.x, t.y, t.z, Quat(r.w, r.x, r.y, r.z))
            world_pose = world_tf * local_pose

            tag_element = TagElement()
            tag_element.id = tag_id
            tag_element.pose.header.stamp = self.node.get_clock().now().to_msg()
            tag_element.pose.header.frame_id = self.target_frame
            tag_element.pose.pose.position.x = world_pose.x
            tag_element.pose.pose.position.y = world_pose.y
            tag_element.pose.pose.position.z = world_pose.z
            tag_element.pose.pose.orientation.x = world_pose.rot.x
            tag_element.pose.pose.orientation.y = world_pose.rot.y
            tag_element.pose.pose.orientation.z = world_pose.rot.z
            tag_element.pose.pose.orientation.w = world_pose.rot.w

            new_elements.append(tag_element)

        if new_elements:
            self.blackboard.visible_tags.update({e.id: e for e in new_elements})
            self.feedback_message = f"Published new tags: {[e.id for e in new_elements]}"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = "No new tags detected"
        return py_trees.common.Status.SUCCESS

    def _get_valid_depth(self, u: int, v: int, radius: int = 5) -> Optional[float]:
        if self.depth_image is None:
            return None

        arr = np.frombuffer(self.depth_image.data, dtype=np.uint16).reshape(
            (self.depth_image.height, self.depth_image.width)
        )

        h, w = arr.shape
        for r in range(radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    x, y = u + dx, v + dy
                    if 0 <= x < w and 0 <= y < h:
                        val = arr[y, x]
                        if val > 0:
                            return float(val) * 1e-3  # mm to m
        return None

    def _unproject_pixel(self, u: int, v: int, z: float):
        k = self.camera_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y

    def terminate(self, new_status: py_trees.common.Status):
        self.logger.info(f"DetectNewTagsWithDepth terminated with status {new_status}")
