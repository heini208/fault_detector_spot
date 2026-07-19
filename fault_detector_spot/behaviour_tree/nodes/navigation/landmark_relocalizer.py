import math
import typing

import numpy as np

import py_trees
import rclpy
import tf2_ros
from fault_detector_spot.behaviour_tree.nodes.mapping.rtab_helper import RTABHelper
from fault_detector_spot.inspection.transform_utils import (
    pose_data_to_pose,
)
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.time import Time
from collections import defaultdict, deque


class LandmarkRelocalizer(py_trees.behaviour.Behaviour):
    """Behaviour that uses AprilTag landmarks (from a JSON map) to publish `/initialpose` for AMCL.

    Behaviour logic:
    - Load landmarks for the active map into a mapping: tag_id (int) -> geometry_msgs/Pose
    - Keep track of which tags have already triggered a relocalization (self.seen_tags).
    - On each update, determine currently visible tag ids from blackboard.visible_tags_map_frame.
      * For tags that are visible now but not in self.seen_tags: compute corrected robot pose and publish `/initialpose`.
      * For tags that were in self.seen_tags but are no longer visible: remove them so they can trigger again later.

    The computation uses TF to obtain the current (observed) pose of the tag in the map frame
    (available in `visible_tags_map_frame`) and the current robot pose in the map frame
    (via tf lookup of base_link in the map frame). It computes the corrective transform that moves
    the observed tag pose to the true tag pose and applies that same corrective transform to the
    robot's observed pose to get the corrected robot pose.
    """

    def __init__(self, slam_helper: RTABHelper, node=None, base_frame: str = "base_link", name="LandmarkRelocalizer"):
        super().__init__(name)
        self.slam_helper = slam_helper
        self.node = node
        self.blackboard = self.attach_blackboard_client()

        # landmark_map: int tag_id -> geometry_msgs.msg.Pose (pose in map frame from JSON)
        self.landmark_map: typing.Dict[int, PoseStamped] = {}

        # config
        self.base_frame = base_frame
        self.map_frame = "map"
        # how long to wait for TF lookups
        self.tf_timeout = Duration(seconds=0.25)

        self.position_error_threshold = 0.60
        self.yaw_error_threshold = math.radians(20.0)

        self.required_samples = 5
        self.maximum_position_spread = 0.20
        self.maximum_yaw_spread = math.radians(10.0)

        self.cooldown_sec = 15.0
        self.last_relocalization_ns = 0

        self.candidate_samples = defaultdict(
            lambda: deque(maxlen=self.required_samples)
        )

        self.loaded_map_name = None

    def setup(self, **kwargs: typing.Any) -> None:
        self.node = kwargs.get("node", self.node)
        self.blackboard.register_key("visible_tags", access=py_trees.common.Access.READ)
        self.blackboard.register_key("visible_tags_map_frame", access=py_trees.common.Access.READ)
        # expose active_map_name from the slam helper via blackboard if available
        self.blackboard.register_key("active_map_name", access=py_trees.common.Access.READ)
        self.initialpose_pub = self.node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    # ---- Helper math (2D) ----
    def _quat_to_yaw(self, qx, qy, qz, qw):
        # Try to use a standard conversion; keep a direct formula fallback
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))  (for quaternion x,y,z,w)
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _pose_to_mat(self, pose: typing.Any) -> np.ndarray:
        """Convert a geometry_msgs/Pose-like object to a 3x3 2D homogeneous matrix (x,y,yaw)."""
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        c = math.cos(yaw)
        s = math.sin(yaw)
        mat = np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])
        return mat

    def _mat_to_pose(self, mat: np.ndarray, pose_out: typing.Any) -> None:
        """Fill pose_out (geometry_msgs/Pose) from 3x3 homogeneous matrix (2D)."""
        x = float(mat[0, 2])
        y = float(mat[1, 2])
        # extract yaw
        yaw = math.atan2(mat[1, 0], mat[0, 0])
        # convert yaw to quaternion (x,y,z,w)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose_out.position.x = x
        pose_out.position.y = y
        pose_out.position.z = 0.0
        pose_out.orientation.x = 0.0
        pose_out.orientation.y = 0.0
        pose_out.orientation.z = qz
        pose_out.orientation.w = qw

    # ---- Landmark loading/parsing ----
    def _load_landmarks_for_map(self, map_name: str) -> None:
        """Load strict localization landmarks for the active map."""
        self.landmark_map.clear()

        try:
            definition = self.slam_helper.map_repository.load(
                map_name
            )
        except (FileNotFoundError, OSError, ValueError) as exception:
            self.feedback_message = (
                f"Failed to load landmarks: {exception}"
            )
            return

        for landmark in definition.localization_landmarks:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose = pose_data_to_pose(landmark.pose_map)
            self.landmark_map[
                landmark.reference_tag.tag_id
            ] = pose

        if not self.landmark_map:
            self.feedback_message = "No localization landmarks found"
            return
        self.feedback_message = (
            f"Loaded {len(self.landmark_map)} localization landmarks"
        )

    # ---- equality helper for published pose (2D) ----
    def poses_equal(self, p1, p2, tol=1e-3):
        dx = abs(p1.position.x - p2.position.x)
        dy = abs(p1.position.y - p2.position.y)
        # compare yaw
        yaw1 = self._quat_to_yaw(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w)
        yaw2 = self._quat_to_yaw(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w)
        dyaw = abs(math.atan2(math.sin(yaw1 - yaw2), math.cos(yaw1 - yaw2)))
        return dx < tol and dy < tol and dyaw < (tol * 10)

    def update(self):
        if not self.slam_helper.is_rtabmap_running():
            self.candidate_samples.clear()
            self.feedback_message = "RTAB-Map not running"
            return py_trees.common.Status.SUCCESS

        map_name = getattr(
            self.blackboard,
            "active_map_name",
            None
        )

        if not map_name:
            self.candidate_samples.clear()
            self.feedback_message = "No active map"
            return py_trees.common.Status.SUCCESS

        if self.loaded_map_name != map_name:
            self._load_landmarks_for_map(map_name)
            self.loaded_map_name = map_name
            self.candidate_samples.clear()

        if not self.landmark_map:
            self.feedback_message = "No landmarks for active map"
            return py_trees.common.Status.SUCCESS

        if (
                not self.blackboard.exists("visible_tags") or
                not isinstance(self.blackboard.visible_tags, dict) or
                not self.blackboard.visible_tags
        ):
            self.candidate_samples.clear()
            self.feedback_message = "No fresh visible tags"
            return py_trees.common.Status.SUCCESS

        now_ns = self.node.get_clock().now().nanoseconds

        if (
                now_ns - self.last_relocalization_ns <
                int(self.cooldown_sec * 1_000_000_000)
        ):
            self.feedback_message = "Relocalization cooldown"
            return py_trees.common.Status.SUCCESS

        visible_tags = self.blackboard.visible_tags

        available_ids = [
            int(tag_id)
            for tag_id in visible_tags.keys()
            if int(tag_id) in self.landmark_map
        ]

        if not available_ids:
            self.feedback_message = "No registered landmark visible"
            return py_trees.common.Status.SUCCESS

        tag_id = min(
            available_ids,
            key=lambda current_id: math.sqrt(
                visible_tags[current_id].pose.pose.position.x ** 2 +
                visible_tags[current_id].pose.pose.position.y ** 2 +
                visible_tags[current_id].pose.pose.position.z ** 2
            )
        )

        tag_element = visible_tags[tag_id]

        tag_observed = PoseStamped()
        tag_observed.header = tag_element.pose.header
        tag_observed.pose = tag_element.pose.pose

        try:
            candidate_matrix, observation_stamp = (
                self._calculate_candidate_pose(
                    self.landmark_map[tag_id],
                    tag_observed
                )
            )
        except Exception as exception:
            self.feedback_message = (
                f"Tag pose calculation failed: {exception}"
            )
            return py_trees.common.Status.SUCCESS

        try:
            current_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time.from_msg(observation_stamp),
                self.tf_timeout
            )
        except Exception:
            try:
                current_transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    Time(),
                    self.tf_timeout
                )
            except Exception as exception:
                self.feedback_message = (
                    f"Current robot pose unavailable: {exception}"
                )
                return py_trees.common.Status.SUCCESS

        current_matrix = self._transform_to_mat4(
            current_transform
        )

        candidate_x = float(candidate_matrix[0, 3])
        candidate_y = float(candidate_matrix[1, 3])
        candidate_yaw = self._matrix_yaw(candidate_matrix)

        current_x = float(current_matrix[0, 3])
        current_y = float(current_matrix[1, 3])
        current_yaw = self._matrix_yaw(current_matrix)

        position_error = math.hypot(
            candidate_x - current_x,
            candidate_y - current_y
        )

        yaw_error = abs(
            self._angle_difference(
                candidate_yaw,
                current_yaw
            )
        )

        if (
                position_error <= self.position_error_threshold and
                yaw_error <= self.yaw_error_threshold
        ):
            self.candidate_samples[tag_id].clear()
            self.feedback_message = (
                f"Localization agrees with tag {tag_id}: "
                f"{position_error:.2f} m, "
                f"{math.degrees(yaw_error):.1f} deg"
            )
            return py_trees.common.Status.SUCCESS

        self.candidate_samples[tag_id].append(
            (
                candidate_x,
                candidate_y,
                candidate_yaw
            )
        )

        stable_pose = self._stable_candidate(tag_id)

        if stable_pose is None:
            self.feedback_message = (
                f"Possible localization error from tag {tag_id}: "
                f"{position_error:.2f} m, "
                f"{math.degrees(yaw_error):.1f} deg, "
                f"{len(self.candidate_samples[tag_id])}/"
                f"{self.required_samples} samples"
            )
            return py_trees.common.Status.SUCCESS

        corrected_x, corrected_y, corrected_yaw = stable_pose

        corrected_pose = PoseWithCovarianceStamped()
        corrected_pose.header.frame_id = self.map_frame
        corrected_pose.header.stamp = (
            self.node.get_clock().now().to_msg()
        )

        corrected_pose.pose.pose.position.x = corrected_x
        corrected_pose.pose.pose.position.y = corrected_y
        corrected_pose.pose.pose.position.z = 0.0

        corrected_pose.pose.pose.orientation.x = 0.0
        corrected_pose.pose.pose.orientation.y = 0.0
        corrected_pose.pose.pose.orientation.z = math.sin(
            corrected_yaw * 0.5
        )
        corrected_pose.pose.pose.orientation.w = math.cos(
            corrected_yaw * 0.5
        )

        covariance = [0.0] * 36
        covariance[0] = 0.04
        covariance[7] = 0.04
        covariance[35] = math.radians(10.0) ** 2
        corrected_pose.pose.covariance = covariance

        self.initialpose_pub.publish(corrected_pose)

        self.last_relocalization_ns = now_ns
        self.candidate_samples.clear()

        self.feedback_message = (
            f"Relocalized from tag {tag_id}: "
            f"{position_error:.2f} m, "
            f"{math.degrees(yaw_error):.1f} deg"
        )

        return py_trees.common.Status.SUCCESS

    def _pose_to_mat4(self, pose) -> np.ndarray:
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        norm = math.sqrt(
            qx * qx +
            qy * qy +
            qz * qz +
            qw * qw
        )

        if norm == 0.0:
            qx = 0.0
            qy = 0.0
            qz = 0.0
            qw = 1.0
        else:
            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm

        rotation = np.array([
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw)
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw)
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy)
            ]
        ])

        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[0, 3] = pose.position.x
        matrix[1, 3] = pose.position.y
        matrix[2, 3] = pose.position.z
        return matrix

    def _transform_to_mat4(self, transform) -> np.ndarray:
        pose = PoseStamped()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return self._pose_to_mat4(pose.pose)

    def _matrix_yaw(self, matrix: np.ndarray) -> float:
        return math.atan2(matrix[1, 0], matrix[0, 0])

    def _angle_difference(self, first: float, second: float) -> float:
        return math.atan2(
            math.sin(first - second),
            math.cos(first - second)
        )

    def _calculate_candidate_pose(
            self,
            tag_true: PoseStamped,
            tag_observed: PoseStamped
    ):
        tag_in_base = self.tf_buffer.transform(
            tag_observed,
            self.base_frame,
            timeout=self.tf_timeout
        )

        map_to_tag = self._pose_to_mat4(tag_true.pose)
        base_to_tag = self._pose_to_mat4(tag_in_base.pose)

        map_to_base = map_to_tag.dot(
            np.linalg.inv(base_to_tag)
        )

        return map_to_base, tag_in_base.header.stamp

    def _stable_candidate(self, tag_id: int):
        samples = self.candidate_samples[tag_id]

        if len(samples) < self.required_samples:
            return None

        xs = np.array([sample[0] for sample in samples])
        ys = np.array([sample[1] for sample in samples])
        yaws = np.array([sample[2] for sample in samples])

        x = float(np.median(xs))
        y = float(np.median(ys))

        yaw = math.atan2(
            float(np.mean(np.sin(yaws))),
            float(np.mean(np.cos(yaws)))
        )

        position_spread = max(
            math.hypot(sample_x - x, sample_y - y)
            for sample_x, sample_y, _ in samples
        )

        yaw_spread = max(
            abs(self._angle_difference(sample_yaw, yaw))
            for _, _, sample_yaw in samples
        )

        if position_spread > self.maximum_position_spread:
            return None

        if yaw_spread > self.maximum_yaw_spread:
            return None

        return x, y, yaw
