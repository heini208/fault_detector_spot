"""Publish AprilTag observations independently of behavior-tree execution."""

import math

import rclpy
import tf2_ros
from apriltag_msgs.msg import AprilTagDetectionArray
from fault_detector_msgs.msg import TagElementArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

from fault_detector_spot.sensing.observations.tag_observation_tracker import (
    BaseTagObservationTracker,
    HandTagObservationTracker,
    merge_tag_observations,
)
from fault_detector_spot.sensing.observations.tag_reachability import (
    TagReachabilityFilter,
)
from fault_detector_spot.shared.ros.qos_profiles import TAG_STATE_QOS


class TagObservationNode(Node):
    """Own tag observation, freshness, reachability, and publication."""

    def __init__(self):
        super().__init__("tag_observation")
        base_frame = self._parameter("tag_sensing.base_frame", "body")
        base_frame_pattern = self._parameter(
            "tag_sensing.base_frame_pattern",
            r"(?<!filtered_)fiducial_(\d+)",
        )
        hand_detection_topic = self._parameter(
            "tag_sensing.hand_detection_topic",
            "/detections",
        )
        hand_tag_frame_prefix = self._parameter(
            "tag_sensing.hand_tag_frame_prefix",
            "tag36h11:",
        )
        base_max_age_sec = float(
            self._parameter("tag_sensing.base_max_age_sec", 1.5)
        )
        hand_max_age_sec = float(
            self._parameter("tag_sensing.hand_max_age_sec", 1.0)
        )
        hand_tf_pending_sec = float(
            self._parameter("tag_sensing.hand_tf_pending_sec", 0.5)
        )
        hand_max_hamming = int(
            self._parameter("tag_sensing.hand_max_hamming", 0)
        )
        hand_min_decision_margin = float(
            self._parameter(
                "tag_sensing.hand_min_decision_margin",
                0.0,
            )
        )
        publish_period_sec = float(
            self._parameter("tag_sensing.publish_period_sec", 0.05)
        )
        arm_base_frame = str(
            self._parameter(
                "tag_sensing.arm_base_frame",
                "arm_link_sh0",
            )
        ).strip()
        arm_reach_m = float(
            self._parameter("tag_sensing.arm_reach_m", 1.1)
        )
        arm_reach_tolerance_m = float(
            self._parameter(
                "tag_sensing.arm_reach_tolerance_m",
                0.0,
            )
        )

        if publish_period_sec <= 0.0:
            raise ValueError(
                "tag_sensing.publish_period_sec must be positive"
            )
        if (
            not math.isfinite(arm_reach_tolerance_m)
            or arm_reach_tolerance_m < 0.0
        ):
            raise ValueError(
                "tag_sensing.arm_reach_tolerance_m must be finite "
                "and non-negative"
            )
        if (
            not math.isfinite(arm_reach_m)
            or arm_reach_m <= arm_reach_tolerance_m
        ):
            raise ValueError(
                "tag_sensing.arm_reach_m must be finite and exceed "
                "its tolerance"
            )
        if not str(base_frame).strip():
            raise ValueError("tag_sensing.base_frame must not be empty")
        if not arm_base_frame:
            raise ValueError(
                "tag_sensing.arm_base_frame must not be empty"
            )

        self._base_frame = str(base_frame).strip()
        self._arm_base_frame = arm_base_frame
        self._arm_base_position = None
        self._reachability_filter = TagReachabilityFilter(
            arm_reach_m - arm_reach_tolerance_m
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            self,
        )
        self._base_tracker = BaseTagObservationTracker(
            self._tf_buffer,
            frame_pattern=base_frame_pattern,
            target_frame=self._base_frame,
            max_age_sec=base_max_age_sec,
            logger=self.get_logger(),
        )
        self._hand_tracker = HandTagObservationTracker(
            self._tf_buffer,
            target_frame=self._base_frame,
            tag_frame_prefix=hand_tag_frame_prefix,
            max_age_sec=hand_max_age_sec,
            tf_pending_sec=hand_tf_pending_sec,
            max_hamming=hand_max_hamming,
            min_decision_margin=hand_min_decision_margin,
            logger=self.get_logger(),
        )
        self._tf_frame_subscription = self.create_subscription(
            TFMessage,
            "/tf",
            self._base_tracker.receive_tf_frames,
            qos_profile_sensor_data,
        )
        self._hand_detection_subscription = self.create_subscription(
            AprilTagDetectionArray,
            hand_detection_topic,
            self._hand_tracker.receive_detections,
            10,
        )
        self._base_publisher = self.create_publisher(
            TagElementArray,
            "fault_detector/state/base_tags",
            TAG_STATE_QOS,
        )
        self._visible_publisher = self.create_publisher(
            TagElementArray,
            "fault_detector/state/visible_tags",
            TAG_STATE_QOS,
        )
        self._reachable_publisher = self.create_publisher(
            TagElementArray,
            "fault_detector/state/reachable_tags",
            TAG_STATE_QOS,
        )
        self._last_signature = None
        self._publish_timer = self.create_timer(
            publish_period_sec,
            self._publish_observations,
        )

    def _parameter(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _publish_observations(self) -> None:
        now = self.get_clock().now()
        base_observations = self._base_tracker.snapshot(now)
        hand_observations = self._hand_tracker.snapshot(now)
        visible_observations = merge_tag_observations(
            base_observations,
            hand_observations,
        )
        reachable_observations = self._reachable_observations(
            visible_observations
        )

        base_message = TagElementArray()
        base_message.elements = list(base_observations.values())
        visible_message = TagElementArray()
        visible_message.elements = list(visible_observations.values())
        reachable_message = TagElementArray()
        reachable_message.elements = list(reachable_observations.values())

        self._base_publisher.publish(base_message)
        self._visible_publisher.publish(visible_message)
        self._reachable_publisher.publish(reachable_message)

        signature = (
            tuple(sorted(base_observations)),
            tuple(sorted(hand_observations)),
            tuple(sorted(reachable_observations)),
        )
        if signature != self._last_signature:
            self.get_logger().info(
                f"Base tags: {list(signature[0])}; "
                f"hand fallback tags: {list(signature[1])}; "
                f"reachable tags: {list(signature[2])}"
            )
            self._last_signature = signature

    def _reachable_observations(self, visible_observations):
        return self._reachability_filter.filter(
            visible_observations,
            self._get_arm_base_position(),
        )

    def _get_arm_base_position(self):
        if self._arm_base_position is not None:
            return self._arm_base_position

        try:
            if not self._tf_buffer.can_transform(
                self._base_frame,
                self._arm_base_frame,
                Time(),
            ):
                return None
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._arm_base_frame,
                Time(),
            )
        except Exception:
            return None

        translation = transform.transform.translation
        self._arm_base_position = (
            float(translation.x),
            float(translation.y),
            float(translation.z),
        )
        self.get_logger().info(
            "Resolved arm base offset "
            f"{self._arm_base_position} in {self._base_frame}"
        )
        return self._arm_base_position

    def destroy_node(self):
        if self._tf_listener is not None:
            self._tf_listener.unregister()
            self._tf_listener = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TagObservationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
