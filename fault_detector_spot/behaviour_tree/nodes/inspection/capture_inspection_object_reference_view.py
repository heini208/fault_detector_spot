"""Capture a routine reference view from one complex command."""

from pathlib import Path
from typing import Any, Optional, Union

import py_trees
import tf2_ros
from rclpy.node import Node

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.commands.generic_complex_command import (
    GenericCommand,
)
from fault_detector_spot.inspection.object_repository import (
    ObjectRepository,
)
from fault_detector_spot.inspection.reference_view_capture import (
    capture_reference_view,
)
from fault_detector_spot.inspection.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)


class CaptureInspectionObjectReferenceView(
    py_trees.behaviour.Behaviour
):
    """Persist one validated reference view for the commanded routine."""

    def __init__(
        self,
        rgb_topic: str = "/camera/hand/image",
        depth_topic: str = "/depth_registered/hand/image",
        camera_info_topic: str = "/depth_registered/hand/camera_info",
        base_tag_topic: str = "fault_detector/state/visible_tags",
        object_root: Optional[Union[str, Path]] = None,
        synchronization_queue_size: int = 10,
        maximum_input_age_sec: float = 0.75,
        maximum_timestamp_skew_sec: float = 0.05,
        maximum_tag_timestamp_skew_sec: float = 0.25,
        fixed_frame: str = "odom",
        transform_timeout_sec: float = 0.05,
        name: str = "CaptureInspectionObjectReferenceView",
    ):
        """Configure persistent capture resources and timing limits."""
        super().__init__(name)
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.base_tag_topic = base_tag_topic
        self.object_root = object_root
        self.synchronization_queue_size = synchronization_queue_size
        self.maximum_input_age_sec = maximum_input_age_sec
        self.maximum_timestamp_skew_sec = maximum_timestamp_skew_sec
        self.maximum_tag_timestamp_skew_sec = (
            maximum_tag_timestamp_skew_sec
        )
        self.fixed_frame = fixed_frame
        self.transform_timeout_sec = transform_timeout_sec
        self.node: Optional[Node] = None
        self.object_repository: Optional[ObjectRepository] = None
        self.input_synchronizer: Optional[
            ReferenceViewInputSynchronizer
        ] = None
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs: Any) -> None:
        """Create subscriptions, TF history, and object storage access."""
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError(f"{self.name}: no ROS node provided")

        self.blackboard.register_key(
            "last_command",
            access=py_trees.common.Access.READ,
        )
        self.object_repository = ObjectRepository(self.object_root)
        self.input_synchronizer = ReferenceViewInputSynchronizer(
            node=self.node,
            rgb_topic=self.rgb_topic,
            depth_topic=self.depth_topic,
            camera_info_topic=self.camera_info_topic,
            base_tag_topic=self.base_tag_topic,
            queue_size=self.synchronization_queue_size,
            maximum_timestamp_skew_sec=(
                self.maximum_timestamp_skew_sec
            ),
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self.node,
        )

    def update(self) -> py_trees.common.Status:
        """Execute one complete reference-view capture transaction."""
        try:
            command = self._command()
            inspection = command.inspection
            object_id = inspection.object.object_id
            routine_id = inspection.routine.routine_id
            result = capture_reference_view(
                self.object_repository,
                self.input_synchronizer,
                self.tf_buffer,
                object_id,
                routine_id,
                self.node.get_clock().now(),
                maximum_input_age_sec=self.maximum_input_age_sec,
                maximum_timestamp_skew_sec=(
                    self.maximum_timestamp_skew_sec
                ),
                maximum_tag_timestamp_skew_sec=(
                    self.maximum_tag_timestamp_skew_sec
                ),
                fixed_frame=self.fixed_frame,
                transform_timeout_sec=self.transform_timeout_sec,
                replace_existing=inspection.replace_existing,
            )
            routine = result.get_routine(routine_id)
            if routine is None or routine.reference_view is None:
                raise RuntimeError(
                    "Capture did not produce a reference view"
                )
            dataset_path = routine.reference_view.reference_dataset_path
        except Exception as exception:
            self.feedback_message = (
                f"Reference-view capture failed: {exception}"
            )
            if self.node is not None:
                self.node.get_logger().error(self.feedback_message)
            return py_trees.common.Status.FAILURE

        self.feedback_message = (
            "Captured reference view for "
            f"{object_id}/{routine_id}: "
            f"{dataset_path}"
        )
        self.node.get_logger().info(self.feedback_message)
        return py_trees.common.Status.SUCCESS

    def _command(self) -> GenericCommand:
        if (
            not self.blackboard.exists("last_command")
            or self.blackboard.last_command is None
        ):
            raise RuntimeError("No command is available")

        command = self.blackboard.last_command
        if not isinstance(command, GenericCommand):
            raise TypeError(
                "Reference-view capture requires a GenericCommand"
            )
        if (
            command.command_id
            != CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW
        ):
            raise ValueError(
                "Command is not a reference-view capture command"
            )
        if command.inspection is None:
            raise ValueError("Command inspection payload must be set")
        if not command.inspection.object.object_id:
            raise ValueError("Command object_id must not be empty")
        if not command.inspection.routine.routine_id:
            raise ValueError("Command routine_id must not be empty")
        return command
