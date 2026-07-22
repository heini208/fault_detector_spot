"""Capture a routine reference view from one complex command."""

import math
from pathlib import Path
from typing import Any, Optional, Tuple, Union

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
    ReferenceViewCaptureNotReady,
    capture_reference_view,
    validate_reference_view_capture_target,
)
from fault_detector_spot.inspection.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)
from fault_detector_spot.inspection.reference_view_validation import (
    ReferenceViewInputNotReady,
)


CaptureRequest = Tuple[str, str, bool, int]


class CaptureInspectionObjectReferenceView(
    py_trees.behaviour.Behaviour
):
    """Collect, validate, and persist one routine reference view."""

    def __init__(
        self,
        rgb_topic: str = "/camera/hand/image",
        depth_topic: str = "/depth_registered/hand/image",
        camera_info_topic: str = "/depth_registered/hand/camera_info",
        base_tag_topic: str = "fault_detector/state/visible_tags",
        object_root: Optional[Union[str, Path]] = None,
        synchronization_queue_size: int = 10,
        maximum_input_age_sec: float = 2.0,
        maximum_timestamp_skew_sec: float = 0.05,
        maximum_tag_timestamp_skew_sec: float = 0.25,
        collection_duration_sec: float = 1.0,
        fixed_frame: str = "odom",
        transform_timeout_sec: float = 0.05,
        capture_timeout_sec: float = 3.0,
        capture_max_attempts: int = 3,
        name: str = "CaptureInspectionObjectReferenceView",
    ):
        """Configure capture topics, timing limits, and storage."""
        super().__init__(name)
        if (
            not math.isfinite(collection_duration_sec)
            or collection_duration_sec <= 0.0
        ):
            raise ValueError(
                "Collection duration must be finite and positive"
            )
        if (
            not math.isfinite(capture_timeout_sec)
            or capture_timeout_sec <= 0.0
        ):
            raise ValueError(
                "Capture timeout must be finite and positive"
            )
        if (
            isinstance(capture_max_attempts, bool)
            or not isinstance(capture_max_attempts, int)
            or capture_max_attempts <= 0
        ):
            raise ValueError(
                "Capture maximum attempts must be a positive integer"
            )

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
        self.collection_duration_sec = collection_duration_sec
        self.fixed_frame = fixed_frame
        self.transform_timeout_sec = transform_timeout_sec
        self.capture_timeout_sec = capture_timeout_sec
        self.capture_max_attempts = capture_max_attempts

        self.node: Optional[Node] = None
        self.object_repository: Optional[ObjectRepository] = None
        self.input_synchronizer: Optional[
            ReferenceViewInputSynchronizer
        ] = None
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        self._request: Optional[CaptureRequest] = None
        self._attempt_started_nanoseconds: Optional[int] = None
        self._attempt_number = 0
        self._minimum_image_sequence: Optional[int] = None
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs: Any) -> None:
        """Create sensor subscriptions, TF history, and storage access."""
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
            maximum_tag_timestamp_skew_sec=(
                self.maximum_tag_timestamp_skew_sec
            ),
            collection_duration_sec=self.collection_duration_sec,
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self.node,
        )

    def initialise(self) -> None:
        """Reset state when a new capture command enters the behavior."""
        if self.input_synchronizer is not None:
            self.input_synchronizer.cancel_collection()
        self._request = None
        self._attempt_started_nanoseconds = None
        self._attempt_number = 0
        self._minimum_image_sequence = None

    def update(self) -> py_trees.common.Status:
        """Drive one nonblocking bounded collection transaction."""
        try:
            if self._request is None:
                self._begin_request()
                return py_trees.common.Status.RUNNING

            current_time = self.node.get_clock().now()
            if not self.input_synchronizer.collection_ready():
                return self._wait_retry_or_timeout(
                    current_time,
                    "collecting synchronized RGB, depth, and tag inputs",
                )

            object_id, routine_id, replace_existing, _ = self._request
            result = capture_reference_view(
                self.object_repository,
                self.input_synchronizer,
                self.tf_buffer,
                object_id,
                routine_id,
                current_time,
                maximum_input_age_sec=self.maximum_input_age_sec,
                maximum_timestamp_skew_sec=(
                    self.maximum_timestamp_skew_sec
                ),
                maximum_tag_timestamp_skew_sec=(
                    self.maximum_tag_timestamp_skew_sec
                ),
                fixed_frame=self.fixed_frame,
                transform_timeout_sec=self.transform_timeout_sec,
                replace_existing=replace_existing,
                minimum_image_sequence=self._minimum_image_sequence,
            )
            routine = result.get_routine(routine_id)
            if routine is None or routine.reference_view is None:
                raise RuntimeError(
                    "Capture did not produce a reference view"
                )
            dataset_path = routine.reference_view.reference_dataset_path
        except (
            ReferenceViewCaptureNotReady,
            ReferenceViewInputNotReady,
        ) as exception:
            return self._retry_or_fail(
                self.node.get_clock().now(),
                str(exception),
            )
        except tf2_ros.TransformException as exception:
            return self._wait_retry_or_timeout(
                self.node.get_clock().now(),
                str(exception),
            )
        except Exception as exception:
            return self._failure(exception)

        self.input_synchronizer.cancel_collection()
        self.feedback_message = (
            "Captured reference view for "
            f"{object_id}/{routine_id}: "
            f"{dataset_path}"
        )
        self.node.get_logger().info(self.feedback_message)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Release any unfinished collection when execution stops."""
        if self.input_synchronizer is not None:
            self.input_synchronizer.cancel_collection()

    def _begin_request(self) -> None:
        command = self._command()
        inspection = command.inspection
        object_id = inspection.object.object_id
        routine_id = inspection.routine.routine_id
        replace_existing = inspection.replace_existing
        reference_tag_id = validate_reference_view_capture_target(
            self.object_repository,
            object_id,
            routine_id,
            replace_existing,
        )
        self._request = (
            object_id,
            routine_id,
            replace_existing,
            reference_tag_id,
        )
        self._attempt_number = 1
        self._start_attempt(self.node.get_clock().now())

    def _start_attempt(self, current_time) -> None:
        _, _, _, reference_tag_id = self._request
        self.input_synchronizer.cancel_collection()
        self._attempt_started_nanoseconds = current_time.nanoseconds
        self._minimum_image_sequence = (
            self.input_synchronizer.image_sequence + 1
        )
        self.input_synchronizer.begin_collection(
            reference_tag_id,
            self._minimum_image_sequence,
        )
        self.feedback_message = (
            "Collecting reference-view inputs, attempt "
            f"{self._attempt_number}/{self.capture_max_attempts}"
        )

    def _wait_retry_or_timeout(
        self,
        current_time,
        reason: str,
    ) -> py_trees.common.Status:
        elapsed_sec = (
            current_time.nanoseconds
            - self._attempt_started_nanoseconds
        ) / 1_000_000_000
        if elapsed_sec < self.capture_timeout_sec:
            self.feedback_message = (
                "Waiting for reference-view inputs: "
                f"{reason}"
            )
            return py_trees.common.Status.RUNNING
        return self._retry_or_fail(current_time, reason)

    def _retry_or_fail(
        self,
        current_time,
        reason: str,
    ) -> py_trees.common.Status:
        if self._attempt_number < self.capture_max_attempts:
            self.node.get_logger().warning(
                "Reference-view capture attempt "
                f"{self._attempt_number}/"
                f"{self.capture_max_attempts} failed: "
                f"{reason}. Retrying"
            )
            self._attempt_number += 1
            self._start_attempt(current_time)
            return py_trees.common.Status.RUNNING

        return self._failure(RuntimeError(
            f"failed after {self.capture_max_attempts} attempts: "
            f"{reason}"
        ))

    def _failure(self, exception: Exception) -> py_trees.common.Status:
        if self.input_synchronizer is not None:
            self.input_synchronizer.cancel_collection()
        self.feedback_message = (
            f"Reference-view capture failed: {exception}"
        )
        if self.node is not None:
            self.node.get_logger().error(self.feedback_message)
        return py_trees.common.Status.FAILURE

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
