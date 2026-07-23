"""Capture up to three synchronized camera-specific reference views."""

import math
from pathlib import Path
from typing import Any, Dict, Optional, Tuple, Union

import py_trees
import tf2_ros
from rclpy.node import Node

from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
)
from fault_detector_spot.behaviour_tree.commands.generic_complex_command import (
    GenericCommand,
)
from fault_detector_spot.inspection.multi_reference_view_capture import (
    CameraCaptureRequest,
    capture_reference_views,
    validate_multi_reference_view_capture_target,
)
from fault_detector_spot.inspection.multi_reference_view_repository import (
    MultiReferenceViewRepository,
)
from fault_detector_spot.inspection.reference_camera_registry import (
    REFERENCE_CAMERA_BY_ID,
    ReferenceCameraConfig,
    validate_reference_camera_slots,
)
from fault_detector_spot.inspection.reference_view_capture import (
    ReferenceViewCaptureNotReady,
)
from fault_detector_spot.inspection.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)
from fault_detector_spot.inspection.reference_view_validation import (
    ReferenceViewInputNotReady,
)


CaptureRequest = Tuple[
    str,
    str,
    bool,
    int,
    Tuple[Tuple[int, str], ...],
]


class CaptureInspectionObjectReferenceView(
    py_trees.behaviour.Behaviour
):
    """Collect and atomically persist one to three camera views."""

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
        super().__init__(name)
        self._validate_configuration(
            collection_duration_sec,
            capture_timeout_sec,
            capture_max_attempts,
        )
        self.camera_configs = dict(REFERENCE_CAMERA_BY_ID)
        self.camera_configs["hand"] = ReferenceCameraConfig(
            camera_id="hand",
            display_name="Hand",
            rgb_topic=rgb_topic,
            depth_topic=depth_topic,
            camera_info_topic=camera_info_topic,
        )
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
        self.repository: Optional[MultiReferenceViewRepository] = None
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        self._request: Optional[CaptureRequest] = None
        self._synchronizers: Dict[str, ReferenceViewInputSynchronizer] = {}
        self._minimum_image_sequences: Dict[str, int] = {}
        self._attempt_started_nanoseconds: Optional[int] = None
        self._attempt_number = 0
        self._collection_started = False
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs: Any) -> None:
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError(f"{self.name}: no ROS node provided")
        self.blackboard.register_key(
            "last_command",
            access=py_trees.common.Access.READ,
        )
        self.repository = MultiReferenceViewRepository(self.object_root)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self.node,
        )

    def initialise(self) -> None:
        self._release_synchronizers()
        self._request = None
        self._minimum_image_sequences = {}
        self._attempt_started_nanoseconds = None
        self._attempt_number = 0
        self._collection_started = False

    def update(self) -> py_trees.common.Status:
        try:
            if self._request is None:
                self._begin_request()
                return py_trees.common.Status.RUNNING

            current_time = self.node.get_clock().now()
            if not self._collection_started:
                return self._wait_for_input_warmup(current_time)

            if not all(
                synchronizer.collection_ready()
                for synchronizer in self._synchronizers.values()
            ):
                return self._wait_retry_or_timeout(
                    current_time,
                    "collecting synchronized camera and tag inputs",
                )

            object_id, routine_id, _, tag_id, selected = self._request
            requests = [
                CameraCaptureRequest(
                    slot_index=slot_index,
                    camera_id=camera_id,
                    input_synchronizer=self._synchronizers[camera_id],
                    minimum_image_sequence=(
                        self._minimum_image_sequences[camera_id]
                    ),
                )
                for slot_index, camera_id in selected
            ]
            capture_reference_views(
                self.repository,
                requests,
                self.tf_buffer,
                object_id,
                routine_id,
                current_time,
                tag_id,
                maximum_input_age_sec=self.maximum_input_age_sec,
                maximum_timestamp_skew_sec=(
                    self.maximum_timestamp_skew_sec
                ),
                maximum_tag_timestamp_skew_sec=(
                    self.maximum_tag_timestamp_skew_sec
                ),
                fixed_frame=self.fixed_frame,
                transform_timeout_sec=self.transform_timeout_sec,
            )
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

        camera_names = ", ".join(
            camera_id for _, camera_id in selected
        )
        self.feedback_message = (
            "Captured reference views for "
            f"{object_id}/{routine_id}: {camera_names}"
        )
        self.node.get_logger().info(self.feedback_message)
        self._release_synchronizers()
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self._release_synchronizers()

    def _begin_request(self) -> None:
        command = self._command()
        inspection = command.inspection
        object_id = inspection.object.object_id
        routine_id = inspection.routine.routine_id
        replace_existing = inspection.replace_existing
        selected = validate_reference_camera_slots(
            inspection.reference_camera_ids
        )
        reference_tag_id = validate_multi_reference_view_capture_target(
            self.repository,
            object_id,
            routine_id,
            replace_existing,
        )
        self._request = (
            object_id,
            routine_id,
            replace_existing,
            reference_tag_id,
            selected,
        )
        self._create_synchronizers(selected)
        self._attempt_number = 1
        self._collection_started = False
        self._attempt_started_nanoseconds = (
            self.node.get_clock().now().nanoseconds
        )
        cameras = ", ".join(
            camera_id for _, camera_id in selected
        )
        self.feedback_message = (
            "Waiting for reference-view inputs from "
            f"{cameras}"
        )

    def _create_synchronizers(self, selected) -> None:
        self._release_synchronizers()
        for _, camera_id in selected:
            camera = self.camera_configs[camera_id]
            self._synchronizers[camera_id] = ReferenceViewInputSynchronizer(
                node=self.node,
                rgb_topic=camera.rgb_topic,
                depth_topic=camera.depth_topic,
                camera_info_topic=camera.camera_info_topic,
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

    def _start_attempt(self, current_time) -> None:
        _, _, _, reference_tag_id, selected = self._request
        self._attempt_started_nanoseconds = current_time.nanoseconds
        self._collection_started = True
        self._minimum_image_sequences = {}
        for _, camera_id in selected:
            synchronizer = self._synchronizers[camera_id]
            synchronizer.cancel_collection()
            minimum_sequence = synchronizer.image_sequence + 1
            self._minimum_image_sequences[camera_id] = minimum_sequence
            synchronizer.begin_collection(
                reference_tag_id,
                minimum_sequence,
            )
        cameras = ", ".join(camera_id for _, camera_id in selected)
        self.feedback_message = (
            "Collecting reference-view inputs for "
            f"{cameras}, attempt {self._attempt_number}/"
            f"{self.capture_max_attempts}"
        )

    def _wait_for_input_warmup(self, current_time):
        _, _, _, reference_tag_id, selected = self._request
        missing = []
        for _, camera_id in selected:
            synchronizer = self._synchronizers[camera_id]
            if not synchronizer.ready_for_collection(reference_tag_id):
                missing.append(camera_id)

        if not missing:
            self._start_attempt(current_time)
            return py_trees.common.Status.RUNNING

        elapsed_sec = (
            current_time.nanoseconds
            - self._attempt_started_nanoseconds
        ) / 1_000_000_000
        details = ", ".join(
            self._camera_input_diagnostics(
                camera_id,
                reference_tag_id,
            )
            for camera_id in missing
        )
        if elapsed_sec < self.capture_timeout_sec:
            self.feedback_message = (
                "Waiting for camera subscriptions to warm up: "
                f"{details}"
            )
            return py_trees.common.Status.RUNNING

        return self._failure(RuntimeError(
            "Camera inputs did not become ready before capture: "
            f"{details}"
        ))

    def _camera_input_diagnostics(
        self,
        camera_id,
        reference_tag_id,
    ):
        synchronizer = self._synchronizers[camera_id]
        return (
            f"{camera_id}: "
            f"{synchronizer.input_diagnostics(reference_tag_id)}"
        )

    def _wait_retry_or_timeout(self, current_time, reason):
        elapsed_sec = (
            current_time.nanoseconds
            - self._attempt_started_nanoseconds
        ) / 1_000_000_000
        if elapsed_sec < self.capture_timeout_sec:
            self.feedback_message = (
                f"Waiting for reference-view inputs: {reason}"
            )
            return py_trees.common.Status.RUNNING
        return self._retry_or_fail(current_time, reason)

    def _retry_or_fail(self, current_time, reason):
        if self._attempt_number < self.capture_max_attempts:
            self.node.get_logger().warning(
                "Reference-view capture attempt "
                f"{self._attempt_number}/{self.capture_max_attempts} "
                f"failed: {reason}. Retrying"
            )
            self._attempt_number += 1
            self._start_attempt(current_time)
            return py_trees.common.Status.RUNNING
        return self._failure(RuntimeError(
            f"failed after {self.capture_max_attempts} attempts: {reason}"
        ))

    def _failure(self, exception: Exception):
        self.feedback_message = (
            f"Reference-view capture failed: {exception}"
        )
        if self.node is not None:
            self.node.get_logger().error(self.feedback_message)
        self._release_synchronizers()
        return py_trees.common.Status.FAILURE

    def _release_synchronizers(self) -> None:
        for synchronizer in self._synchronizers.values():
            synchronizer.cancel_collection()
            rgb_subscription = getattr(
                synchronizer,
                "rgb_subscription",
                None,
            )
            depth_subscription = getattr(
                synchronizer,
                "depth_subscription",
                None,
            )
            if rgb_subscription is not None:
                self._destroy_filter_subscription(rgb_subscription)
            if depth_subscription is not None:
                self._destroy_filter_subscription(depth_subscription)
            if (
                self.node is not None
                and hasattr(self.node, "destroy_subscription")
            ):
                for attribute in (
                    "camera_info_subscription",
                    "base_tag_subscription",
                ):
                    subscription = getattr(
                        synchronizer,
                        attribute,
                        None,
                    )
                    if subscription is not None:
                        self.node.destroy_subscription(subscription)
        self._synchronizers = {}
        self._collection_started = False

    def _destroy_filter_subscription(self, subscription) -> None:
        if hasattr(subscription, "unsubscribe"):
            subscription.unsubscribe()
            return
        underlying = getattr(subscription, "sub", None)
        if underlying is not None and self.node is not None:
            self.node.destroy_subscription(underlying)

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

    @staticmethod
    def _validate_configuration(
        collection_duration_sec,
        capture_timeout_sec,
        capture_max_attempts,
    ) -> None:
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
