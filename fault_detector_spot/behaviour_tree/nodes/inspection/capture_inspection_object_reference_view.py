"""Capture up to three synchronized camera-specific reference views."""

import math
from collections import deque
from copy import deepcopy
from pathlib import Path
from typing import Any, Deque, Dict, Optional, Set, Tuple, Union

import py_trees
import tf2_ros
from fault_detector_msgs.msg import TagElement, TagElementArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from fault_detector_spot.behaviour_tree.commands import (
    generic_complex_command,
)
from fault_detector_spot.behaviour_tree.commands.command_ids import (
    CommandID,
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


GenericCommand = generic_complex_command.GenericCommand


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
        rgb_camera_info_topic: str = "/camera/hand/camera_info",
        depth_camera_info_topic: str = (
            "/depth_registered/hand/camera_info"
        ),
        base_tag_topic: str = "fault_detector/state/base_tags",
        object_root: Optional[Union[str, Path]] = None,
        synchronization_queue_size: int = 10,
        maximum_input_age_sec: float = 2.0,
        maximum_timestamp_skew_sec: float = 0.05,
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
            maximum_input_age_sec,
        )
        self.camera_configs = dict(REFERENCE_CAMERA_BY_ID)
        self.camera_configs["hand"] = ReferenceCameraConfig(
            camera_id="hand",
            display_name="Hand",
            rgb_topic=rgb_topic,
            depth_topic=depth_topic,
            rgb_camera_info_topic=rgb_camera_info_topic,
            depth_camera_info_topic=depth_camera_info_topic,
        )
        self.base_tag_topic = base_tag_topic
        self.object_root = object_root
        self.synchronization_queue_size = synchronization_queue_size
        self.maximum_input_age_sec = maximum_input_age_sec
        self.maximum_timestamp_skew_sec = maximum_timestamp_skew_sec
        self.collection_duration_sec = collection_duration_sec
        self.fixed_frame = fixed_frame
        self.transform_timeout_sec = transform_timeout_sec
        self.capture_timeout_sec = capture_timeout_sec
        self.capture_max_attempts = capture_max_attempts

        self.node: Optional[Node] = None
        self.repository: Optional[MultiReferenceViewRepository] = None
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        self.base_tag_subscription = None
        self._request: Optional[CaptureRequest] = None
        self._synchronizers: Dict[
            str,
            ReferenceViewInputSynchronizer,
        ] = {}
        self._minimum_input_sequences: Dict[str, int] = {}
        self._tag_history_size = max(
            synchronization_queue_size,
            60,
        )
        self._base_tag_history: Dict[
            int,
            Deque[TagElement],
        ] = {}
        self._visible_tag_ids: Set[int] = set()
        self._latest_tag_stamp_nanoseconds: Optional[int] = None
        self._warmup_started_nanoseconds: Optional[int] = None
        self._attempt_started_nanoseconds: Optional[int] = None
        self._capture_validation_time = None
        self._attempt_number = 0
        self._collection_started = False
        self._attempt_tag_samples: Deque[TagElement] = deque(maxlen=1)
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
        self.base_tag_subscription = self.node.create_subscription(
            TagElementArray,
            self.base_tag_topic,
            self._base_tags_callback,
            qos_profile_sensor_data,
        )

    def initialise(self) -> None:
        self._release_synchronizers()
        self._request = None
        self._minimum_input_sequences = {}
        self._warmup_started_nanoseconds = None
        self._attempt_started_nanoseconds = None
        self._capture_validation_time = None
        self._attempt_number = 0
        self._collection_started = False
        self._attempt_tag_samples.clear()

    def update(self) -> py_trees.common.Status:
        try:
            if self._request is None:
                self._begin_request()
                return py_trees.common.Status.RUNNING

            current_time = self.node.get_clock().now()
            if not self._collection_started:
                return self._wait_for_selected_inputs(current_time)

            object_id, routine_id, _, tag_id, selected = self._request
            if not all(
                self._synchronizers[camera_id].collection_ready()
                for _, camera_id in selected
            ):
                return self._wait_retry_or_timeout(
                    current_time,
                    "collecting synchronized camera and tag inputs",
                )

            if self._capture_validation_time is None:
                self._capture_validation_time = current_time

            requests = [
                CameraCaptureRequest(
                    slot_index=slot_index,
                    camera_id=camera_id,
                    input_synchronizer=self._synchronizers[camera_id],
                    minimum_input_sequence=(
                        self._minimum_input_sequences[camera_id]
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
                self._capture_validation_time,
                tag_id,
                tuple(self._attempt_tag_samples),
                maximum_input_age_sec=self.maximum_input_age_sec,
                maximum_timestamp_skew_sec=(
                    self.maximum_timestamp_skew_sec
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
        self._create_selected_synchronizers(
            selected,
        )
        self._attempt_number = 1
        self._warmup_started_nanoseconds = (
            self.node.get_clock().now().nanoseconds
        )
        cameras = ", ".join(
            camera_id for _, camera_id in selected
        )
        self.feedback_message = (
            "Waiting for selected reference camera inputs from "
            f"{cameras}"
        )
        self.node.get_logger().info(
            "Reference-view capture requested for "
            f"{object_id}/{routine_id} using {cameras}"
        )

    def _create_selected_synchronizers(
        self,
        selected,
    ) -> None:
        self._release_synchronizers()
        for _, camera_id in selected:
            camera = self.camera_configs[camera_id]
            synchronizer = ReferenceViewInputSynchronizer(
                node=self.node,
                rgb_topic=camera.rgb_topic,
                depth_topic=camera.depth_topic,
                rgb_camera_info_topic=(
                    camera.rgb_camera_info_topic
                ),
                depth_camera_info_topic=(
                    camera.depth_camera_info_topic
                ),
                queue_size=self.synchronization_queue_size,
                maximum_timestamp_skew_sec=(
                    self.maximum_timestamp_skew_sec
                ),
                collection_duration_sec=(
                    self.collection_duration_sec
                ),
            )
            self._synchronizers[camera_id] = synchronizer

    def _base_tags_callback(
        self,
        observations: TagElementArray,
    ) -> None:
        values = tuple(observations.elements)
        self._reset_history_on_backward_time_jump(values)
        self._visible_tag_ids = {
            int(observation.id) for observation in values
        }

        for observation in values:
            tag_id = int(observation.id)
            history = self._base_tag_history.setdefault(
                tag_id,
                deque(maxlen=self._tag_history_size),
            )
            self._append_tag_observation(
                history,
                observation,
            )

    def _reset_history_on_backward_time_jump(
        self,
        observations,
    ) -> None:
        stamps = [
            self._tag_stamp_nanoseconds(observation)
            for observation in observations
            if self._tag_stamp_nanoseconds(observation) > 0
        ]
        if not stamps:
            return

        newest_stamp = max(stamps)
        previous_stamp = self._latest_tag_stamp_nanoseconds
        if (
            previous_stamp is not None
            and newest_stamp + 100_000_000 < previous_stamp
        ):
            self._base_tag_history.clear()
            if self.node is not None:
                self.node.get_logger().warning(
                    "Reference-tag time moved backwards; "
                    "cleared retained tag history"
                )
        if (
            previous_stamp is None
            or newest_stamp > previous_stamp
            or newest_stamp + 100_000_000 < previous_stamp
        ):
            self._latest_tag_stamp_nanoseconds = newest_stamp

    def _current_reference_tag(self, tag_id: int):
        if tag_id not in self._visible_tag_ids:
            raise ReferenceViewCaptureNotReady(
                f"Reference tag {tag_id} is not visible in the latest "
                "base-tag snapshot"
            )
        history = self._base_tag_history.get(tag_id)
        if not history:
            raise ReferenceViewCaptureNotReady(
                f"No observation is available for reference tag {tag_id}"
            )
        observation = history[-1]
        stamp_nanoseconds = self._tag_stamp_nanoseconds(observation)
        if stamp_nanoseconds <= 0:
            raise ValueError(
                "Base-camera tag observation timestamp must not be zero"
            )
        return deepcopy(observation)

    def _resolve_capture_anchor(self, reference_tag: TagElement):
        timeout = Duration(seconds=self.transform_timeout_sec)
        anchored = deepcopy(reference_tag)
        anchored.pose = self.tf_buffer.transform(
            deepcopy(reference_tag.pose),
            self.fixed_frame,
            timeout=timeout,
        )
        if anchored.pose.header.frame_id != self.fixed_frame:
            raise ValueError(
                "TF returned the reference tag in frame "
                f"'{anchored.pose.header.frame_id}', expected "
                f"'{self.fixed_frame}'"
            )
        return anchored

    def _camera_tf_error(self, selected) -> str:
        timeout = Duration(seconds=self.transform_timeout_sec)
        for _, camera_id in selected:
            frame_id = self._synchronizers[
                camera_id
            ].latest_rgb_frame_id
            if not frame_id:
                return f"{camera_id}: RGB frame ID is not available"
            try:
                self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    frame_id,
                    Time(),
                    timeout=timeout,
                )
            except tf2_ros.TransformException as exception:
                return (
                    f"{camera_id}: camera TF frame '{frame_id}' "
                    f"is not connected to '{self.fixed_frame}': "
                    f"{exception}"
                )
        return ""

    @staticmethod
    def _append_tag_observation(
        history: Deque[TagElement],
        observation: TagElement,
    ) -> None:
        if not history:
            history.append(deepcopy(observation))
            return

        current_stamp = (
            CaptureInspectionObjectReferenceView
            ._tag_stamp_nanoseconds(history[-1])
        )
        observation_stamp = (
            CaptureInspectionObjectReferenceView
            ._tag_stamp_nanoseconds(observation)
        )
        if observation_stamp > current_stamp:
            history.append(deepcopy(observation))
        elif observation_stamp == current_stamp:
            history[-1] = deepcopy(observation)

    @staticmethod
    def _tag_stamp_nanoseconds(observation: TagElement) -> int:
        stamp = observation.pose.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _wait_for_selected_inputs(self, current_time):
        _, _, _, reference_tag_id, selected = self._request
        missing = [
            camera_id
            for _, camera_id in selected
            if not self._synchronizers[
                camera_id
            ].ready_for_collection()
        ]
        tag_error = ""
        reference_tag = None
        try:
            reference_tag = self._current_reference_tag(
                reference_tag_id,
            )
        except (ReferenceViewCaptureNotReady, ValueError) as exception:
            tag_error = str(exception)
        tf_error = ""
        anchored_tag = None
        if not missing and not tag_error:
            try:
                anchored_tag = self._resolve_capture_anchor(reference_tag)
                tf_error = self._camera_tf_error(selected)
            except (
                tf2_ros.TransformException,
                ValueError,
            ) as exception:
                tf_error = str(exception)
        if not missing and not tag_error and not tf_error:
            self._start_attempt(current_time, anchored_tag)
            return py_trees.common.Status.RUNNING

        elapsed_sec = (
            current_time.nanoseconds
            - self._warmup_started_nanoseconds
        ) / 1_000_000_000
        details = ", ".join(
            self._camera_input_diagnostics(
                camera_id,
            )
            for camera_id in missing
        )
        if tag_error:
            details = ", ".join(
                detail for detail in (
                    details,
                    f"tag {reference_tag_id}: {tag_error}",
                )
                if detail
            )
        if tf_error:
            details = ", ".join(
                detail for detail in (
                    details,
                    f"TF: {tf_error}",
                )
                if detail
            )
        if elapsed_sec < self.capture_timeout_sec:
            self.feedback_message = (
                "Waiting for selected camera subscriptions: "
                f"{details}"
            )
            return py_trees.common.Status.RUNNING

        return self._retry_or_fail(
            current_time,
            "Selected camera inputs did not become ready: "
            f"{details}",
        )

    def _camera_input_diagnostics(
        self,
        camera_id,
    ):
        synchronizer = self._synchronizers[camera_id]
        return (
            f"{camera_id}: "
            f"{synchronizer.input_diagnostics()}"
        )

    def _start_attempt(
        self,
        current_time,
        anchored_reference_tag: TagElement,
    ) -> None:
        _, _, _, _, selected = self._request
        self._attempt_started_nanoseconds = current_time.nanoseconds
        self._capture_validation_time = None
        self._collection_started = True
        self._attempt_tag_samples.clear()
        self._attempt_tag_samples.append(anchored_reference_tag)
        self._minimum_input_sequences = {}
        for _, camera_id in selected:
            synchronizer = self._synchronizers[camera_id]
            synchronizer.cancel_collection()
            minimum_sequence = synchronizer.input_sequence + 1
            self._minimum_input_sequences[camera_id] = minimum_sequence
            synchronizer.begin_collection(
                minimum_sequence,
            )
        cameras = ", ".join(
            camera_id for _, camera_id in selected
        )
        self.feedback_message = (
            "Collecting reference-view inputs for "
            f"{cameras}, attempt {self._attempt_number}/"
            f"{self.capture_max_attempts}"
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
            self._prepare_attempt(current_time)
            return py_trees.common.Status.RUNNING
        return self._failure(RuntimeError(
            f"failed after {self.capture_max_attempts} attempts: "
            f"{reason}"
        ))

    def _prepare_attempt(self, current_time) -> None:
        for synchronizer in self._synchronizers.values():
            synchronizer.cancel_collection()
        self._minimum_input_sequences = {}
        self._warmup_started_nanoseconds = current_time.nanoseconds
        self._attempt_started_nanoseconds = None
        self._capture_validation_time = None
        self._collection_started = False
        self._attempt_tag_samples.clear()

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
            self._destroy_filter_subscription(
                getattr(synchronizer, "rgb_subscription", None)
            )
            self._destroy_filter_subscription(
                getattr(synchronizer, "depth_subscription", None)
            )
            if (
                self.node is not None
                and hasattr(self.node, "destroy_subscription")
            ):
                for attribute in (
                    "rgb_camera_info_subscription",
                    "depth_camera_info_subscription",
                ):
                    subscription = getattr(
                        synchronizer,
                        attribute,
                        None,
                    )
                    if subscription is not None:
                        self.node.destroy_subscription(subscription)
        self._synchronizers = {}
        self._minimum_input_sequences = {}
        self._capture_validation_time = None
        self._collection_started = False
        self._attempt_tag_samples.clear()

    def _destroy_filter_subscription(self, subscription) -> None:
        if subscription is None:
            return
        if hasattr(subscription, "unsubscribe"):
            subscription.unsubscribe()
            return
        underlying = getattr(subscription, "sub", None)
        if (
            underlying is not None
            and self.node is not None
            and hasattr(self.node, "destroy_subscription")
        ):
            self.node.destroy_subscription(underlying)
            return
        if (
            self.node is not None
            and hasattr(self.node, "destroy_subscription")
        ):
            self.node.destroy_subscription(subscription)

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
        maximum_input_age_sec,
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
        for value, name in (
            (maximum_input_age_sec, "Maximum input age"),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"{name} must be finite and non-negative"
                )
