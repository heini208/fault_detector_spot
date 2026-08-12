"""Coordinate server-owned synchronized probe reference capture."""

import math
import time
from dataclasses import dataclass
from enum import Enum
from threading import Event, RLock

import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time

from fault_detector_spot.inspection.setup.multi_reference_view_capture import (
    CameraCaptureRequest,
    capture_reference_views,
    validate_multi_reference_view_capture_target,
)
from fault_detector_spot.inspection.setup.reference_camera_registry import (
    REFERENCE_CAMERA_BY_ID,
    validate_reference_camera_slots,
)
from fault_detector_spot.inspection.setup.reference_view_input_synchronizer import (
    ReferenceViewInputSynchronizer,
)
from fault_detector_spot.inspection.setup.reference_view_validation import (
    ReferenceViewCaptureNotReady,
)


class ReferenceCaptureCancelled(RuntimeError):
    """Indicate that reference capture was cancelled or shut down."""


class ReferenceCapturePhase(str, Enum):
    """Observable phases of one reference capture transaction."""

    WAITING_FOR_INPUTS = "waiting_for_inputs"
    VALIDATING = "validating"
    SAVING = "saving"
    COMPLETE = "complete"


@dataclass(frozen=True)
class ProbeReferenceCaptureSpec:
    """Describe selected capture slots and replacement policy."""

    reference_camera_ids: tuple
    replace_existing: bool


class ProbeReferenceCaptureCoordinator:
    """Own synchronized collection, validation, persistence, and cleanup."""

    def __init__(
        self,
        node,
        probe_setup_coordinator,
        reference_repository,
        motion_state_source,
        queue_size=10,
        maximum_input_age_sec=1.5,
        maximum_timestamp_skew_sec=0.05,
        collection_duration_sec=1.0,
        capture_timeout_sec=3.0,
        capture_max_attempts=3,
        settle_duration_sec=0.5,
        fixed_frame="odom",
        transform_timeout_sec=0.05,
        poll_sec=0.05,
    ):
        self.node = node
        self.probe_setup_coordinator = probe_setup_coordinator
        self.reference_repository = reference_repository
        self.motion_state_source = motion_state_source
        self.queue_size = int(queue_size)
        self.maximum_input_age_sec = float(maximum_input_age_sec)
        self.maximum_timestamp_skew_sec = float(
            maximum_timestamp_skew_sec
        )
        self.collection_duration_sec = float(collection_duration_sec)
        self.capture_timeout_sec = float(capture_timeout_sec)
        self.capture_max_attempts = int(capture_max_attempts)
        self.settle_duration_sec = float(settle_duration_sec)
        self.fixed_frame = fixed_frame.strip()
        self.transform_timeout_sec = float(transform_timeout_sec)
        self.poll_sec = float(poll_sec)
        self._validate_configuration()
        self._lock = RLock()
        self._active_request_id = ""
        self._shutdown = Event()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            node,
        )

    def run(
        self,
        context,
        request_id,
        spec,
        cancel_requested,
        state_changed=None,
    ):
        """Run one complete reference capture transaction."""
        if not isinstance(spec, ProbeReferenceCaptureSpec):
            raise TypeError("Expected a ProbeReferenceCaptureSpec")
        if not callable(cancel_requested):
            raise TypeError("Cancellation predicate must be callable")
        if state_changed is not None and not callable(state_changed):
            raise TypeError("State listener must be callable")

        with self._lock:
            if self._active_request_id:
                raise RuntimeError("Reference capture is already in progress")
            self._active_request_id = request_id

        synchronizers = {}
        try:
            selected = validate_reference_camera_slots(
                spec.reference_camera_ids
            )
            snapshot = self.probe_setup_coordinator.snapshot(context)
            object_id = snapshot.selected_object_id
            routine_id = snapshot.selected_routine_id
            if not object_id or not routine_id:
                raise ValueError(
                    "Select a saved object and routine before capture"
                )
            tag_id = validate_multi_reference_view_capture_target(
                self.reference_repository,
                object_id,
                routine_id,
                spec.replace_existing,
            )

            self._require_command_lane_idle()
            self._wait_settled(cancel_requested)
            self.probe_setup_coordinator.setup_coordinator.require_current(
                context
            )
            self._require_command_lane_idle()

            synchronizers = self._create_synchronizers(selected)
            last_error = None
            for attempt in range(1, self.capture_max_attempts + 1):
                self._check_cancel(cancel_requested)
                self._emit(
                    state_changed,
                    ReferenceCapturePhase.WAITING_FOR_INPUTS,
                    attempt,
                    snapshot,
                )
                try:
                    self._wait_for_ready_inputs(
                        synchronizers,
                        selected,
                        tag_id,
                        cancel_requested,
                    )
                    minimum_sequences = self._begin_collections(
                        synchronizers,
                        selected,
                    )
                    self._wait_for_collections(
                        synchronizers,
                        selected,
                        cancel_requested,
                    )
                    self.probe_setup_coordinator.setup_coordinator.require_current(
                        context
                    )
                    self._require_command_lane_idle()
                    reference_tag = self._stable_reference_tag(tag_id)
                    self._require_tf_ready(
                        synchronizers,
                        selected,
                        reference_tag,
                    )
                    requests = tuple(
                        CameraCaptureRequest(
                            slot_index=slot_index,
                            camera_id=camera_id,
                            input_synchronizer=synchronizers[camera_id],
                            minimum_input_sequence=(
                                minimum_sequences[camera_id]
                            ),
                        )
                        for slot_index, camera_id in selected
                    )
                    self._emit(
                        state_changed,
                        ReferenceCapturePhase.VALIDATING,
                        attempt,
                        snapshot,
                    )
                    self._emit(
                        state_changed,
                        ReferenceCapturePhase.SAVING,
                        attempt,
                        snapshot,
                    )
                    capture_reference_views(
                        self.reference_repository,
                        requests,
                        self.tf_buffer,
                        object_id,
                        routine_id,
                        self.node.get_clock().now(),
                        tag_id,
                        (reference_tag,),
                        maximum_input_age_sec=(
                            self.maximum_input_age_sec
                        ),
                        maximum_timestamp_skew_sec=(
                            self.maximum_timestamp_skew_sec
                        ),
                        fixed_frame=self.fixed_frame,
                        transform_timeout_sec=(
                            self.transform_timeout_sec
                        ),
                    )
                    current = self.probe_setup_coordinator.context(
                        context.context_id,
                        context.client_id,
                    )
                    snapshot = self.probe_setup_coordinator.select_routine(
                        current,
                        object_id,
                        routine_id,
                    )
                    self._emit(
                        state_changed,
                        ReferenceCapturePhase.COMPLETE,
                        attempt,
                        snapshot,
                    )
                    return snapshot
                except (
                    ReferenceViewCaptureNotReady,
                    tf2_ros.TransformException,
                ) as exception:
                    last_error = exception
                    self._cancel_collections(synchronizers)
                    if attempt >= self.capture_max_attempts:
                        break
            raise RuntimeError(
                "Reference-view capture failed after "
                f"{self.capture_max_attempts} attempts: {last_error}"
            )
        finally:
            self._release_synchronizers(synchronizers)
            with self._lock:
                if self._active_request_id == request_id:
                    self._active_request_id = ""

    def _create_synchronizers(self, selected):
        values = {}
        for _, camera_id in selected:
            camera = REFERENCE_CAMERA_BY_ID[camera_id]
            values[camera_id] = ReferenceViewInputSynchronizer(
                node=self.node,
                rgb_topic=camera.rgb_topic,
                depth_topic=camera.depth_topic,
                rgb_camera_info_topic=camera.rgb_camera_info_topic,
                depth_camera_info_topic=camera.depth_camera_info_topic,
                queue_size=self.queue_size,
                maximum_timestamp_skew_sec=(
                    self.maximum_timestamp_skew_sec
                ),
                collection_duration_sec=self.collection_duration_sec,
            )
        return values

    def _wait_settled(self, cancel_requested):
        deadline = time.monotonic() + self.settle_duration_sec
        while time.monotonic() < deadline:
            self._check_cancel(cancel_requested)
            self._require_command_lane_idle()
            self._shutdown.wait(
                min(self.poll_sec, max(0.0, deadline - time.monotonic()))
            )

    def _wait_for_ready_inputs(
        self,
        synchronizers,
        selected,
        tag_id,
        cancel_requested,
    ):
        deadline = time.monotonic() + self.capture_timeout_sec
        last_detail = ""
        while time.monotonic() < deadline:
            self._check_cancel(cancel_requested)
            missing = [
                camera_id
                for _, camera_id in selected
                if not synchronizers[camera_id].ready_for_collection()
            ]
            tag = None
            tag_error = ""
            try:
                tag = self._stable_reference_tag(tag_id)
            except ReferenceViewCaptureNotReady as exception:
                tag_error = str(exception)
            tf_error = ""
            if not missing and not tag_error:
                try:
                    self._require_tf_ready(
                        synchronizers,
                        selected,
                        tag,
                    )
                except (
                    tf2_ros.TransformException,
                    ValueError,
                ) as exception:
                    tf_error = str(exception)
            if not missing and not tag_error and not tf_error:
                return
            details = [
                f"{camera_id}: "
                f"{synchronizers[camera_id].input_diagnostics()}"
                for camera_id in missing
            ]
            if tag_error:
                details.append(f"tag {tag_id}: {tag_error}")
            if tf_error:
                details.append(f"TF: {tf_error}")
            last_detail = ", ".join(details)
            self._shutdown.wait(self.poll_sec)
        raise ReferenceViewCaptureNotReady(
            "Selected capture inputs did not become ready: "
            f"{last_detail}"
        )

    def _begin_collections(self, synchronizers, selected):
        minimum_sequences = {}
        for _, camera_id in selected:
            synchronizer = synchronizers[camera_id]
            synchronizer.cancel_collection()
            minimum_sequence = synchronizer.input_sequence + 1
            minimum_sequences[camera_id] = minimum_sequence
            synchronizer.begin_collection(minimum_sequence)
        return minimum_sequences

    def _wait_for_collections(
        self,
        synchronizers,
        selected,
        cancel_requested,
    ):
        deadline = (
            time.monotonic()
            + self.collection_duration_sec
            + self.capture_timeout_sec
        )
        while time.monotonic() < deadline:
            self._check_cancel(cancel_requested)
            if all(
                synchronizers[camera_id].collection_ready()
                for _, camera_id in selected
            ):
                return
            self._shutdown.wait(self.poll_sec)
        raise ReferenceViewCaptureNotReady(
            "Reference-view collection window did not complete"
        )

    def _stable_reference_tag(self, tag_id):
        tag = self.motion_state_source.reference_tag(tag_id)
        if tag is None:
            raise ReferenceViewCaptureNotReady(
                f"No fresh stable base-camera observation is available "
                f"for reference tag {tag_id}"
            )
        return tag

    def _require_tf_ready(self, synchronizers, selected, reference_tag):
        timeout = Duration(seconds=self.transform_timeout_sec)
        self.tf_buffer.transform(
            reference_tag.pose,
            self.fixed_frame,
            timeout=timeout,
        )
        for _, camera_id in selected:
            frame_id = synchronizers[camera_id].latest_rgb_frame_id
            if not frame_id:
                raise ReferenceViewCaptureNotReady(
                    f"{camera_id}: RGB frame ID is not available"
                )
            self.tf_buffer.lookup_transform(
                self.fixed_frame,
                frame_id,
                Time(),
                timeout=timeout,
            )

    def _require_command_lane_idle(self):
        controller = (
            self.probe_setup_coordinator.setup_coordinator.command_controller
        )
        if controller.active_request_id:
            raise RuntimeError(
                "Reference capture requires the physical command lane to be idle"
            )
        if controller.queued_request_ids:
            raise RuntimeError(
                "Reference capture requires an empty physical command queue"
            )

    def _check_cancel(self, cancel_requested):
        if self._shutdown.is_set():
            raise ReferenceCaptureCancelled(
                "Reference capture stopped during shutdown"
            )
        if cancel_requested():
            raise ReferenceCaptureCancelled(
                "Reference capture was cancelled"
            )

    @staticmethod
    def _emit(listener, phase, attempt, snapshot):
        if listener is not None:
            listener(phase, attempt, snapshot)

    @staticmethod
    def _cancel_collections(synchronizers):
        for synchronizer in synchronizers.values():
            synchronizer.cancel_collection()

    def _release_synchronizers(self, synchronizers):
        for synchronizer in synchronizers.values():
            synchronizer.cancel_collection()
            for attribute in (
                "rgb_subscription",
                "depth_subscription",
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

    def _validate_configuration(self):
        if self.queue_size < 1:
            raise ValueError("Capture queue size must be positive")
        if self.capture_max_attempts < 1:
            raise ValueError(
                "Capture maximum attempts must be positive"
            )
        if not self.fixed_frame:
            raise ValueError("Capture fixed frame must not be empty")
        positive = (
            (self.collection_duration_sec, "Collection duration"),
            (self.capture_timeout_sec, "Capture timeout"),
            (self.settle_duration_sec, "Settle duration"),
            (self.poll_sec, "Polling interval"),
        )
        for value, name in positive:
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        non_negative = (
            (self.maximum_input_age_sec, "Maximum input age"),
            (
                self.maximum_timestamp_skew_sec,
                "Maximum timestamp skew",
            ),
            (self.transform_timeout_sec, "Transform timeout"),
        )
        for value, name in non_negative:
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"{name} must be finite and non-negative"
                )

    def close(self):
        """Stop active waits and detach capture TF observation."""
        self._shutdown.set()
        listener = self.tf_listener
        if listener is not None and hasattr(listener, "unregister"):
            listener.unregister()


__all__ = [
    "ProbeReferenceCaptureCoordinator",
    "ProbeReferenceCaptureSpec",
    "ReferenceCaptureCancelled",
    "ReferenceCapturePhase",
]
