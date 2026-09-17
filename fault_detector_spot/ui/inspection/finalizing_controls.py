"""Add server-owned probe finalization to inspection controls."""

import math

from fault_detector_msgs.msg import (
    ApplicationCommandState,
    OperationalIntent,
    ProbeSetupIntent,
)

from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.reference_camera_registry import (
    REFERENCE_CAMERAS,
    REFERENCE_CAMERA_BY_ID,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion,
)

from .controls import InspectionControls


class FinalizingInspectionControls(InspectionControls):
    """Route physical setup workflows through server-owned APIs."""

    def __init__(self, ui):
        super().__init__(ui)
        self._surface_test_active = False
        self._surface_move_succeeded = False
        self._surface_move_target_m = None
        self._refinement_emergency_stop_requested = False
        self._reference_preview_cache = {}
        self._reference_view_id_by_camera = {}
        self._configure_single_reference_view_ui()

    def _configure_single_reference_view_ui(self):
        """Keep one visible reference viewer while preserving legacy wiring."""
        self.reference_view_widget = self.reference_view_widgets[0]
        for slot_index in range(1, len(self.reference_view_widgets)):
            widget = self.reference_view_widgets[slot_index]
            dropdown = self.reference_camera_dropdowns[slot_index]
            widget.hide()
            dropdown.hide()
            parent = widget.parentWidget()
            if parent is not None:
                parent.hide()

    def _clear_reference_preview_cache(self):
        self._reference_preview_cache = {}
        self._reference_view_id_by_camera = {}

    def _selected_reference_camera_id(self):
        if not self.reference_camera_dropdowns:
            return ""
        return str(self.reference_camera_dropdowns[0].currentData() or "")

    def _set_reference_camera_defaults(self):
        if not self.reference_camera_dropdowns:
            return
        dropdown = self.reference_camera_dropdowns[0]
        dropdown.blockSignals(True)
        default_id = "hand" if dropdown.findData("hand") >= 0 else ""
        dropdown.setCurrentIndex(dropdown.findData(default_id))
        dropdown.blockSignals(False)
        for slot_index in range(1, len(self.reference_camera_dropdowns)):
            hidden_dropdown = self.reference_camera_dropdowns[slot_index]
            hidden_dropdown.blockSignals(True)
            hidden_dropdown.setCurrentIndex(hidden_dropdown.findData(""))
            hidden_dropdown.blockSignals(False)

    def handle_capture_reference_view(self):
        """Request the server-owned complete six-camera capture."""
        state = self._probe_setup_state
        if state is None or not state.selected_routine_id:
            self.show_warning(
                "Capture Reference View",
                "Select a saved object and routine first.",
            )
            return False
        self._clear_reference_preview_cache()
        legacy_camera_ids = tuple(
            camera.camera_id for camera in REFERENCE_CAMERAS[:3]
        )
        request_id = self.ui.execute_probe_reference_capture(
            legacy_camera_ids,
            replace_existing=(
                self.replace_reference_view_checkbox.isChecked()
            ),
        )
        if request_id is None:
            return False
        self.reference_view_status_label.setText(
            "Reference capture running"
        )
        self.reference_view_widget.clear_preview(
            "Capturing reference views"
        )
        return True

    def _reference_view_id_for_camera(self, camera_id):
        state = self._probe_setup_state
        if state is None:
            return ""
        for stored_camera_id, view_id in zip(
            state.reference_camera_ids,
            state.reference_view_ids,
        ):
            if stored_camera_id == camera_id:
                return view_id
        return ""

    def _render_reference_preview(self, response):
        region = ImageRegion(
            x=int(response.selectable_x),
            y=int(response.selectable_y),
            width=int(response.selectable_width),
            height=int(response.selectable_height),
        )
        widget = self.reference_view_widget
        widget.blockSignals(True)
        widget.set_ros_image(
            response.image,
            valid_region=region,
        )
        widget.blockSignals(False)
        self._reference_slot_view_ids[0] = response.reference_view_id
        self.reference_view_status_label.setText(
            "Reference view: remote preview ready"
        )
        self._restore_authoritative_selection()

    def _handle_reference_camera_selection_changed(self, slot_index):
        if slot_index != 0 or not self.reference_camera_dropdowns:
            return
        camera_id = self._selected_reference_camera_id()
        self._reference_slot_view_ids[0] = ""
        widget = self.reference_view_widget
        widget.blockSignals(True)
        widget.clear_selection()
        widget.blockSignals(False)

        if self._active_reference_slot == 0:
            self._active_reference_slot = None
            self._reference_rgb_size = None
            self._reference_depth_image = None
            self._reference_rgb_camera_info = None
            self._reference_camera_info = None
            self._reference_view = None
            self._handle_reference_image_point_cleared()

        if not camera_id:
            widget.clear_preview("No camera selected")
            return

        camera = REFERENCE_CAMERA_BY_ID[camera_id]
        view_id = self._reference_view_id_for_camera(camera_id)
        if not view_id:
            widget.clear_preview(
                f"{camera.display_name} not captured"
            )
            self.reference_view_status_label.setText(
                "Reference view: not captured"
            )
            return

        cached = self._reference_preview_cache.get(view_id)
        if cached is not None:
            self._render_reference_preview(cached)
            return

        client = getattr(self.ui, "probe_setup_client", None)
        if client is None:
            widget.clear_preview("Remote preview unavailable")
            self.reference_view_status_label.setText(
                "Reference view: remote preview unavailable"
            )
            return

        widget.clear_preview(f"Loading {camera.display_name}")
        self.reference_view_status_label.setText(
            "Reference view: loading preview"
        )
        client.request_preview(view_id)

    def _request_reference_previews(self, state):
        self._clear_reference_preview_cache()
        self._clear_reference_previews(
            "Loading reference preview"
        )
        self._reference_slot_view_ids = ["", "", ""]
        if not state.selected_routine_id:
            self.reference_view_status_label.setText(
                "Reference view: no routine selected"
            )
            self._set_reference_camera_defaults()
            return
        if not state.reference_view_ids:
            self.reference_view_status_label.setText(
                "Reference view: not captured"
            )
            self._set_reference_camera_defaults()
            self.reference_view_widget.clear_preview(
                "Capture a reference view"
            )
            return
        client = getattr(self.ui, "probe_setup_client", None)
        if client is None:
            self.reference_view_status_label.setText(
                "Reference view: remote preview unavailable"
            )
            return

        self._set_reference_camera_defaults()
        for camera_id, view_id in zip(
            state.reference_camera_ids,
            state.reference_view_ids,
        ):
            if not camera_id or not view_id:
                continue
            self._reference_view_id_by_camera[camera_id] = view_id
            client.request_preview(view_id)
        self._handle_reference_camera_selection_changed(0)

    def apply_reference_preview(self, response):
        state = self._probe_setup_state
        if (
            state is None
            or response.reference_view_id
            not in state.reference_view_ids
        ):
            return False

        self._reference_preview_cache[
            response.reference_view_id
        ] = response
        self._reference_view_id_by_camera[
            response.camera_id
        ] = response.reference_view_id

        current_camera_id = self._selected_reference_camera_id()
        if current_camera_id != response.camera_id:
            return True

        self._render_reference_preview(response)
        return True

    def handle_refinement_emergency_stop(self):
        self._refinement_emergency_stop_requested = True
        return super().handle_refinement_emergency_stop()

    def request_close_refinement_workflow(self):
        if self._refinement_emergency_stop_requested:
            if hasattr(self, "inspection_workspace_splitter"):
                self.inspection_workspace_splitter.setEnabled(True)
            self.start_probe_refinement_button.setText(
                "Resume Probe Point Position Refinement Workflow"
            )
            self.refinement_summary_status_label.setText(
                "Refinement paused after emergency stop."
            )
            return True

        presentation = self._refinement_presentation
        if presentation is None:
            return True
        if presentation.pending_motion is not None:
            self.refinement_recovery_status_label.setText(
                "Wait for the active movement and settle check."
            )
            return False
        if (
            presentation.recovery_required
            or self._distance_failure_requires_retraction
        ):
            self.refinement_recovery_status_label.setText(
                "Retract Without Saving is required before closing this "
                "workflow."
            )
            return False

        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_END_REFINEMENT
        if self._submit_probe_setup(intent) is None:
            return False
        if hasattr(self, "inspection_workspace_splitter"):
            self.inspection_workspace_splitter.setEnabled(True)
        self.start_probe_refinement_button.setText(
            "Resume Probe Point Position Refinement Workflow"
        )
        self.refinement_summary_status_label.setText(
            "Ending refinement workflow."
        )
        return True

    def resume_refinement_dialog(self):
        resumed = super().resume_refinement_dialog()
        if resumed:
            self._refinement_emergency_stop_requested = False
        return resumed

    def _finish_refinement_workflow_close(self):
        self._refinement_emergency_stop_requested = False
        result = super()._finish_refinement_workflow_close()
        if (
            hasattr(self, "refinement_dialog")
            and self.refinement_dialog.isVisible()
        ):
            self.refinement_dialog.close_after_completion()
        return result

    def handle_test_surface_distance(self):
        presentation = self._require_refinement_presentation()
        self._surface_test_active = False
        self._surface_move_succeeded = False
        self._surface_move_target_m = float(
            presentation.target_surface_distance_m
        )
        self._update_save_probe_point_state()

        intent = OperationalIntent()
        intent.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
        intent.target_surface_distance_m = float(
            presentation.target_surface_distance_m
        )
        intent.aligned_preapproach_distance_m = float(
            presentation.aligned_preapproach_distance_m
        )
        request_id = self.ui.execute_operation(intent)
        if request_id is None:
            self.surface_distance_test_status_label.setText(
                "Move close to surface is unavailable"
            )
            return False
        self._surface_test_active = True
        self.surface_distance_test_status_label.setText(
            "Move close to surface running"
        )
        return True

    def handle_application_state(self, status):
        """Track only this UI's standalone close-surface test command."""
        if (
            not self._surface_test_active
            or status.intent
            != OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
        ):
            return None
        client = getattr(self.ui, "application_client", None)
        if client is not None and status.client_id != client.client_id:
            return None

        states = {
            ApplicationCommandState.STATE_QUEUED: "Queued",
            ApplicationCommandState.STATE_DISPATCHED: "Dispatched",
            ApplicationCommandState.STATE_RUNNING: "Running",
            ApplicationCommandState.STATE_SUCCEEDED: "Reached",
            ApplicationCommandState.STATE_FAILED: "Failed",
            ApplicationCommandState.STATE_CANCELLED: "Cancelled",
        }
        label = states.get(status.state, "Surface movement")
        detail = status.detail.strip()
        self.surface_distance_test_status_label.setText(
            f"{label}: {detail}" if detail else label
        )
        if status.state == ApplicationCommandState.STATE_SUCCEEDED:
            self._surface_test_active = False
            self._surface_move_succeeded = True
            self._distance_failure_requires_retraction = False
        elif status.state in {
            ApplicationCommandState.STATE_FAILED,
            ApplicationCommandState.STATE_CANCELLED,
        }:
            self._surface_test_active = False
            self._surface_move_succeeded = False
            self._distance_failure_requires_retraction = (
                status.state == ApplicationCommandState.STATE_CANCELLED
                or "surface recovery" in detail.lower()
            )
        self._update_save_probe_point_state()
        return None

    def _surface_result_current(self):
        presentation = self._refinement_presentation
        if (
            not self._surface_move_succeeded
            or presentation is None
            or self._surface_move_target_m is None
        ):
            return False
        return math.isclose(
            presentation.target_surface_distance_m,
            self._surface_move_target_m,
            rel_tol=0.0,
            abs_tol=1e-6,
        )

    def _update_save_probe_point_state(self, _value=None):
        state = self._probe_setup_state
        point_id = self.probe_point_id_field.text().strip()
        display_name = self.probe_point_display_name_field.text().strip()
        numeric_ready = all(
            self._is_positive_number(field.text())
            for field in (
                self.probe_position_tolerance_field,
                self.probe_orientation_tolerance_field,
                self.probe_measurement_duration_field,
            )
        )
        duplicate = (
            state is not None
            and point_id in state.probe_point_ids
            and point_id != self._editing_probe_point_id
        )
        ready = False
        if state is None or not state.selected_routine_id:
            status = "Select a saved object and routine."
        elif not state.has_reference_pixel:
            status = "Select a point in a captured reference view."
        elif not point_id or not display_name:
            status = "Enter a probe point ID and display name."
        elif duplicate:
            status = f"Probe point '{point_id}' already exists."
        elif not numeric_ready:
            status = "Enter positive tolerances and measurement duration."
        elif not self._surface_result_current():
            status = "Run Move Close to Surface successfully before saving."
        elif state.motion_pending:
            status = "Wait for the active probe movement to finish."
        else:
            ready = True
            status = "Ready to approve the reached pose, save, and retract."
        self.approve_and_retract_button.setEnabled(ready)
        self.save_probe_point_status_label.setText(status)

    def _refresh_refinement_dialog(self):
        super()._refresh_refinement_dialog()
        presentation = self._refinement_presentation
        if presentation is None:
            self._surface_test_active = False
            self._surface_move_succeeded = False
            self._surface_move_target_m = None
            return
        if presentation.active_stage is not RefinementStage.PROBE:
            self._surface_move_succeeded = False
        pending = presentation.pending_motion is not None
        probe_page = presentation.active_stage is RefinementStage.PROBE
        alignment_reached = (
            presentation.motion_states[RefinementStage.ALIGNMENT]
            is RefinementMotionState.REACHED
        )
        retraction_required = (
            presentation.recovery_required
            or self._distance_failure_requires_retraction
        )
        self.retract_without_saving_button.setEnabled(
            not pending
            and (
                retraction_required
                or (probe_page and alignment_reached)
            )
        )
        self._update_save_probe_point_state()

    def handle_approve_and_retract(self):
        """Request server-owned save followed by mandatory retraction."""
        point_id = self._required_text(
            self.probe_point_id_field,
            "a probe point ID",
        )
        display_name = self._required_text(
            self.probe_point_display_name_field,
            "a probe point display name",
        )
        if point_id is None or display_name is None:
            return False
        try:
            position_tolerance = self._positive_value(
                self.probe_position_tolerance_field,
                "Position tolerance",
            )
            orientation_tolerance = self._positive_value(
                self.probe_orientation_tolerance_field,
                "Orientation tolerance",
            )
            measurement_duration = self._positive_value(
                self.probe_measurement_duration_field,
                "Measurement duration",
            )
        except ValueError as exception:
            self.show_warning("Finalize Probe Point", str(exception))
            return False
        request_id = self.ui.execute_probe_refinement_finalization(
            save_requested=True,
            probe_point_id=point_id,
            probe_point_display_name=display_name,
            position_tolerance_m=position_tolerance,
            orientation_tolerance_rad=orientation_tolerance,
            measurement_duration_sec=measurement_duration,
        )
        if request_id is None:
            return False
        self.approve_and_retract_button.setEnabled(False)
        self.retract_without_saving_button.setEnabled(False)
        self.save_probe_point_status_label.setText(
            "Saving probe point and retracting"
        )
        return True

    def handle_retract_without_saving(self):
        """Request server-owned retraction without persistence."""
        request_id = self.ui.execute_probe_refinement_finalization(
            save_requested=False,
        )
        if request_id is None:
            return False
        self.approve_and_retract_button.setEnabled(False)
        self.retract_without_saving_button.setEnabled(False)
        self.refinement_recovery_status_label.setText(
            "Retracting without saving"
        )
        return True

    @staticmethod
    def _positive_value(field, label):
        text = field.text().strip()
        try:
            value = float(text)
        except ValueError as exception:
            raise ValueError(f"{label} must be a number") from exception
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{label} must be positive")
        return value


__all__ = ["FinalizingInspectionControls"]
