"""Add server-owned probe finalization to inspection controls."""

import math

from fault_detector_spot.ui.sensor.models import SensorAttachmentViewStatus
from uuid import uuid4

from PyQt5.QtWidgets import QPushButton

from fault_detector_msgs.msg import (
    ApplicationCommandState,
    OperationalIntent,
    ProbeSetupIntent,
    ProbeSetupState,
)

from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose
from fault_detector_spot.shared.geometry.movement_frames import OrientationModes

from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)
from fault_detector_spot.inspection.setup.reference_camera_registry import (
    REFERENCE_CAMERA_BY_ID,
)
from fault_detector_spot.inspection.setup.reference_view_depth_projection import (
    ImageRegion,
)


from .controls import InspectionControls
from .saved_probe_controls import SavedProbeControls
from fault_detector_spot.ui.ros.probe_setup_client import (
    preloaded_reference_view_ids,
)


class FinalizingInspectionControls(InspectionControls):
    """Route physical setup workflows through server-owned APIs."""

    def __init__(self, ui):
        self._reference_start_pending = False
        self._reference_start_error = ""
        self._reset_sensor_tip_distance_tracking()
        self._reset_surface_move_tracking()
        self._refinement_emergency_stop_requested = False
        self._reference_preview_cache = {}
        self._reference_capture_in_progress = False
        self._abort_refinement_after_start = False
        self._base_position_operation_context = ""
        self._routine_arm_pose_operation_contexts = set()
        self._clear_pending_probe_finalization()
        super().__init__(ui)
        self.move_to_base_position_button.clicked.connect(
            self.handle_move_to_base_position
        )
        self.move_to_routine_arm_pose_button.clicked.connect(
            self.handle_move_to_routine_arm_pose
        )
        self.alignment_step_status_label.setWordWrap(True)
        self.refinement_dialog.attach_workflow_controls()
        self._configure_probe_point_entry_ui()

    def _reset_sensor_tip_distance_tracking(self):
        """Forget measurement feedback that no longer belongs to this setup."""
        self._sensor_tip_distance_test_active = False
        self._sensor_tip_distance_context_id = ""
        self._sensor_tip_distance_request_id = ""

    def _reset_surface_move_tracking(self):
        """Forget the surface movement and its previously verified result."""
        self._surface_test_active = False
        self._surface_move_succeeded = False
        self._surface_move_target_m = None
        self._surface_move_context_id = ""
        self._surface_move_setup_context_id = ""

    def _clear_pending_probe_finalization(self):
        self._probe_finalization_point_id = ""
        self._probe_finalization_scope = None

    def _configure_probe_point_entry_ui(self):
        """Build the probe-point controls in their execution tab."""

        self.saved_probe_controls = SavedProbeControls(
            submit_setup=self._submit_probe_setup,
            execute_operation=lambda intent, **kwargs: self.ui.execute_operation(intent, **kwargs),
            ask_question=self.ask_question,
            is_refinement_start_pending=lambda: self._reference_start_pending,
        )
        self.start_probe_refinement_button.setText(
            "+ Add"
        )

        self.abort_probe_refinement_button = QPushButton(
            "Abort Probe Point Setup"
        )
        self.abort_probe_refinement_button.setToolTip(
            "Discard the current unsaved probe-point setup without "
            "resuming it."
        )
        self.abort_probe_refinement_button.clicked.connect(
            self.handle_abort_probe_refinement
        )
        self.abort_probe_refinement_button.hide()
        self.saved_probe_controls.attach_refinement_controls(
            self.start_probe_refinement_button, self.abort_probe_refinement_button,
        )
        self.inspection_workspace_tabs.addTab(
            self.saved_probe_controls, "Probe Point Execution",
        )

    def handle_move_to_base_position(self):
        state = self._probe_setup_state
        if state is None:
            return False
        self._apply_base_position_state(state)
        if not self.move_to_base_position_button.isEnabled():
            return False

        intent = OperationalIntent()
        intent.intent = (
            OperationalIntent.INTENT_MOVE_TO_ROUTINE_BASE_POSITION
        )
        intent.object_id = state.selected_object_id
        intent.routine_id = state.selected_routine_id
        self._base_position_operation_context = (
            "routine-base-" + uuid4().hex
        )
        self._apply_base_position_state(state)
        request_id = self.ui.execute_operation(
            intent,
            context_id=self._base_position_operation_context,
        )
        if request_id is None:
            self.handle_base_position_rejected(
                "Movement could not be submitted."
            )
            return False
        return True

    def handle_base_position_rejected(self, _detail, context_id=None):
        if not self._base_position_operation_context or (
            context_id is not None
            and context_id != self._base_position_operation_context
        ):
            return
        self._base_position_operation_context = ""
        if self._probe_setup_state is not None:
            self._apply_base_position_state(self._probe_setup_state)

    def _handle_base_position_status(self, status):
        if (
            not self._base_position_operation_context
            or status.context_id != self._base_position_operation_context
        ):
            return
        if status.state in {
            ApplicationCommandState.STATE_SUCCEEDED,
            ApplicationCommandState.STATE_FAILED,
            ApplicationCommandState.STATE_CANCELLED,
            ApplicationCommandState.STATE_STOP_CONFIRMED,
        }:
            self._base_position_operation_context = ""
            if self._probe_setup_state is not None:
                self._apply_base_position_state(self._probe_setup_state)

    def handle_move_to_routine_arm_pose(self):
        state = self._probe_setup_state
        if state is None:
            return False
        self._apply_base_position_state(state)
        if not self.move_to_routine_arm_pose_button.isEnabled():
            return False

        intent = OperationalIntent()
        intent.intent = (
            OperationalIntent.INTENT_MOVE_TO_ROUTINE_SAFE_APPROACH
        )
        intent.object_id = state.selected_object_id
        intent.routine_id = state.selected_routine_id
        context_id = "routine-arm-" + uuid4().hex
        self._routine_arm_pose_operation_contexts.add(context_id)
        self._apply_base_position_state(state)
        request_id = self.ui.execute_operation(
            intent,
            context_id=context_id,
        )
        if request_id is None:
            self.handle_routine_arm_pose_rejected(
                "Movement could not be submitted.", context_id
            )
            return False
        return True

    def handle_routine_arm_pose_rejected(self, _detail, context_id):
        if context_id not in self._routine_arm_pose_operation_contexts:
            return
        self._routine_arm_pose_operation_contexts.remove(context_id)
        if self._probe_setup_state is not None:
            self._apply_base_position_state(self._probe_setup_state)

    def _handle_routine_arm_pose_status(self, status):
        if status.context_id not in self._routine_arm_pose_operation_contexts:
            return
        if status.state in {
            ApplicationCommandState.STATE_SUCCEEDED,
            ApplicationCommandState.STATE_FAILED,
            ApplicationCommandState.STATE_CANCELLED,
            ApplicationCommandState.STATE_STOP_CONFIRMED,
        }:
            self._routine_arm_pose_operation_contexts.remove(status.context_id)
            if self._probe_setup_state is not None:
                self._apply_base_position_state(self._probe_setup_state)

    def handle_operation_uncertain(self, detail, context_id):
        """Report lost transport feedback without releasing motion ownership."""
        text = f"Outcome unknown; awaiting confirmation: {detail}"
        self.saved_probe_controls.handle_operation_uncertain(detail, context_id)
        if (self._sensor_tip_distance_test_active
                and context_id == self._sensor_tip_distance_context_id):
            self.refinement_dialog.sensor_tip_distance_status_label.setText(text)
        if self._surface_test_active and context_id == self._surface_move_context_id:
            self.surface_distance_test_status_label.setText(text)
        self.routine_navigation_controls.handle_operation_uncertain(detail, context_id)

    def handle_start_probe_refinement(self):
        """Open reference selection before beginning physical refinement."""
        if self._refinement_presentation is not None:
            return self.resume_refinement_dialog()
        state = self._probe_setup_state
        if state is None or not state.selected_routine_id:
            self.show_warning(
                "Add Probe Point",
                "Select a saved object and routine first.",
            )
            return False
        attachment = getattr(self.ui, "_sensor_attachment_state", None)
        if attachment is None or attachment.status is not SensorAttachmentViewStatus.ACTIVE:
            self.show_warning(
                "Add Probe Point",
                "Confirm the sensor attachment in the sensor controls before "
                "adding a probe point. If using the bare hand, select and "
                "confirm No sensor.",
            )
            return False
        if not state.has_routine_safe_approach_pose:
            self.show_warning(
                "Add Probe Point",
                "Set the routine safe pre-approach arm pose beside the base "
                "pose before adding a probe point.",
            )
            return False
        self.refinement_dialog.open_mode_selection()
        return True

    def handle_probe_mode_selected(self, fully_custom):
        state = self._probe_setup_state
        self.refinement_dialog.fully_custom = fully_custom
        self._reference_start_pending = False
        self._reference_start_error = ""
        self.inspection_workspace_tabs.setEnabled(False)
        if self.refinement_dialog.fully_custom:
            self.refinement_dialog.show()
            return self._start_refinement_after_reference()
        self.refinement_dialog.open_reference_selection(
            has_existing=bool(state.reference_view_ids)
        )
        if state.reference_view_ids:
            self._request_reference_previews(state)
        return True

    def handle_use_existing_reference_views(self):
        state = self._probe_setup_state
        if state is None or not state.reference_view_ids:
            self.refinement_dialog.set_reference_status(
                "No saved reference views are available."
            )
            return False
        self._request_reference_previews(state)
        self.refinement_dialog.enable_reference_selection()
        return True

    def handle_capture_new_reference_views(self):
        self.replace_reference_view_checkbox.setChecked(False)
        return self.handle_capture_reference_view()

    def handle_retake_reference_views(self):
        self.replace_reference_view_checkbox.setChecked(True)
        return self.handle_capture_reference_view()

    def handle_reference_point_approved(self):
        """Start refinement from an already validated saved reference point."""
        if self._reference_start_pending:
            return False
        state = self._probe_setup_state
        if state is None:
            self.refinement_dialog.set_reference_status(
                "Reference-point state is unavailable."
            )
            return False
        if not self._reference_state_matches_draft():
            self.refinement_dialog.set_reference_status(
                "Wait for the selected point to be checked against the saved "
                "depth image."
            )
            return False
        if not state.has_surface_point:
            detail = (
                state.validation_error.strip()
                or state.detail.strip()
                or "No valid registered depth is available at this point."
            )
            self.refinement_dialog.set_reference_status(
                f"Invalid point: {detail}"
            )
            return False
        if not state.has_probe_setup:
            detail = (
                state.validation_error.strip()
                or state.detail.strip()
                or "Probe geometry could not be calculated for this point."
            )
            self.refinement_dialog.set_reference_status(
                f"Point cannot be used: {detail}"
            )
            return False
        return self._start_refinement_after_reference()

    def _reference_state_matches_draft(self):
        state = self._probe_setup_state
        point = self.reference_view_widget.selected_image_point
        if state is None or point is None:
            return False
        return (
            bool(state.has_reference_pixel)
            and state.selected_reference_view_id
            == self._reference_slot_view_ids[0]
            and int(state.reference_pixel_u) == int(point.u)
            and int(state.reference_pixel_v) == int(point.v)
        )

    def _start_refinement_after_reference(self):
        if self._reference_start_pending:
            return False
        self.refinement_dialog.show_stage(RefinementStage.SAFE_APPROACH)
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_BEGIN_REFINEMENT
        intent.fully_custom = self.refinement_dialog.fully_custom
        self._reference_start_error = ""
        self._reference_start_pending = True
        self.refinement_dialog.refresh_reference_selection()
        self.refinement_dialog.refresh_refinement_start()
        request_id = self._submit_probe_setup(intent)
        if request_id is None:
            self.handle_reference_start_rejected(
                "Could not submit probe refinement. Check that the setup "
                "service is available and no other request is pending."
            )
            return False
        return True

    def handle_reference_start_rejected(self, detail):
        if not self._reference_start_pending:
            return
        self._reference_start_pending = False
        self._reference_start_error = detail
        self.refinement_dialog.refresh_reference_selection()
        self.refinement_dialog.refresh_refinement_start()

    def apply_setup_state(self, state):
        if (
            self._reference_start_pending
            and state.operation == ProbeSetupIntent.OPERATION_BEGIN_REFINEMENT
            and state.state in (
                ProbeSetupState.STATE_SUCCEEDED,
                ProbeSetupState.STATE_FAILED,
            )
        ):
            self._reference_start_pending = False
            self._reference_start_error = (
                state.detail or "Probe refinement could not start."
                if state.state == ProbeSetupState.STATE_FAILED else ""
            )
        result = super().apply_setup_state(state)

        if self._abort_refinement_after_start:
            if (
                state.operation
                == ProbeSetupIntent.OPERATION_BEGIN_REFINEMENT
                and state.state == ProbeSetupState.STATE_FAILED
            ):
                self._abort_refinement_after_start = False
            elif state.refinement_active:
                intent = ProbeSetupIntent()
                intent.operation = (
                    ProbeSetupIntent.OPERATION_END_REFINEMENT
                )
                if self._submit_probe_setup(intent) is not None:
                    self._abort_refinement_after_start = False

        self._update_abort_probe_refinement_state()

        self.saved_probe_controls.apply_setup_state(state)

        if self._refinement_presentation is None:
            self.start_probe_refinement_button.setText(
                "+ Add"
            )
            self.start_probe_refinement_button.setEnabled(
                bool(state.selected_routine_id)
                and not bool(state.motion_pending)
            )

        if hasattr(self, "refinement_dialog"):
            self.refinement_dialog.update_reference_availability(
                bool(state.reference_view_ids)
            )
            self.refinement_dialog.refresh_reference_selection()
            self.refinement_dialog.refresh_refinement_start()

        if self._reference_capture_in_progress:
            if (
                state.state == ProbeSetupState.STATE_SUCCEEDED
                and state.reference_view_ids
            ):
                self._reference_capture_in_progress = False
                self.refinement_dialog.reference_views_ready()
            elif state.state in (
                ProbeSetupState.STATE_FAILED,
                ProbeSetupState.STATE_CANCELLED,
            ):
                self._reference_capture_in_progress = False
                self.refinement_dialog.update_reference_availability(
                    bool(state.reference_view_ids)
                )
                detail = state.detail or "Reference capture failed."
                if state.reference_view_ids:
                    detail += "\nSaved reference views remain visible."
                self.refinement_dialog.set_reference_status(
                    detail
                )

        return result

    def _clear_reference_preview_cache(self):
        self._reference_preview_cache = {}

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
        self._reference_capture_in_progress = True
        request_id = self.ui.execute_probe_reference_capture(
            replace_existing=(
                self.replace_reference_view_checkbox.isChecked()
            ),
        )
        if request_id is None:
            self._reference_capture_in_progress = False
            return False
        self.reference_view_status_label.setText(
            "Reference capture running"
        )
        self.reference_view_widget.clear_preview(
            "Capturing reference views"
        )
        self.refinement_dialog.set_reference_capture_running()
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
            source_region=region,
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

        state = self._probe_setup_state
        had_reference_pixel = bool(
            state is not None and state.has_reference_pixel
        )

        camera_id = self._selected_reference_camera_id()
        self._reference_slot_view_ids[0] = ""
        widget = self.reference_view_widget
        widget.blockSignals(True)
        widget.clear_selection()
        widget.blockSignals(False)

        self._active_reference_slot = None
        self._handle_reference_image_point_cleared()

        if had_reference_pixel:
            intent = ProbeSetupIntent()
            intent.operation = (
                ProbeSetupIntent.OPERATION_CLEAR_REFERENCE_PIXEL
            )
            self._submit_probe_setup(intent)

        self.refinement_dialog.reference_selection_changed()

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
                "Capture reference views to continue"
            )
            self.refinement_dialog.update_reference_availability(False)
            return
        client = getattr(self.ui, "probe_setup_client", None)
        if client is None:
            self.reference_view_status_label.setText(
                "Reference view: remote preview unavailable"
            )
            return

        self._set_reference_camera_defaults()
        for view_id in preloaded_reference_view_ids(state):
            client.request_preview(view_id)

        self.refinement_dialog.update_reference_availability(True)

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
        current_camera_id = self._selected_reference_camera_id()
        if current_camera_id == response.camera_id:
            self._render_reference_preview(response)
            self.refinement_dialog.refresh_reference_selection()
        return True

    def handle_open_probe_summary(self):
        presentation = self._refinement_presentation
        if presentation is None:
            return False
        if presentation.active_stage is not RefinementStage.PROBE:
            return False
        if not self._can_open_probe_summary():
            self.surface_distance_test_status_label.setText(
                "Reach the requested surface distance before continuing."
            )
            return False
        self.refinement_dialog.show_summary()
        return True

    def _can_open_probe_summary(self):
        presentation = self._refinement_presentation
        return bool(
            presentation is not None
            and presentation.active_stage is RefinementStage.PROBE
            and not self._refinement_has_pending_work(presentation)
            and not presentation.recovery_required
            and not self._distance_failure_requires_retraction
            and not self._surface_test_active
            and self._surface_result_current()
            and (
                self._custom_probe_mode()
                or presentation.motion_states[RefinementStage.ALIGNMENT]
                is RefinementMotionState.REACHED
            )
        )

    def _can_continue_refinement(self, presentation):
        """Use the verified surface result when leaving the probe stage."""
        if presentation.active_stage is RefinementStage.PROBE:
            return self._can_open_probe_summary()
        return super()._can_continue_refinement(presentation)

    def _can_close_refinement_workflow(self, presentation):
        """Keep the dialog open while an accepted save is retracting."""
        return bool(
            not self._refinement_has_pending_work(presentation)
            and (
                not self._probe_finalization_point_id
                or presentation.recovery_required
            )
        )

    def _can_abort_probe_refinement(self):
        """Keep movement, measurement, and finalization owned until they finish."""
        return bool(
            not self._sensor_tip_distance_test_active
            and not self._surface_test_active
            and not self._probe_finalization_point_id
        )

    def _update_abort_probe_refinement_state(self):
        refinement_active = self._refinement_presentation is not None
        can_abort = self._can_abort_probe_refinement()
        self.abort_probe_refinement_button.setVisible(refinement_active)
        self.abort_probe_refinement_button.setEnabled(refinement_active and can_abort)
        self.refinement_dialog.set_abort_enabled(can_abort)

    def _can_retract_without_saving(self, presentation):
        """Allow retraction from recovery or the reached probe stage."""
        return bool(
            not self._refinement_has_pending_work(presentation)
            and (
                presentation.recovery_required
                or self._distance_failure_requires_retraction
                or (
                    presentation.active_stage is RefinementStage.PROBE
                    and presentation.motion_states[RefinementStage.ALIGNMENT]
                    is RefinementMotionState.REACHED
                )
            )
        )

    def _handle_refinement_stage_changed(self, stage):
        super()._handle_refinement_stage_changed(stage)
        if not self._custom_probe_mode() and stage is not RefinementStage.PROBE:
            self._surface_move_succeeded = False

    def handle_refinement_back(self):
        if self.refinement_dialog.is_summary_page():
            self.refinement_dialog.show_stage(RefinementStage.PROBE)
            return True
        return super().handle_refinement_back()

    def handle_refinement_emergency_stop(self):
        self._refinement_emergency_stop_requested = True
        return super().handle_refinement_emergency_stop()

    def handle_abort_probe_refinement(self):
        """Abort the current unsaved probe-point setup."""
        if not self._can_abort_probe_refinement():
            self.refinement_recovery_status_label.setText(
                "Wait for the movement or distance test to finish or use Software Stop "
                "before aborting setup."
                if self._sensor_tip_distance_test_active or self._surface_test_active
                else "Probe-point saving is already in progress and cannot be aborted here."
            )
            return False

        if self._reference_start_pending:
            self._abort_refinement_after_start = True
            self.refinement_recovery_status_label.setText(
                "Abort requested. Waiting for refinement startup to finish."
            )
            if self.refinement_dialog.isVisible():
                self.refinement_dialog.close_after_completion()
            if hasattr(self, "inspection_workspace_tabs"):
                self.inspection_workspace_tabs.setEnabled(True)
            return True

        if self._refinement_presentation is None:
            self._abort_refinement_after_start = False
            if hasattr(self, "inspection_workspace_tabs"):
                self.inspection_workspace_tabs.setEnabled(True)
            self.start_probe_refinement_button.setText(
                "+ Add"
            )
            self.abort_probe_refinement_button.hide()
            if self.refinement_dialog.isVisible():
                self.refinement_dialog.close_after_completion()
            return True

        if not self.ask_question(
            "Abort Probe Point Setup",
            (
                "Discard the current unsaved probe-point setup? "
                "Any active setup movement will be cancelled."
            ),
        ):
            return False

        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_END_REFINEMENT
        request_id = self._submit_probe_setup(intent)
        if request_id is None:
            self.refinement_recovery_status_label.setText(
                "Probe-point setup could not be aborted."
            )
            return False

        self.abort_probe_refinement_button.setEnabled(False)
        self.refinement_dialog.set_abort_enabled(False)
        self.refinement_recovery_status_label.setText(
            "Aborting probe-point setup."
        )
        return True

    def request_close_refinement_workflow(self):
        """Pause the UI without discarding server-owned refinement state."""
        presentation = self._refinement_presentation
        if presentation is None:
            if hasattr(self, "inspection_workspace_tabs"):
                self.inspection_workspace_tabs.setEnabled(True)
            self.start_probe_refinement_button.setText(
                "+ Add"
            )
            return True

        if not self._can_close_refinement_workflow(presentation):
            if self._refinement_has_pending_work(presentation):
                self.refinement_recovery_status_label.setText(
                    "Wait for the active movement and settle check before "
                    "closing the setup window."
                )
            else:
                self.refinement_recovery_status_label.setText(
                    "Wait for probe-point saving and mandatory retraction "
                    "to finish."
                )
            return False

        if hasattr(self, "inspection_workspace_tabs"):
            self.inspection_workspace_tabs.setEnabled(True)

        self.start_probe_refinement_button.setText(
            "Resume Probe Point Setup"
        )

        if (
            presentation.recovery_required
            or self._distance_failure_requires_retraction
        ):
            self.refinement_summary_status_label.setText(
                "Probe-point setup paused in recovery. Resume it to "
                "continue recovery, or use Abort Probe Point Setup to "
                "discard it."
            )
        elif self._refinement_emergency_stop_requested:
            self.refinement_summary_status_label.setText(
                "Probe-point setup paused after software stop. Resume it "
                "to continue, or abort it from the main panel."
            )
        else:
            self.refinement_summary_status_label.setText(
                "Probe-point setup paused. Resume to continue from the "
                "current stage."
            )

        return True

    def resume_refinement_dialog(self):
        resumed = super().resume_refinement_dialog()
        if resumed:
            self._refinement_emergency_stop_requested = False
            self.start_probe_refinement_button.setText(
                "Resume Probe Point Setup"
            )
        return resumed

    def _finish_refinement_workflow_close(self):
        point_id = self._probe_finalization_point_id
        scope = self._probe_finalization_scope
        state = self._probe_setup_state
        if (
            point_id
            and scope is not None
            and state is not None
            and scope == (
                state.selected_object_id,
                state.selected_routine_id,
            )
            and point_id in state.probe_point_ids
        ):
            self.saved_probe_controls.select_after_finalization(
                scope[0], scope[1], point_id,
            )
        self._clear_pending_probe_finalization()
        self._refinement_emergency_stop_requested = False
        result = super()._finish_refinement_workflow_close()
        self.start_probe_refinement_button.setText(
            "+ Add"
        )
        if (
            hasattr(self, "refinement_dialog")
            and self.refinement_dialog.isVisible()
        ):
            self.refinement_dialog.close_after_completion()
        return result

    def handle_test_sensor_tip_distance(self):
        """Request a measured distance and verified return through the command API."""
        presentation = self._require_refinement_presentation()
        state = self._probe_setup_state
        if (state is None or not state.context_id
                or presentation.active_stage is not RefinementStage.ALIGNMENT
                or self._refinement_has_pending_work(presentation)
                or presentation.recovery_required
                or presentation.motion_states[RefinementStage.ALIGNMENT]
                is not RefinementMotionState.REACHED):
            return False
        self._sensor_tip_distance_context_id = (
            state.context_id + "/measure-sensor-tip-distance/" + uuid4().hex
        )
        self._sensor_tip_distance_request_id = ""
        self._sensor_tip_distance_test_active = True
        label = self.refinement_dialog.sensor_tip_distance_status_label
        label.setText("Measuring sensor tip distance; arm may move and return.")
        self._refresh_refinement_dialog()
        intent = OperationalIntent()
        intent.intent = OperationalIntent.INTENT_MEASURE_SENSOR_TIP_DISTANCE
        intent.aligned_preapproach_distance_m = float(
            presentation.aligned_preapproach_distance_m
        )
        submitted = self.ui.execute_operation(
            intent, context_id=self._sensor_tip_distance_context_id,
        )
        if submitted is None:
            self.handle_sensor_tip_distance_rejected(
                "Measurement could not be submitted.",
                self._sensor_tip_distance_context_id,
            )
            return False
        return True

    def handle_sensor_tip_distance_rejected(self, detail, context_id):
        if (not self._sensor_tip_distance_test_active
                or context_id != self._sensor_tip_distance_context_id):
            return
        self._sensor_tip_distance_test_active = False
        if self._sensor_tip_distance_request_id:
            # Losing the result after acceptance leaves the return unverified.
            self._surface_move_succeeded = False
            self._distance_failure_requires_retraction = True
        self.refinement_dialog.sensor_tip_distance_status_label.setText(
            f"Measurement unavailable: {detail}"
        )
        self._refresh_refinement_dialog()

    def _handle_sensor_tip_distance_status(self, status):
        if (not self._sensor_tip_distance_test_active
                or status.intent != OperationalIntent.INTENT_MEASURE_SENSOR_TIP_DISTANCE
                or status.context_id != self._sensor_tip_distance_context_id):
            return
        client = getattr(self.ui, "application_client", None)
        if client is not None and status.client_id != client.client_id:
            return
        if (self._sensor_tip_distance_request_id
                and status.request_id != self._sensor_tip_distance_request_id):
            return
        self._sensor_tip_distance_request_id = status.request_id
        self.refinement_dialog.sensor_tip_distance_status_label.setText(
            status.detail.strip() or {
                ApplicationCommandState.STATE_UNKNOWN: "Measurement outcome unknown; reconciling",
                ApplicationCommandState.STATE_STOP_REQUESTED: "Stop requested; awaiting confirmation",
                ApplicationCommandState.STATE_STOP_CONFIRMED: "Measurement stop confirmed",
                ApplicationCommandState.STATE_STOP_UNCONFIRMED: "Measurement stop unconfirmed",
            }.get(status.state, "Measuring sensor tip distance")
        )
        if status.state in {
            ApplicationCommandState.STATE_SUCCEEDED,
            ApplicationCommandState.STATE_FAILED,
            ApplicationCommandState.STATE_CANCELLED,
            ApplicationCommandState.STATE_STOP_CONFIRMED,
        }:
            self._sensor_tip_distance_test_active = False
            if (status.state != ApplicationCommandState.STATE_SUCCEEDED
                    and not (
                        status.state == ApplicationCommandState.STATE_FAILED
                        and status.continuation_safe
                    )):
                self._surface_move_succeeded = False
                self._distance_failure_requires_retraction = True
        self._refresh_refinement_dialog()

    def handle_test_surface_distance(self):
        presentation = self._require_refinement_presentation()
        if self._refinement_has_pending_work(presentation):
            return False
        state = self._probe_setup_state
        if state is None or not state.context_id:
            self.surface_distance_test_status_label.setText(
                "Probe setup context is unavailable"
            )
            return False
        self._surface_test_active = True
        self._surface_move_succeeded = False
        self._surface_move_setup_context_id = state.context_id
        context_id = state.context_id + "/move-close-to-surface/" + uuid4().hex
        self._surface_move_context_id = context_id
        self._surface_move_target_m = float(
            presentation.target_surface_distance_m
        )
        self.surface_distance_test_status_label.setText(
            "Move close to surface running"
        )
        self._refresh_refinement_dialog()

        intent = OperationalIntent()
        intent.intent = OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
        intent.target_surface_distance_m = float(
            presentation.target_surface_distance_m
        )
        intent.aligned_preapproach_distance_m = float(
            presentation.aligned_preapproach_distance_m
        )
        candidate = presentation.candidate_pose(RefinementStage.ALIGNMENT)
        if (candidate is not None
                and presentation.active_stage in (RefinementStage.ALIGNMENT, RefinementStage.PROBE)
                and presentation.motion_states[RefinementStage.ALIGNMENT]
                is RefinementMotionState.REACHED):
            intent.tag.id = state.selected_reference_tag_id
            intent.offset.header.frame_id = f"filtered_fiducial_{intent.tag.id}"
            intent.offset.pose = pose_data_to_pose(candidate)
            intent.orientation_mode = OrientationModes.CUSTOM_ORIENTATION.value
            intent.tag_position_tolerance_m = state.aligned_position_tolerance_m
            intent.precision_mode = state.aligned_precision_mode
        # The scoped context identifies this attempt before feedback or rejection
        # can arrive, while preserving its association with the wizard.
        submitted = self.ui.execute_operation(
            intent, context_id=context_id,
        )
        if submitted is None:
            self.handle_surface_move_rejected(
                "Movement could not be submitted.", context_id,
            )
            return False
        return self._surface_move_context_id == context_id

    def handle_surface_move_rejected(self, detail, context_id):
        """Release only the surface attempt confirmed not to have been accepted."""
        if not self._surface_test_active or context_id != self._surface_move_context_id:
            return
        self._reset_surface_move_tracking()
        self.surface_distance_test_status_label.setText(
            f"Move close to surface unavailable: {detail}"
        )
        self._refresh_refinement_dialog()

    def handle_application_state(self, status):
        """Track inspection movements and standalone close-surface tests."""
        super().handle_application_state(status)
        self._handle_base_position_status(status)
        self._handle_routine_arm_pose_status(status)
        self.saved_probe_controls.handle_application_state(status)
        self._handle_sensor_tip_distance_status(status)
        if (
            not self._surface_test_active
            or status.intent
            != OperationalIntent.INTENT_MOVE_CLOSE_TO_SURFACE
            or status.context_id != self._surface_move_context_id
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
            ApplicationCommandState.STATE_UNKNOWN: "Outcome unknown; reconciling",
            ApplicationCommandState.STATE_STOP_REQUESTED: "Stop requested",
            ApplicationCommandState.STATE_STOP_CONFIRMED: "Stop confirmed",
            ApplicationCommandState.STATE_STOP_UNCONFIRMED: "Stop unconfirmed",
        }
        label = states.get(status.state, "Surface movement")
        detail = status.detail.strip()
        self.surface_distance_test_status_label.setText(
            f"{label}: {detail}" if detail else label
        )
        if status.state == ApplicationCommandState.STATE_SUCCEEDED:
            self._surface_test_active = False
            presentation = self._refinement_presentation
            self._surface_move_succeeded = bool(
                presentation is not None
                and presentation.active_stage is RefinementStage.PROBE
            )
            self._distance_failure_requires_retraction = False
        elif status.state in {
            ApplicationCommandState.STATE_FAILED,
            ApplicationCommandState.STATE_CANCELLED,
            ApplicationCommandState.STATE_STOP_CONFIRMED,
        }:
            self._surface_test_active = False
            self._surface_move_succeeded = False
            self._distance_failure_requires_retraction = (
                status.state in {
                    ApplicationCommandState.STATE_CANCELLED,
                    ApplicationCommandState.STATE_STOP_CONFIRMED,
                }
                or "surface recovery" in detail.lower()
            )
        self._update_save_probe_point_state()
        self._refresh_refinement_dialog()
        return None

    def _surface_result_current(self):
        if self._custom_probe_mode():
            presentation = self._refinement_presentation
            return bool(presentation and presentation.stage_is_approved(RefinementStage.PROBE)
                        and presentation.motion_states[RefinementStage.PROBE] is RefinementMotionState.REACHED)
        presentation = self._refinement_presentation
        if (
            not self._surface_move_succeeded
            or presentation is None
            or self._surface_move_target_m is None
            or self._probe_setup_state is None
            or self._probe_setup_state.context_id != self._surface_move_setup_context_id
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
        finalizing_current = bool(
            point_id
            and point_id == self._probe_finalization_point_id
        )
        saved_during_finalization = bool(
            finalizing_current
            and state is not None
            and point_id in state.probe_point_ids
        )
        duplicate = (
            state is not None
            and point_id in state.probe_point_ids
            and point_id != self._editing_probe_point_id
        )
        ready = False
        if finalizing_current:
            if state is not None and state.refinement_recovery_required:
                detail = (
                    state.refinement_recovery_message.strip()
                    or state.detail.strip()
                )
                status = (
                    "Probe point saved, but mandatory retraction requires "
                    "recovery."
                    if saved_during_finalization
                    else "Probe-point finalization requires recovery."
                )
                if detail:
                    status = f"{status} {detail}"
            elif saved_during_finalization:
                status = (
                    "Probe point saved. Mandatory retraction in progress."
                )
            else:
                status = "Saving probe point and retracting."
        elif state is None or not state.selected_routine_id:
            status = "Select a saved object and routine."
        elif not state.has_reference_pixel and not state.fully_custom:
            status = "Select a point in a captured reference view."
        elif not point_id or not display_name:
            status = "Enter a probe point ID and display name."
        elif duplicate:
            status = f"Probe point '{point_id}' already exists."
        elif not numeric_ready:
            status = "Enter positive tolerances and measurement duration."
        elif not self._can_open_probe_summary():
            status = ("Reach and save the final probe candidate before saving."
                      if state.fully_custom else
                      "Run Move Close to Surface successfully before saving.")
        elif state.motion_pending:
            status = "Wait for the active probe movement to finish."
        else:
            ready = True
            status = "Ready to approve the reached pose, save, and retract."
        self.approve_and_retract_button.setEnabled(ready)
        self.save_probe_point_status_label.setText(status)

    def _refresh_alignment_depth_status(self):
        if self._custom_probe_mode():
            return
        presentation = self._refinement_presentation
        state = self._probe_setup_state
        if presentation is None or state is None:
            return

        motion_state = presentation.motion_states[RefinementStage.ALIGNMENT]
        movement_label = self.alignment_step_status_label
        movement_label.setToolTip("")
        aligned_distance = presentation.aligned_preapproach_distance_m
        distance_text = (
            f"Configured aligned probe-to-surface distance: "
            f"{aligned_distance:.3f} m."
        )

        if presentation.stage_is_approved(RefinementStage.ALIGNMENT):
            movement_label.setText("Reached, registered depth verified")
            detail = (
                "Depth readiness for Move Close to Wall: READY. "
                f"{distance_text} Live registered hand depth at the "
                "reached aligned pose verified usable camera clearance."
            )
            self._set_depth_readiness_status(detail)
            movement_label.setToolTip(detail)
            return

        if (
            state.operation
            == ProbeSetupIntent.OPERATION_APPROVE_ALIGNED_POSE
            and state.state == ProbeSetupState.STATE_FAILED
        ):
            detail = (
                state.detail.strip()
                or state.validation_error.strip()
                or "Live registered hand depth could not verify camera clearance."
            )
            movement_label.setText(
                f"{motion_state.value}, registered depth NOT verified"
            )
            readiness = (
                "Depth readiness for Move Close to Wall: NOT READY. "
                f"{distance_text} {detail}"
            )
            self._set_depth_readiness_status(readiness)
            movement_label.setToolTip(detail)
            return

        if motion_state is RefinementMotionState.REACHED:
            movement_label.setText("Reached, depth check pending approval")
            detail = (
                "Depth readiness for Move Close to Wall: CHECK REQUIRED. "
                f"{distance_text} Press Approve Current Pose to verify "
                "live registered hand depth at the reached pose."
            )
            self._set_depth_readiness_status(detail)
            movement_label.setToolTip(detail)
            return

        movement_label.setText(motion_state.value)
        self._set_depth_readiness_status(
            "Depth readiness for Move Close to Wall: NOT VERIFIED. "
            f"{distance_text} Reach the aligned candidate and approve the "
            "current pose to verify live registered hand depth."
        )

    def _set_depth_readiness_status(self, text):
        for name in (
            "alignment_depth_readiness_label",
            "probe_depth_readiness_label",
        ):
            label = getattr(self.refinement_dialog, name, None)
            if label is not None:
                label.setText(text)
                label.setToolTip(text)

    def _synchronize_refinement_presentation(self, presentation):
        """Discard stale operation tracking before rendering a setup snapshot."""
        state = self._probe_setup_state
        if presentation is None:
            self._reset_sensor_tip_distance_tracking()
            self._reset_surface_move_tracking()
        elif state is not None:
            if (
                self._sensor_tip_distance_context_id
                and not self._sensor_tip_distance_context_id.startswith(
                    state.context_id + "/measure-sensor-tip-distance/"
                )
            ):
                self._reset_sensor_tip_distance_tracking()
            if (
                self._surface_move_setup_context_id
                and self._surface_move_setup_context_id != state.context_id
            ):
                self._reset_surface_move_tracking()
        super()._synchronize_refinement_presentation(presentation)

    def _refresh_refinement_dialog(self):
        super()._refresh_refinement_dialog()
        self._update_abort_probe_refinement_state()
        if self._custom_probe_mode():
            return
        self._refresh_alignment_depth_status()
        presentation = self._refinement_presentation
        if not self._sensor_tip_distance_context_id:
            self.refinement_dialog.sensor_tip_distance_status_label.setText(
                "Test may move the arm to see the target, measure, and return."
            )
        if presentation is None:
            return
        alignment_available = (
            presentation.active_stage is RefinementStage.ALIGNMENT
            and not self._refinement_has_pending_work(presentation)
            and not presentation.recovery_required
            and not self._distance_failure_requires_retraction
        )
        self.refinement_dialog.test_distance_measure_button.setEnabled(
            alignment_available
            and presentation.motion_states[RefinementStage.ALIGNMENT]
            is RefinementMotionState.REACHED
        )
        self.refinement_dialog.orient_sensor_tip_to_surface_button.setEnabled(
            alignment_available
            and presentation.motion_states[RefinementStage.SAFE_APPROACH]
            is RefinementMotionState.REACHED
        )
        self.refinement_dialog.continue_to_summary_button.setEnabled(
            self._can_open_probe_summary()
        )
        if presentation.active_stage is RefinementStage.PROBE:
            self.refinement_dialog.set_next_action(
                text="Next", visible=not self.refinement_dialog.is_summary_page(),
            )
        if self.refinement_dialog.is_summary_page():
            self.refinement_dialog.refresh_summary()

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
        state = self._probe_setup_state
        self._probe_finalization_point_id = point_id
        self._probe_finalization_scope = (
            (state.selected_object_id, state.selected_routine_id)
            if state is not None
            else None
        )
        self.saved_probe_controls.clear_pending_selection()
        self.approve_and_retract_button.setEnabled(False)
        self.retract_without_saving_button.setEnabled(False)
        self._refresh_refinement_dialog()
        return True

    def handle_finalization_rejected(self, detail):
        """Release the saving guard only when the action was not accepted."""
        self._clear_pending_probe_finalization()
        self._refresh_refinement_dialog()
        self.refinement_recovery_status_label.setText(detail)

    def handle_retract_without_saving(self):
        """Request server-owned retraction without persistence."""
        presentation = self._refinement_presentation
        if (
            presentation is None
            or not self._can_retract_without_saving(presentation)
        ):
            return False
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
