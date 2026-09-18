"""Add server-owned probe finalization to inspection controls."""

import math
from uuid import uuid4

from PyQt5.QtWidgets import QDoubleSpinBox, QFrame, QLabel, QListWidget, QPushButton, QVBoxLayout

from fault_detector_msgs.msg import (
    ApplicationCommandState,
    OperationalIntent,
    ProbeSetupIntent,
    ProbeSetupState,
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

from fault_detector_spot.ui.sensor.models import SensorAttachmentViewStatus

from .controls import InspectionControls


class FinalizingInspectionControls(InspectionControls):
    """Route physical setup workflows through server-owned APIs."""

    def __init__(self, ui):
        self._reference_start_pending = False
        self._reference_start_error = ""
        self._surface_test_active = False
        self._surface_move_succeeded = False
        self._surface_move_target_m = None
        self._refinement_emergency_stop_requested = False
        self._reference_preview_cache = {}
        self._reference_view_id_by_camera = {}
        self._begin_refinement_after_reference_commit = False
        self._reference_capture_in_progress = False
        self._saved_probe_scope = None
        self._saved_probe_operation_context = ""
        self._probe_finalization_point_id = ""
        self._probe_finalization_scope = None
        self._saved_probe_selection_after_finalization = None
        self._probe_point_entry_panel = None
        self._probe_point_overview_label = None
        super().__init__(ui)
        self.alignment_step_status_label.setWordWrap(True)
        self.refinement_dialog.attach_workflow_controls()
        self._configure_probe_point_entry_ui()

    def _configure_probe_point_entry_ui(self):
        """Replace the old setup workspace with one probe-point entry action."""
        splitter = self.inspection_workspace_splitter
        original_widgets = [
            splitter.widget(index)
            for index in range(splitter.count())
        ]
        for widget in original_widgets:
            if widget is not None:
                widget.hide()

        panel = QFrame()
        panel.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        title = QLabel("Probe Points")
        title_font = title.font()
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        self._probe_point_overview_label = QLabel(
            "Select an inspection routine to add a probe point."
        )
        self._probe_point_overview_label.setWordWrap(True)
        layout.addWidget(self._probe_point_overview_label)

        self.saved_probe_points_list = QListWidget()
        self.saved_probe_points_list.currentRowChanged.connect(
            self._saved_probe_selection_changed
        )
        layout.addWidget(self.saved_probe_points_list)
        layout.addWidget(QLabel("Target distance from wall [m]:"))
        self.saved_probe_distance = QDoubleSpinBox()
        self.saved_probe_distance.setDecimals(3)
        self.saved_probe_distance.setRange(0.0, 10.0)
        self.saved_probe_distance.setSingleStep(0.005)
        self.saved_probe_distance.setEnabled(False)
        self.saved_probe_distance.setToolTip(
            "Defaults to the selected point’s saved value. Changes apply only to this movement."
        )
        layout.addWidget(self.saved_probe_distance)
        self.saved_probe_action_buttons = {}
        for label, operation in (
            ("Move to Saved Safe Approach", OperationalIntent.INTENT_MOVE_SAVED_PROBE_SAFE_APPROACH),
            ("Move to Saved Aligned Pre-approach", OperationalIntent.INTENT_MOVE_SAVED_PROBE_ALIGNED_PREAPPROACH),
            ("Move Close to Wall", OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE),
        ):
            button = QPushButton(label)
            button.setEnabled(False)
            button.clicked.connect(
                lambda _checked=False, operation=operation: self.handle_saved_probe_motion(operation)
            )
            self.saved_probe_action_buttons[operation] = button
            layout.addWidget(button)
        self.saved_probe_motion_status = QLabel("Select a saved probe point.")
        self.saved_probe_motion_status.setWordWrap(True)
        layout.addWidget(self.saved_probe_motion_status)


        self.start_probe_refinement_button.setText(
            "Add New Probe Point"
        )
        layout.addWidget(self.start_probe_refinement_button)
        layout.addStretch()

        splitter.addWidget(panel)
        splitter.setStretchFactor(splitter.count() - 1, 1)
        splitter.setSizes(
            [0] * (splitter.count() - 1) + [500]
        )
        self._probe_point_entry_panel = panel

    def _saved_probe_selection_changed(self, _row=None):
        state = self._probe_setup_state
        row = self.saved_probe_points_list.currentRow()
        if state is not None and 0 <= row < len(state.probe_point_target_surface_distances_m):
            self.saved_probe_distance.setValue(state.probe_point_target_surface_distances_m[row])
        else:
            self.saved_probe_distance.setValue(0.0)
        self._refresh_saved_probe_actions()

    def _refresh_saved_probe_actions(self, _row=None):
        state = self._probe_setup_state
        enabled = bool(
            state is not None
            and state.selected_object_id and state.selected_routine_id
            and self.saved_probe_points_list.currentItem() is not None
            and not state.refinement_active and not state.motion_pending
            and not self._reference_start_pending
            and not self._saved_probe_operation_context
        )
        row = self.saved_probe_points_list.currentRow()
        self.saved_probe_distance.setEnabled(
            enabled and 0 <= row < len(state.probe_point_target_surface_distances_m)
        )
        for button in self.saved_probe_action_buttons.values():
            button.setEnabled(enabled)
        self.saved_probe_points_list.setEnabled(
            not bool(self._saved_probe_operation_context)
        )

    def _update_saved_probe_points(self, state):
        scope = (state.selected_object_id, state.selected_routine_id)
        ids = list(state.probe_point_ids) if all(scope) else []
        current = self.saved_probe_points_list.currentItem()
        selected = (
            current.text()
            if current and scope == self._saved_probe_scope
            else None
        )
        completed = self._saved_probe_selection_after_finalization
        select_completed = bool(
            completed is not None
            and completed[:2] == scope
            and completed[2] in ids
        )
        if select_completed:
            selected = completed[2]
        previous = [
            self.saved_probe_points_list.item(i).text()
            for i in range(self.saved_probe_points_list.count())
        ]
        changed = scope != self._saved_probe_scope or ids != previous
        if changed:
            self.saved_probe_points_list.blockSignals(True)
            self.saved_probe_points_list.clear()
            self.saved_probe_points_list.addItems(ids)
            if selected in ids:
                self.saved_probe_points_list.setCurrentRow(
                    ids.index(selected)
                )
            self.saved_probe_points_list.blockSignals(False)
            self._saved_probe_scope = scope

        if select_completed:
            if not changed:
                self.saved_probe_points_list.blockSignals(True)
                self.saved_probe_points_list.setCurrentRow(
                    ids.index(selected)
                )
                self.saved_probe_points_list.blockSignals(False)
            self._saved_probe_selection_after_finalization = None
            self._saved_probe_selection_changed()
            if not self._saved_probe_operation_context:
                self.saved_probe_motion_status.setText(
                    f"{selected}: ready."
                )
        elif changed:
            if selected not in ids:
                self._saved_probe_selection_changed()
            if not self._saved_probe_operation_context:
                self.saved_probe_motion_status.setText(
                    "Select a saved probe point."
                )
        self._refresh_saved_probe_actions()

    def handle_saved_probe_motion(self, operation):
        self._refresh_saved_probe_actions()
        button = self.saved_probe_action_buttons.get(operation)
        if button is None or not button.isEnabled():
            return False
        state = self._probe_setup_state
        point_id = self.saved_probe_points_list.currentItem().text()
        if (self._saved_probe_scope != (state.selected_object_id, state.selected_routine_id)
                or point_id not in state.probe_point_ids):
            return False
        intent = OperationalIntent()
        intent.intent = operation
        intent.object_id = state.selected_object_id
        intent.routine_id = state.selected_routine_id
        intent.probe_point_id = point_id
        if (operation == OperationalIntent.INTENT_MOVE_SAVED_PROBE_CLOSE_TO_SURFACE
                and self.saved_probe_distance.isEnabled()):
            intent.override_target_surface_distance = True
            intent.target_surface_distance_m = self.saved_probe_distance.value()
        self._saved_probe_operation_context = "saved-probe-" + uuid4().hex
        self.saved_probe_motion_status.setText(f"{point_id}: submitting movement...")
        self._refresh_saved_probe_actions()
        request_id = self.ui.execute_operation(
            intent, context_id=self._saved_probe_operation_context,
        )
        if request_id is None:
            self.handle_saved_probe_rejected("Movement could not be submitted.")
            return False
        return True

    def handle_saved_probe_rejected(self, detail):
        if not self._saved_probe_operation_context:
            return
        self._saved_probe_operation_context = ""
        self.saved_probe_motion_status.setText(detail)
        self._refresh_saved_probe_actions()

    def _handle_saved_probe_status(self, status):
        if (not self._saved_probe_operation_context
                or status.context_id != self._saved_probe_operation_context):
            return
        self.saved_probe_motion_status.setText(status.detail or "Movement in progress")
        if status.state in {
            ApplicationCommandState.STATE_SUCCEEDED,
            ApplicationCommandState.STATE_FAILED,
            ApplicationCommandState.STATE_CANCELLED,
        }:
            self._saved_probe_operation_context = ""
            labels = {
                ApplicationCommandState.STATE_SUCCEEDED: "Movement completed",
                ApplicationCommandState.STATE_FAILED: "Movement failed",
                ApplicationCommandState.STATE_CANCELLED: "Movement cancelled",
            }
            self.saved_probe_motion_status.setText(
                labels[status.state] + (": " + status.detail if status.detail else "")
            )
        self._refresh_saved_probe_actions()

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
        if (
            attachment is None
            or attachment.status is not SensorAttachmentViewStatus.ACTIVE
        ):
            self.show_warning(
                "Add Probe Point",
                "Confirm the sensor attachment in the sensor controls before "
                "adding a probe point. If using the bare hand, select and "
                "confirm No sensor.",
            )
            return False
        self._reference_start_pending = False
        self._reference_start_error = ""
        self._begin_refinement_after_reference_commit = False
        self.inspection_workspace_splitter.setEnabled(False)
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
        self._begin_refinement_after_reference_commit = False
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

        self._update_saved_probe_points(state)

        if self._probe_point_overview_label is not None:
            if not state.selected_routine_id:
                overview = "Select an inspection routine to add a probe point."
            elif state.probe_point_ids:
                overview = (
                    "Existing probe points: "
                    + ", ".join(state.probe_point_ids)
                )
            else:
                overview = "No probe points are saved for this routine."
            self._probe_point_overview_label.setText(overview)

        if self._refinement_presentation is None:
            self.start_probe_refinement_button.setText(
                "Add New Probe Point"
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
            elif state.state == ProbeSetupState.STATE_FAILED:
                self._reference_capture_in_progress = False
                self.refinement_dialog.set_reference_status(
                    state.detail or "Reference capture failed."
                )

        return result

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
        self._begin_refinement_after_reference_commit = False
        self._reference_capture_in_progress = True
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
        self._reference_rgb_size = None
        self._reference_depth_image = None
        self._reference_rgb_camera_info = None
        self._reference_camera_info = None
        self._reference_view = None
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
        for camera_id, view_id in zip(
            state.reference_camera_ids,
            state.reference_view_ids,
        ):
            if not camera_id or not view_id:
                continue
            self._reference_view_id_by_camera[camera_id] = view_id
            client.request_preview(view_id)

        self.refinement_dialog.update_reference_availability(True)
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
        if not self._surface_result_current():
            self.surface_distance_test_status_label.setText(
                "Reach the requested surface distance before continuing."
            )
            return False
        self.refinement_dialog.show_summary()
        return True

    def handle_refinement_back(self):
        if self.refinement_dialog.is_summary_page():
            self.refinement_dialog.show_stage(RefinementStage.PROBE)
            return True
        return super().handle_refinement_back()

    def handle_refinement_emergency_stop(self):
        self._refinement_emergency_stop_requested = True
        return super().handle_refinement_emergency_stop()

    def request_close_refinement_workflow(self):
        if self._refinement_presentation is None:
            self._begin_refinement_after_reference_commit = False
            if hasattr(self, "inspection_workspace_splitter"):
                self.inspection_workspace_splitter.setEnabled(True)
            self.start_probe_refinement_button.setText(
                "Add New Probe Point"
            )
            return True

        presentation = self._refinement_presentation
        if (
            self._probe_finalization_point_id
            and not presentation.recovery_required
        ):
            self.refinement_recovery_status_label.setText(
                "Wait for probe-point saving and mandatory retraction to "
                "finish."
            )
            return False

        if self._refinement_emergency_stop_requested:
            if hasattr(self, "inspection_workspace_splitter"):
                self.inspection_workspace_splitter.setEnabled(True)
            self.start_probe_refinement_button.setText(
                "Resume Probe Point Setup"
            )
            self.refinement_summary_status_label.setText(
                "Refinement paused after emergency stop."
            )
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
            "Add New Probe Point"
        )
        self.refinement_summary_status_label.setText(
            "Ending refinement workflow."
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
            self._saved_probe_selection_after_finalization = (
                scope[0],
                scope[1],
                point_id,
            )
        self._probe_finalization_point_id = ""
        self._probe_finalization_scope = None
        self._refinement_emergency_stop_requested = False
        self._begin_refinement_after_reference_commit = False
        result = super()._finish_refinement_workflow_close()
        self.start_probe_refinement_button.setText(
            "Add New Probe Point"
        )
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
        """Track saved-point movements and standalone close-surface tests."""
        self._handle_saved_probe_status(status)
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
        self._refresh_refinement_dialog()
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

    def _refresh_alignment_depth_status(self):
        presentation = self._refinement_presentation
        state = self._probe_setup_state
        if presentation is None or state is None:
            return

        motion_state = presentation.motion_states[RefinementStage.ALIGNMENT]
        label = self.alignment_step_status_label
        label.setToolTip("")

        if presentation.stage_is_approved(RefinementStage.ALIGNMENT):
            label.setText("Reached, registered depth verified")
            label.setToolTip(
                "The aligned pre-approach was approved only after live "
                "registered hand depth confirmed usable camera clearance "
                "for the probe step."
            )
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
            label.setText(
                f"{motion_state.value}, registered depth NOT verified"
            )
            label.setToolTip(detail)
            return

        if motion_state is RefinementMotionState.REACHED:
            label.setText("Reached, depth check pending approval")
            label.setToolTip(
                "Approve Current Pose to verify live registered hand depth "
                "before continuing to the probe step."
            )
            return

        label.setText(motion_state.value)

    def _refresh_refinement_dialog(self):
        super()._refresh_refinement_dialog()
        self._refresh_alignment_depth_status()
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
        self.refinement_dialog.continue_to_summary_button.setEnabled(
            probe_page
            and alignment_reached
            and self._surface_result_current()
            and not pending
            and not retraction_required
        )
        self._update_save_probe_point_state()
        if (
            self._probe_finalization_point_id
            and not presentation.recovery_required
        ):
            self.refinement_dialog.close_button.setEnabled(False)
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
        self._saved_probe_selection_after_finalization = None
        self.approve_and_retract_button.setEnabled(False)
        self.retract_without_saving_button.setEnabled(False)
        self._refresh_refinement_dialog()
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
