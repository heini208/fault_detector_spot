"""Guided probe-point setup dialog."""

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from fault_detector_spot.inspection.setup.probe_refinement_session import (
    RefinementMotionState,
    RefinementStage,
)


class ProbeRefinementDialog(QDialog):
    """Guide reference selection, refinement, and final probe-point saving."""

    STAGES = (
        RefinementStage.SAFE_APPROACH,
        RefinementStage.ALIGNMENT,
        RefinementStage.PROBE,
    )

    REFERENCE_PAGE = 0
    SAFE_APPROACH_PAGE = 1
    ALIGNMENT_PAGE = 2
    PROBE_PAGE = 3
    SUMMARY_PAGE = 4

    def __init__(self, controls):
        parent = controls.ui if isinstance(controls.ui, QWidget) else None
        super().__init__(parent)
        self.controls = controls
        self._force_close = False
        self._workflow_controls_attached = False
        self._reference_selection_enabled = False

        self.setWindowTitle("Add Probe Point")
        self.setModal(False)
        self.resize(920, 760)

        self.aligned_distance_field = QLineEdit(
            controls.reference_preapproach_distance_field.text()
        )
        self.aligned_distance_field.setFixedWidth(90)
        self.aligned_distance_field.setValidator(
            controls._distance_validator(self.aligned_distance_field)
        )
        self.target_distance_field = QLineEdit(
            controls.reference_target_distance_field.text()
        )
        self.target_distance_field.setFixedWidth(90)
        self.target_distance_field.setValidator(
            controls._distance_validator(self.target_distance_field)
        )
        self.aligned_distance_field.editingFinished.connect(
            self._handle_distance_editing_finished
        )
        self.target_distance_field.editingFinished.connect(
            self._handle_distance_editing_finished
        )

        layout = QVBoxLayout(self)

        self.progress_label = QLabel("Step 1 of 5")
        progress_font = self.progress_label.font()
        progress_font.setBold(True)
        self.progress_label.setFont(progress_font)
        layout.addWidget(self.progress_label)

        self.refinement_controls_widget = QWidget()
        step_row = QHBoxLayout(self.refinement_controls_widget)
        step_row.setContentsMargins(0, 0, 0, 0)
        step_row.addWidget(QLabel("Translation step [m]:"))
        step_row.addWidget(self.controls.refine_translation_step_field)
        step_row.addSpacing(12)
        step_row.addWidget(QLabel("Rotation step [deg]:"))
        step_row.addWidget(self.controls.refine_rotation_step_field)
        step_row.addSpacing(12)
        step_row.addWidget(QLabel("Adjustment frame:"))
        step_row.addWidget(self.controls.refine_frame_dropdown)
        step_row.addStretch()
        layout.addWidget(self.refinement_controls_widget)

        self.workflow_stack = QStackedWidget()
        self.stage_stack = self.workflow_stack
        self.pose_comparison_labels = {}

        self.workflow_stack.addWidget(self._make_reference_page())
        self.workflow_stack.addWidget(self._make_safe_approach_page())
        self.workflow_stack.addWidget(self._make_alignment_page())
        self.workflow_stack.addWidget(self._make_probe_page())
        self.workflow_stack.addWidget(self._make_summary_page())
        layout.addWidget(self.workflow_stack, 1)

        footer = QHBoxLayout()

        self.emergency_stop_button = QPushButton("EMERGENCY STOP")
        self.emergency_stop_button.setStyleSheet(
            "QPushButton { background-color: #C62828; color: white; "
            "font-weight: bold; padding: 8px; }"
        )
        self.emergency_stop_button.clicked.connect(
            self.controls.handle_refinement_emergency_stop
        )
        footer.addWidget(self.emergency_stop_button)
        footer.addStretch()

        self.back_button = QPushButton("Back")
        self.back_button.clicked.connect(
            self.controls.handle_refinement_back
        )
        footer.addWidget(self.back_button)

        self.next_button = QPushButton("Next")
        self.next_button.clicked.connect(
            self.controls.handle_refinement_next
        )
        footer.addWidget(self.next_button)

        self.close_button = QPushButton("Close")
        self.close_button.clicked.connect(self.close)
        footer.addWidget(self.close_button)
        layout.addLayout(footer)

        self.refinement_controls_widget.hide()
        self.emergency_stop_button.hide()
        self.back_button.hide()
        self.next_button.hide()

    def attach_workflow_controls(self):
        """Move existing controls into their new workflow-owned pages."""
        if self._workflow_controls_attached:
            return

        camera_dropdown = self.controls.reference_camera_dropdowns[0]
        self.reference_camera_row.addWidget(camera_dropdown, 1)
        self.reference_view_layout.addWidget(
            self.controls.reference_view_widget,
            1,
        )
        self.reference_view_placeholder.hide()

        self.reference_selection_row.addWidget(
            self.controls.reference_pixel_value_label
        )
        self.reference_selection_row.addWidget(
            self.controls.clear_reference_pixel_button
        )

        definition_layout = self.summary_definition_layout
        definition_layout.addRow(
            "Probe point ID:",
            self.controls.probe_point_id_field,
        )
        definition_layout.addRow(
            "Display name:",
            self.controls.probe_point_display_name_field,
        )
        definition_layout.addRow(
            "Position tolerance [m]:",
            self.controls.probe_position_tolerance_field,
        )
        definition_layout.addRow(
            "Orientation tolerance [rad]:",
            self.controls.probe_orientation_tolerance_field,
        )
        definition_layout.addRow(
            "Measurement duration [s]:",
            self.controls.probe_measurement_duration_field,
        )

        self.summary_action_layout.addWidget(
            self.controls.approve_and_retract_button
        )
        self.controls.approve_and_retract_button.setText(
            "Save Probe Point and Retract"
        )
        self.summary_layout.addWidget(
            self.controls.save_probe_point_status_label
        )

        self.controls.reference_view_widget.image_point_changed.connect(
            lambda _u, _v: self.refresh_reference_selection()
        )
        self.controls.reference_view_widget.image_point_cleared.connect(
            self.refresh_reference_selection
        )
        self._workflow_controls_attached = True

    def _make_reference_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        heading = QLabel("Select Probe Point from Reference View")
        heading_font = heading.font()
        heading_font.setPointSize(heading_font.pointSize() + 2)
        heading_font.setBold(True)
        heading.setFont(heading_font)
        layout.addWidget(heading)

        description = QLabel(
            "Choose whether to reuse the saved six-camera reference set or "
            "capture a new one. Select a camera, mark the desired probe "
            "position, then approve the point to start physical refinement."
        )
        description.setWordWrap(True)
        layout.addWidget(description)

        self.reference_choice_group = QGroupBox("Reference views")
        choice_layout = QVBoxLayout(self.reference_choice_group)
        self.reference_choice_status = QLabel("")
        self.reference_choice_status.setWordWrap(True)
        choice_layout.addWidget(self.reference_choice_status)

        choice_buttons = QHBoxLayout()
        self.use_existing_reference_button = QPushButton(
            "Use Existing Reference Views"
        )
        self.use_existing_reference_button.clicked.connect(
            self.controls.handle_use_existing_reference_views
        )
        choice_buttons.addWidget(self.use_existing_reference_button)

        self.capture_reference_button = QPushButton(
            "Capture Reference Views"
        )
        self.capture_reference_button.clicked.connect(
            self.controls.handle_capture_new_reference_views
        )
        choice_buttons.addWidget(self.capture_reference_button)

        self.retake_reference_button = QPushButton(
            "Retake All Reference Views"
        )
        self.retake_reference_button.clicked.connect(
            self.controls.handle_retake_reference_views
        )
        choice_buttons.addWidget(self.retake_reference_button)
        choice_buttons.addStretch()
        choice_layout.addLayout(choice_buttons)
        layout.addWidget(self.reference_choice_group)

        self.reference_view_group = QGroupBox("Reference image")
        self.reference_view_layout = QVBoxLayout(self.reference_view_group)

        self.reference_camera_row = QHBoxLayout()
        self.reference_camera_row.addWidget(QLabel("Camera:"))
        self.reference_view_layout.addLayout(self.reference_camera_row)

        self.reference_view_placeholder = QLabel(
            "Reference image controls will appear here."
        )
        self.reference_view_placeholder.setAlignment(Qt.AlignCenter)
        self.reference_view_layout.addWidget(
            self.reference_view_placeholder,
            1,
        )

        self.reference_selection_row = QHBoxLayout()
        self.reference_selection_row.addWidget(QLabel("Selected pixel:"))
        self.reference_selection_row.addStretch()
        self.reference_view_layout.addLayout(self.reference_selection_row)
        layout.addWidget(self.reference_view_group, 1)

        self.reference_status_label = QLabel("")
        self.reference_status_label.setWordWrap(True)
        layout.addWidget(self.reference_status_label)

        actions = QHBoxLayout()
        actions.addStretch()
        self.approve_reference_button = QPushButton(
            "Approve Point and Start Refinement"
        )
        self.approve_reference_button.clicked.connect(
            self.controls.handle_reference_point_approved
        )
        actions.addWidget(self.approve_reference_button)
        layout.addLayout(actions)
        return page

    def _make_safe_approach_page(self):
        return self._make_scroll_page(
            RefinementStage.SAFE_APPROACH,
            "Safe Approach Pose",
            "Capture or reach an obstacle-safe sensor-tip pose. This pose "
            "is independent from the surface-aligned geometry.",
            self.controls.approach_step_status_label,
            self.controls.move_calculated_approach_button,
            self.controls.use_current_approach_button,
            self.controls._make_refinement_controls("approach"),
        )

    def _make_alignment_page(self):
        distance_row = QHBoxLayout()
        distance_row.addWidget(QLabel("Absolute surface distance [m]:"))
        distance_row.addWidget(self.aligned_distance_field)
        distance_row.addStretch()
        distance_widget = QWidget()
        distance_widget.setLayout(distance_row)

        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.addWidget(distance_widget)

        clearance_group = QGroupBox("Camera clearance recovery")
        clearance_layout = QVBoxLayout(clearance_group)
        clearance_hint = QLabel(
            "If lateral or rotational refinement moves the hand camera too "
            "close for reliable registered depth, move one translation step "
            "away from the surface along probe local -X."
        )
        clearance_hint.setWordWrap(True)
        clearance_layout.addWidget(clearance_hint)
        self.back_away_surface_button = QPushButton(
            "Back Away from Surface"
        )
        self.back_away_surface_button.setToolTip(
            "Move one Translation step away from the surface in the Sensor "
            "frame, regardless of the selected adjustment frame."
        )
        self.back_away_surface_button.clicked.connect(
            self._handle_back_away_from_surface
        )
        clearance_layout.addWidget(self.back_away_surface_button)

        content_layout.addWidget(
            self.controls._make_refinement_controls("alignment")
        )
        content_layout.addWidget(clearance_group)
        return self._make_scroll_page(
            RefinementStage.ALIGNMENT,
            "Aligned Pre-approach Pose",
            "Refine the shared lateral position and orientation at the "
            "absolute pre-approach distance. No independent axial offset "
            "is permitted.",
            self.controls.alignment_step_status_label,
            self.controls.move_aligned_pose_button,
            self.controls.use_current_alignment_button,
            content,
        )

    def _make_probe_page(self):
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.addWidget(
            self.controls._make_surface_distance_controls(
                self.target_distance_field
            )
        )

        actions = QHBoxLayout()
        self.continue_to_summary_button = QPushButton(
            "Continue to Summary"
        )
        self.continue_to_summary_button.setEnabled(False)
        self.continue_to_summary_button.clicked.connect(
            self.controls.handle_open_probe_summary
        )
        actions.addWidget(self.continue_to_summary_button)
        actions.addWidget(self.controls.retract_without_saving_button)
        actions.addStretch()
        content_layout.addLayout(actions)
        content_layout.addWidget(
            self.controls.refinement_recovery_status_label
        )
        return self._make_scroll_page(
            RefinementStage.PROBE,
            "Probe Pose and Surface Distance",
            "Move to the desired live surface distance. Once the target "
            "distance is reached, continue to the summary page to name and "
            "save the probe point.",
            self.controls.probe_step_status_label,
            None,
            None,
            content,
        )

    def _make_summary_page(self):
        page = QWidget()
        self.summary_layout = QVBoxLayout(page)

        heading = QLabel("Probe Point Summary")
        heading_font = heading.font()
        heading_font.setPointSize(heading_font.pointSize() + 2)
        heading_font.setBold(True)
        heading.setFont(heading_font)
        self.summary_layout.addWidget(heading)

        description = QLabel(
            "Review the approved approach geometry and enter the probe-point "
            "identity before saving. Saving performs the existing mandatory "
            "retraction."
        )
        description.setWordWrap(True)
        self.summary_layout.addWidget(description)

        pose_group = QGroupBox("Approved poses in object frame")
        pose_layout = QFormLayout(pose_group)
        self.summary_safe_pose_label = QLabel("Not set")
        self.summary_safe_pose_label.setWordWrap(True)
        self.summary_aligned_pose_label = QLabel("Not set")
        self.summary_aligned_pose_label.setWordWrap(True)
        pose_layout.addRow(
            "Approach pose:",
            self.summary_safe_pose_label,
        )
        pose_layout.addRow(
            "Aligned pre-approach pose:",
            self.summary_aligned_pose_label,
        )
        self.summary_layout.addWidget(pose_group)

        reference_group = QGroupBox("Reference point")
        reference_layout = QFormLayout(reference_group)
        self.summary_reference_camera_label = QLabel("Not set")
        self.summary_reference_pixel_label = QLabel("Not set")
        reference_layout.addRow(
            "Camera:",
            self.summary_reference_camera_label,
        )
        reference_layout.addRow(
            "Pixel:",
            self.summary_reference_pixel_label,
        )
        self.summary_layout.addWidget(reference_group)

        definition_group = QGroupBox("Probe point definition")
        self.summary_definition_layout = QFormLayout(definition_group)
        self.summary_layout.addWidget(definition_group)

        self.summary_action_layout = QHBoxLayout()
        self.summary_action_layout.addStretch()
        self.summary_layout.addLayout(self.summary_action_layout)
        self.summary_layout.addStretch()
        return page

    def _make_scroll_page(
        self,
        stage,
        title,
        description,
        status_label,
        move_button,
        approve_button,
        controls_widget,
    ):
        page = QWidget()
        page_layout = QVBoxLayout(page)
        heading = QLabel(title)
        heading_font = heading.font()
        heading_font.setPointSize(heading_font.pointSize() + 2)
        heading_font.setBold(True)
        heading.setFont(heading_font)
        page_layout.addWidget(heading)

        description_label = QLabel(description)
        description_label.setWordWrap(True)
        page_layout.addWidget(description_label)

        status_row = QHBoxLayout()
        status_row.addWidget(QLabel("Movement state:"))
        status_row.addWidget(status_label)
        status_row.addStretch()
        page_layout.addLayout(status_row)
        page_layout.addWidget(self._make_pose_comparison(stage))

        if move_button is not None and approve_button is not None:
            actions = QHBoxLayout()
            actions.addWidget(move_button)
            actions.addWidget(approve_button)
            actions.addStretch()
            page_layout.addLayout(actions)

        page_layout.addWidget(controls_widget)
        page_layout.addStretch()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        return scroll

    def _make_pose_comparison(self, stage):
        group = QGroupBox("Pose Comparison in Object Frame")
        layout = QFormLayout(group)
        labels = {}
        for key, title in (
            ("calculated", "Calculated pose:"),
            ("candidate", "Current candidate:"),
            ("approved", "Approved final pose:"),
            ("difference", "Candidate difference:"),
            ("status", "Status:"),
        ):
            label = QLabel("Not set")
            label.setWordWrap(True)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            if key != "status":
                font = QFont("Monospace")
                font.setStyleHint(QFont.TypeWriter)
                label.setFont(font)
            labels[key] = label
            layout.addRow(title, label)
        self.pose_comparison_labels[stage] = labels
        return group

    def _handle_distance_editing_finished(self):
        fields = (
            self.aligned_distance_field,
            self.target_distance_field,
        )
        if not any(field.isModified() for field in fields):
            return
        if not self.controls._handle_dialog_distances_changed():
            return
        for field in fields:
            field.setModified(False)
            field.setEnabled(False)
        for button in (
            self.controls.move_aligned_pose_button,
            self.controls.use_current_alignment_button,
            self.controls.orient_to_surface_button,
            self.controls.orient_to_tag_button,
            self.back_button,
            self.next_button,
        ):
            button.setEnabled(False)

    def _handle_back_away_from_surface(self):
        presentation = self.controls._require_refinement_presentation()
        if presentation.active_stage is not RefinementStage.ALIGNMENT:
            self.controls._show_setup_error(
                "Back Away from Surface",
                ValueError("Alignment refinement is not active"),
            )
            return False

        dropdown = self.controls.refine_frame_dropdown
        sensor_index = dropdown.findData("sensor")
        if sensor_index < 0:
            self.controls._show_setup_error(
                "Back Away from Surface",
                ValueError("Sensor refinement frame is unavailable"),
            )
            return False

        previous_index = dropdown.currentIndex()
        dropdown.blockSignals(True)
        dropdown.setCurrentIndex(sensor_index)
        dropdown.blockSignals(False)
        try:
            return self.controls.handle_refine_pose("alignment", "back")
        finally:
            dropdown.blockSignals(True)
            dropdown.setCurrentIndex(previous_index)
            dropdown.blockSignals(False)

    def open_reference_selection(self, has_existing):
        self._force_close = False
        self._reference_selection_enabled = False
        self.workflow_stack.setCurrentIndex(self.REFERENCE_PAGE)
        self.progress_label.setText("Step 1 of 5 — Select Reference Point")
        self.refinement_controls_widget.hide()
        self.emergency_stop_button.hide()
        self.back_button.hide()
        self.next_button.hide()
        self.update_reference_availability(has_existing)
        self.show()
        self.raise_()
        self.activateWindow()

    def update_reference_availability(self, has_existing):
        if self.workflow_stack.currentIndex() != self.REFERENCE_PAGE:
            return
        if has_existing:
            self.reference_choice_status.setText(
                "Saved reference views are available for this routine."
            )
            self.use_existing_reference_button.show()
            self.retake_reference_button.show()
            self.capture_reference_button.hide()
            self.reference_view_group.setEnabled(
                self._reference_selection_enabled
            )
            if self._reference_selection_enabled:
                self.refresh_reference_selection()
            else:
                self.approve_reference_button.setEnabled(False)
        else:
            self._reference_selection_enabled = False
            self.reference_choice_status.setText(
                "No saved reference views are available for this routine."
            )
            self.use_existing_reference_button.hide()
            self.retake_reference_button.hide()
            self.capture_reference_button.show()
            self.reference_view_group.setEnabled(False)
            self.approve_reference_button.setEnabled(False)

    def enable_reference_selection(self):
        self._reference_selection_enabled = True
        self.reference_view_group.setEnabled(True)
        self.set_reference_status(
            "Select a camera and click the desired probe position."
        )
        self.refresh_reference_selection()

    def set_reference_capture_running(self):
        self._reference_selection_enabled = False
        self.reference_choice_status.setText(
            "Capturing all six reference views..."
        )
        self.use_existing_reference_button.setEnabled(False)
        self.capture_reference_button.setEnabled(False)
        self.retake_reference_button.setEnabled(False)
        self.reference_view_group.setEnabled(False)
        self.approve_reference_button.setEnabled(False)

    def reference_views_ready(self):
        self._reference_selection_enabled = True
        self.use_existing_reference_button.setEnabled(True)
        self.capture_reference_button.setEnabled(True)
        self.retake_reference_button.setEnabled(True)
        self.reference_choice_status.setText(
            "Reference views are ready."
        )
        self.reference_view_group.setEnabled(True)
        self.refresh_reference_selection()

    def set_reference_status(self, text):
        self.reference_status_label.setText(text)

    def reference_selection_changed(self):
        self.approve_reference_button.setEnabled(False)
        self.set_reference_status(
            "Select a point in the displayed reference image."
        )

    def refresh_reference_selection(self):
        if not self._workflow_controls_attached:
            return
        point = self.controls.reference_view_widget.selected_image_point
        view_id = self.controls._reference_slot_view_ids[0]
        ready = point is not None and bool(view_id)
        self.approve_reference_button.setEnabled(ready)
        if ready:
            self.set_reference_status(
                f"Selected u={point.u}, v={point.v}. "
                "Approve to begin refinement."
            )

    def show_stage(self, stage):
        index = self.STAGES.index(stage)
        self.workflow_stack.setCurrentIndex(index + 1)
        self.progress_label.setText(
            f"Step {index + 2} of 5 — "
            f"{self._stage_title(stage)}"
        )
        self.refinement_controls_widget.show()
        self.emergency_stop_button.show()
        self.back_button.show()
        self.next_button.show()
        self.controls._handle_refinement_stage_changed(stage)

        presentation = self.controls._refinement_presentation
        if (
            stage is RefinementStage.PROBE
            and presentation is not None
            and presentation.stage_is_approved(RefinementStage.ALIGNMENT)
            and presentation.motion_states[RefinementStage.ALIGNMENT]
            is RefinementMotionState.REACHED
        ):
            self.controls.surface_distance_test_status_label.setText(
                "Aligned pre-approach approved. Ready to test surface distance."
            )
        self.refresh()

    def show_summary(self):
        self.workflow_stack.setCurrentIndex(self.SUMMARY_PAGE)
        self.progress_label.setText("Step 5 of 5 — Probe Point Summary")
        self.refinement_controls_widget.hide()
        self.emergency_stop_button.show()
        self.back_button.show()
        self.next_button.hide()
        self.refresh_summary()

    def is_summary_page(self):
        return self.workflow_stack.currentIndex() == self.SUMMARY_PAGE

    def refresh_summary(self):
        presentation = self.controls._refinement_presentation
        if presentation is None:
            return
        safe_pose = presentation.approved_pose(
            RefinementStage.SAFE_APPROACH
        )
        aligned_pose = presentation.approved_pose(
            RefinementStage.ALIGNMENT
        )
        self.summary_safe_pose_label.setText(
            self.controls._pose_summary(safe_pose)
            if safe_pose is not None
            else "Not approved"
        )
        self.summary_aligned_pose_label.setText(
            self.controls._pose_summary(aligned_pose)
            if aligned_pose is not None
            else "Not approved"
        )

        state = self.controls._probe_setup_state
        if state is None or not state.has_reference_pixel:
            self.summary_reference_camera_label.setText("Not set")
            self.summary_reference_pixel_label.setText("Not set")
            return

        camera_id = "Unknown"
        for candidate_camera, candidate_view in zip(
            state.reference_camera_ids,
            state.reference_view_ids,
        ):
            if candidate_view == state.selected_reference_view_id:
                camera_id = candidate_camera
                break
        self.summary_reference_camera_label.setText(camera_id)
        self.summary_reference_pixel_label.setText(
            f"u={state.reference_pixel_u}, v={state.reference_pixel_v}"
        )

    @staticmethod
    def _stage_title(stage):
        return {
            RefinementStage.SAFE_APPROACH: "Safe Approach",
            RefinementStage.ALIGNMENT: "Aligned Pre-approach",
            RefinementStage.PROBE: "Probe",
        }[stage]

    def refresh(self):
        self.controls._refresh_refinement_dialog()

    def open_for_stage(self, stage):
        self._force_close = False
        self.show_stage(stage)
        self.show()
        self.raise_()
        self.activateWindow()

    def close_after_completion(self):
        self._force_close = True
        self.close()

    def closeEvent(self, event):
        if self._force_close:
            event.accept()
            return
        if self.controls.request_close_refinement_workflow():
            event.accept()
            return
        event.ignore()
