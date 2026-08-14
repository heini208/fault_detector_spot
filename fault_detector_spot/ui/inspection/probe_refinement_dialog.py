"""Three-stage supervised probe-point refinement dialog."""

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
    RefinementStage,
)


class ProbeRefinementDialog(QDialog):
    """Display one ordered refinement stage at a time."""

    STAGES = (
        RefinementStage.SAFE_APPROACH,
        RefinementStage.ALIGNMENT,
        RefinementStage.PROBE,
    )

    def __init__(self, controls):
        """Create the dialog around controls owned by InspectionControls."""
        parent = controls.ui if isinstance(controls.ui, QWidget) else None
        super().__init__(parent)
        self.controls = controls
        self._force_close = False
        self.setWindowTitle("Probe Point Position Refinement")
        self.setModal(False)
        self.resize(880, 720)
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
        self.progress_label = QLabel("Step 1 of 3")
        progress_font = self.progress_label.font()
        progress_font.setBold(True)
        self.progress_label.setFont(progress_font)
        layout.addWidget(self.progress_label)

        step_row = QHBoxLayout()
        step_row.addWidget(QLabel("Translation step [m]:"))
        step_row.addWidget(self.controls.refine_translation_step_field)
        step_row.addSpacing(12)
        step_row.addWidget(QLabel("Rotation step [deg]:"))
        step_row.addWidget(self.controls.refine_rotation_step_field)
        step_row.addSpacing(12)
        step_row.addWidget(QLabel("Adjustment frame:"))
        step_row.addWidget(self.controls.refine_frame_dropdown)
        step_row.addStretch()
        layout.addLayout(step_row)

        self.stage_stack = QStackedWidget()
        self.pose_comparison_labels = {}
        self.stage_stack.addWidget(self._make_safe_approach_page())
        self.stage_stack.addWidget(self._make_alignment_page())
        self.stage_stack.addWidget(self._make_probe_page())
        layout.addWidget(self.stage_stack, 1)

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
            self.controls.calculate_hand_surface_orientation_button,
            self.controls.orient_to_calculated_surface_button,
            self.back_button,
            self.next_button,
        ):
            button.setEnabled(False)

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
        content_layout.addWidget(
            self.controls._make_refinement_controls("alignment")
        )
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
        actions.addWidget(self.controls.approve_and_retract_button)
        actions.addWidget(self.controls.retract_without_saving_button)
        actions.addStretch()
        content_layout.addLayout(actions)
        content_layout.addWidget(
            self.controls.refinement_recovery_status_label
        )
        return self._make_scroll_page(
            RefinementStage.PROBE,
            "Probe Pose and Surface Distance",
            "The aligned and probe poses share lateral position and "
            "orientation. Only bounded live distance corrections may move "
            "the tip along the surface normal.",
            self.controls.probe_step_status_label,
            None,
            None,
            content,
        )

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

    def show_stage(self, stage):
        """Show a stage without commanding robot movement."""
        index = self.STAGES.index(stage)
        self.stage_stack.setCurrentIndex(index)
        self.progress_label.setText(f"Step {index + 1} of 3")
        self.controls._handle_refinement_stage_changed(stage)
        self.refresh()

    def refresh(self):
        """Refresh navigation and pose readouts from the owner state."""
        self.controls._refresh_refinement_dialog()

    def open_for_stage(self, stage):
        """Open at the first missing stage or an explicit retained stage."""
        self._force_close = False
        self.show_stage(stage)
        self.show()
        self.raise_()
        self.activateWindow()

    def close_after_completion(self):
        """Close without re-running recovery guards."""
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
