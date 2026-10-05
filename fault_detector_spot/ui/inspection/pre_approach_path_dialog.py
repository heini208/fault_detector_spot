"""Present server-owned pre-approach path editing and movement intents."""

from PyQt5.QtWidgets import (
    QDialog, QDoubleSpinBox, QHBoxLayout, QLabel, QLineEdit, QListWidget, QPushButton, QVBoxLayout,
)
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupMotionIntent

from .speed_slider import SpeedSlider


class PreApproachPathDialog(QDialog):
    def __init__(self, controls, parent):
        super().__init__(parent)
        self.controls = controls
        self._point_speeds = []
        self.setWindowTitle("Pre-approach Pathing Points")
        self.resize(520, 440)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Points are visited in this order before the final aligned pose."))
        self.points = QListWidget()
        layout.addWidget(self.points)
        row = QHBoxLayout()
        self.move_button = QPushButton("Move to Pathing Point")
        self.up_button = QPushButton("Move Up")
        self.down_button = QPushButton("Move Down")
        for button in (self.move_button, self.up_button, self.down_button):
            row.addWidget(button)
        layout.addLayout(row)
        self.delete_button = QPushButton("Delete Selected Pathing Point")
        self.delete_button.setAutoDefault(False)
        self.delete_button.clicked.connect(self._delete)
        layout.addWidget(self.delete_button)
        self.adjust_button = QPushButton("Open Fine Adjustment")
        self.adjust_button.setAutoDefault(False)
        self.adjust_button.clicked.connect(
            lambda: controls.refinement_dialog.fine_adjustment_dialog.open_for(True)
        )
        layout.addWidget(self.adjust_button)
        self.name_field = QLineEdit()
        self.name_field.setPlaceholderText("Pathing point name")
        layout.addWidget(self.name_field)
        layout.addWidget(QLabel("New pathing point position tolerance [m]:"))
        self.tolerance_field = QDoubleSpinBox()
        self.tolerance_field.setDecimals(3)
        self.tolerance_field.setRange(.001, 1.)
        self.tolerance_field.setValue(.01)
        layout.addWidget(self.tolerance_field)
        layout.addWidget(QLabel("New or selected point speed (% of configured arm speed):"))
        self.speed_field = SpeedSlider()
        self.speed_field.setValue(100)
        layout.addWidget(self.speed_field)
        self.save_speed_button = QPushButton("Save Speed for Selected Pathing Point")
        self.save_speed_button.setAutoDefault(False)
        self.save_speed_button.clicked.connect(self._save_speed)
        layout.addWidget(self.save_speed_button)
        self.name_error_label = QLabel()
        self.name_error_label.setWordWrap(True)
        layout.addWidget(self.name_error_label)
        self.add_button = QPushButton("Add Current Arm Pose as Pathing Point")
        layout.addWidget(self.add_button)
        self.close_button = QPushButton("Close Pathing Point View")
        layout.addWidget(self.close_button)
        for button in (self.move_button, self.up_button, self.down_button,
                       self.add_button, self.close_button):
            button.setAutoDefault(False)
        self.close_button.clicked.connect(self.hide)
        self.add_button.clicked.connect(self._add)
        self.move_button.clicked.connect(self._move)
        self.up_button.clicked.connect(lambda: self._reorder(-1))
        self.down_button.clicked.connect(lambda: self._reorder(1))
        self.points.currentRowChanged.connect(self._update_buttons)
        self.points.currentRowChanged.connect(self._select_speed)
        self.name_field.textChanged.connect(self._update_buttons)
        self._enabled = False

    def refresh(self, state, enabled):
        self._enabled = enabled
        final = self.controls._path_stage() == "probe"
        self.setWindowTitle("Final Probe Path" if final else "Pre-approach Path")
        names = list(state.final_pathing_point_names if final else state.pathing_point_names) if state is not None else []
        current = [self.points.item(i).text() for i in range(self.points.count())]
        if names != current:
            row = self.points.currentRow()
            self.points.blockSignals(True)
            self.points.clear()
            self.points.addItems(names)
            self.points.setCurrentRow(min(max(row, 0), len(names) - 1))
            self.points.blockSignals(False)
        if state is not None:
            tolerances = state.final_pathing_point_tolerances_m if final else state.pathing_point_tolerances_m
            speeds = state.final_pathing_point_speed_scales if final else state.pathing_point_speed_scales
            if list(speeds) != self._point_speeds:
                self._point_speeds = list(speeds)
                self._select_speed(self.points.currentRow())
            for index, (tolerance, speed) in enumerate(zip(tolerances, speeds)):
                self.points.item(index).setToolTip(f"Position tolerance: {tolerance:g} m; speed: {100 * speed:g}%")
        self._update_buttons()
        if state is None or not state.refinement_active:
            self.hide()

    def _select_speed(self, row):
        if 0 <= row < len(self._point_speeds):
            self.speed_field.setValue(100 * self._point_speeds[row])

    def _update_buttons(self, *_):
        row = self.points.currentRow()
        selected = self._enabled and row >= 0
        self.adjust_button.setEnabled(self._enabled)
        self.move_button.setEnabled(selected)
        self.save_speed_button.setEnabled(selected)
        self.delete_button.setEnabled(selected)
        self.up_button.setEnabled(selected and row > 0)
        self.down_button.setEnabled(selected and row + 1 < self.points.count())
        name = self.name_field.text().strip()
        duplicate = any(
            self.points.item(i).text().strip().casefold() == name.casefold()
            for i in range(self.points.count())
        )
        self.name_error_label.setText("A pathing point with this name already exists." if duplicate else "")
        self.name_error_label.setVisible(duplicate)
        self.add_button.setEnabled(self._enabled and bool(name) and not duplicate)
        self.name_field.setEnabled(self._enabled)
        self.tolerance_field.setEnabled(self._enabled)
        self.speed_field.setEnabled(self._enabled)

    def _add(self):
        intent = ProbeSetupIntent()
        intent.path_stage = self.controls._path_stage()
        intent.operation = ProbeSetupIntent.OPERATION_ADD_PATHING_POINT
        intent.pathing_point_name = self.name_field.text().strip()
        intent.position_tolerance_m = self.tolerance_field.value()
        intent.arm_speed_scale = self.speed_field.value() / 100.0
        if self.controls._submit_probe_setup(intent):
            self._enabled = False
            self._update_buttons()

    def _save_speed(self):
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_SET_PATHING_POINT_SPEED
        intent.path_stage = self.controls._path_stage()
        intent.pathing_point_index = self.points.currentRow()
        intent.arm_speed_scale = self.speed_field.value() / 100.0
        if self.controls._submit_probe_setup(intent):
            self._enabled = False
            self._update_buttons()

    def _delete(self):
        intent = ProbeSetupIntent()
        intent.path_stage = self.controls._path_stage()
        intent.operation = ProbeSetupIntent.OPERATION_DELETE_PATHING_POINT
        intent.pathing_point_index = self.points.currentRow()
        if self.controls._submit_probe_setup(intent):
            self._enabled = False
            self._update_buttons()

    def _reorder(self, direction):
        intent = ProbeSetupIntent()
        intent.path_stage = self.controls._path_stage()
        intent.operation = ProbeSetupIntent.OPERATION_REORDER_PATHING_POINT
        intent.pathing_point_index = self.points.currentRow()
        intent.pathing_point_direction = direction
        if self.controls._submit_probe_setup(intent):
            self.points.setCurrentRow(intent.pathing_point_index + direction)
            self._enabled = False
            self._update_buttons()

    def _move(self):
        self.controls.handle_path_motion(
            ProbeSetupMotionIntent.OPERATION_MOVE_PATHING_POINT,
            self.points.currentRow(),
        )
