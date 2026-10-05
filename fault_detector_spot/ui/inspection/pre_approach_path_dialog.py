"""Present server-owned pre-approach path editing and movement intents."""

from PyQt5.QtWidgets import (
    QDialog, QHBoxLayout, QLabel, QLineEdit, QListWidget, QPushButton, QVBoxLayout,
)
from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupMotionIntent


class PreApproachPathDialog(QDialog):
    def __init__(self, controls, parent):
        super().__init__(parent)
        self.controls = controls
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
        self.name_field.textChanged.connect(self._update_buttons)
        self._enabled = False

    def refresh(self, state, enabled):
        self._enabled = enabled
        names = list(state.pathing_point_names) if state is not None else []
        current = [self.points.item(i).text() for i in range(self.points.count())]
        if names != current:
            row = self.points.currentRow()
            self.points.blockSignals(True)
            self.points.clear()
            self.points.addItems(names)
            self.points.setCurrentRow(min(max(row, 0), len(names) - 1))
            self.points.blockSignals(False)
        self._update_buttons()
        if state is None or not state.refinement_active:
            self.hide()

    def _update_buttons(self, *_):
        row = self.points.currentRow()
        selected = self._enabled and row >= 0
        self.adjust_button.setEnabled(self._enabled)
        self.move_button.setEnabled(selected)
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

    def _add(self):
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_ADD_PATHING_POINT
        intent.pathing_point_name = self.name_field.text().strip()
        if self.controls._submit_probe_setup(intent):
            self._enabled = False
            self._update_buttons()

    def _delete(self):
        intent = ProbeSetupIntent()
        intent.operation = ProbeSetupIntent.OPERATION_DELETE_PATHING_POINT
        intent.pathing_point_index = self.points.currentRow()
        if self.controls._submit_probe_setup(intent):
            self._enabled = False
            self._update_buttons()

    def _reorder(self, direction):
        intent = ProbeSetupIntent()
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
