"""One shared arm adjustment window for probe setup and path authoring."""

from PyQt5.QtWidgets import (
    QDialog, QFormLayout, QGridLayout, QLabel,
    QPushButton, QVBoxLayout,
)

from .speed_slider import SpeedSlider


class FineAdjustmentDialog(QDialog):
    def __init__(self, controls, parent):
        super().__init__(parent)
        self.controls = controls
        self.pathing = False
        self.setWindowTitle("Fine Arm Adjustment")
        self.setModal(False)
        self.resize(440, 390)
        layout = QVBoxLayout(self)
        self.context_label = QLabel()
        self.context_label.setWordWrap(True)
        layout.addWidget(self.context_label)
        settings = QFormLayout()
        settings.addRow("Translation step [m]", controls.refine_translation_step_field)
        settings.addRow("Rotation step [deg]", controls.refine_rotation_step_field)
        self.speed_field = SpeedSlider()
        self.speed_field.setValue(100.0)
        self.speed_field.setToolTip("Percentage of configured linear and angular arm speed")
        settings.addRow("Speed", self.speed_field)
        self._final_stage = False
        self._stage_settings = {
            True: ("0.001", "1.0", 10.0),
        }
        settings.addRow("Adjustment frame", controls.refine_frame_dropdown)
        layout.addLayout(settings)
        grid = QGridLayout()
        actions = (("up", "down"), ("left", "right"), ("back", "front"),
                   ("pitch_up", "pitch_down"), ("yaw_left", "yaw_right"))
        for row, pair in enumerate(actions):
            for column, action in enumerate(pair):
                button = controls.refinement_buttons["alignment"][action]
                button.setAutoDefault(False)
                grid.addWidget(button, row, column)
        layout.addLayout(grid)
        close = QPushButton("Close Fine Adjustment")
        close.setAutoDefault(False)
        close.clicked.connect(self.hide)
        layout.addWidget(close)

    def set_stage(self, final):
        """Keep fine adjustments independent of saved candidate move speed."""
        if final == self._final_stage:
            return
        self._stage_settings[self._final_stage] = (
            self.controls.refine_translation_step_field.text(),
            self.controls.refine_rotation_step_field.text(),
            self.speed_field.value(),
        )
        translation, rotation, speed = self._stage_settings[final]
        self.controls.refine_translation_step_field.setText(translation)
        self.controls.refine_rotation_step_field.setText(rotation)
        self.speed_field.setValue(speed)
        self._final_stage = final

    def open_for(self, pathing=False):
        self.pathing = pathing
        final = self.controls._path_stage() == "probe"
        self.context_label.setText(
            "Adjust the arm, then capture it as a pathing point. The final aligned pose is preserved."
            if pathing else ("Adjust the final probe candidate." if final else "Adjust the final pre-approach candidate.")
        )
        self.controls._refresh_refinement_dialog()
        self.show()
        self.raise_()
        self.activateWindow()

    def refresh(self, alignment_enabled, pathing_enabled):
        enabled = pathing_enabled if self.pathing else alignment_enabled
        for button in self.controls.refinement_buttons["alignment"].values():
            button.setEnabled(enabled)
        for field in (self.controls.refine_translation_step_field,
                      self.controls.refine_rotation_step_field,
                      self.controls.refine_frame_dropdown, self.speed_field):
            field.setEnabled(enabled)
