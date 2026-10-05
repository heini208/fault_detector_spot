"""Percentage speed control shared by probe authoring windows."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QSlider, QWidget


class SpeedSlider(QWidget):
    """Select a whole-number percentage of configured arm speed."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(1, 100)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(10)
        self.slider.setAccessibleName("Arm speed percentage")
        self.label = QLabel()
        self.label.setMinimumWidth(65)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.label)
        self.slider.valueChanged.connect(self._refresh_label)
        self.setToolTip("1–100% of configured arm speed; arrow keys change by 1%")
        self.setValue(100)

    def value(self):
        return self.slider.value()

    def setValue(self, percentage):
        self.slider.setValue(round(percentage))
        self._refresh_label()

    def _refresh_label(self, *_):
        self.label.setText(f"{self.value():g} %")
