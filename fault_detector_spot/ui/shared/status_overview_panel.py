"""Compact overview panel for global UI state."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QWidget,
)


class StatusOverviewPanel(QFrame):
    """Arrange global status, buffer, tags, sensor, and emergency stop."""

    def __init__(
        self,
        status_label: QLabel,
        command_status_label: QLabel,
        navigation_mode_label: QLabel,
        visible_label: QLabel,
        buffer_label: QLabel,
        sensor_indicator_label: QLabel,
        sensor_status_label: QLabel,
        sensor_confirm_button: QPushButton,
        estop_button: QPushButton,
        parent=None,
    ):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)

        layout = QGridLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(5)

        for label in (
            status_label,
            command_status_label,
            navigation_mode_label,
        ):
            self._configure_status_label(label)

        for label in (
            buffer_label,
            visible_label,
            sensor_status_label,
        ):
            label.setSizePolicy(
                QSizePolicy.Expanding,
                QSizePolicy.Preferred,
            )

        buffer_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        visible_label.setMinimumWidth(190)
        sensor_status_label.setMinimumWidth(150)

        sensor_widget = QWidget()
        sensor_layout = QHBoxLayout(sensor_widget)
        sensor_layout.setContentsMargins(0, 0, 0, 0)
        sensor_layout.setSpacing(4)
        sensor_layout.addWidget(QLabel("Sensor:"))
        sensor_layout.addWidget(sensor_indicator_label)
        sensor_layout.addWidget(sensor_status_label, 1)
        sensor_layout.addWidget(sensor_confirm_button)

        layout.addWidget(status_label, 0, 0)
        layout.addWidget(command_status_label, 0, 1)
        layout.addWidget(navigation_mode_label, 0, 2)
        layout.addWidget(estop_button, 0, 3, 2, 1)

        layout.addWidget(buffer_label, 1, 0)
        layout.addWidget(visible_label, 1, 1)
        layout.addWidget(sensor_widget, 1, 2)

        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 2)
        layout.setColumnStretch(2, 2)

        self.sensor_widget = sensor_widget
        self.grid_layout = layout

    @staticmethod
    def _configure_status_label(label: QLabel) -> None:
        label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        label.setSizePolicy(
            QSizePolicy.Maximum,
            QSizePolicy.Preferred,
        )
        label.setStyleSheet(
            "QLabel {"
            " background: palette(alternate-base);"
            " border: 1px solid palette(mid);"
            " border-radius: 4px;"
            " padding: 3px 7px;"
            "}"
        )
