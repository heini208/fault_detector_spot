"""Compact overview panel for global UI state."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
)


class StatusOverviewPanel(QFrame):
    """Arrange global status, tags, buffer, and emergency stop compactly."""

    def __init__(
        self,
        status_label: QLabel,
        command_status_label: QLabel,
        navigation_mode_label: QLabel,
        visible_label: QLabel,
        buffer_label: QLabel,
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

        visible_label.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Preferred,
        )
        visible_label.setMinimumWidth(190)

        buffer_label.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Preferred,
        )
        buffer_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        layout.addWidget(status_label, 0, 0)
        layout.addWidget(command_status_label, 0, 1)
        layout.addWidget(navigation_mode_label, 0, 2)
        layout.addWidget(visible_label, 0, 3)
        layout.addWidget(estop_button, 0, 4, 2, 1)
        layout.addWidget(buffer_label, 1, 0, 1, 4)
        layout.setColumnStretch(3, 1)

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
