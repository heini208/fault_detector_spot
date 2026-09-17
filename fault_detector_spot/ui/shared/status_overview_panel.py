"""Compact overview panel for global UI state."""

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from std_msgs.msg import Float32


class StatusOverviewPanel(QFrame):
    """Separate robot-wide status from the compact sensor-system strip."""

    RECORDING_ACTIVE_STYLE = (
        "QPushButton {"
        " background-color: #C62828;"
        " color: white;"
        " border: 1px solid #B71C1C;"
        " border-radius: 4px;"
        " padding: 4px 10px;"
        " font-weight: bold;"
        "}"
        "QPushButton:hover {"
        " background-color: #B71C1C;"
        "}"
        "QPushButton:pressed {"
        " background-color: #8E0000;"
        "}"
    )

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
        sensor_recording_indicator_label: QLabel,
        sensor_recording_button: QPushButton,
        open_measurements_button: QPushButton,
        sensor_connection_indicator_label: QLabel,
        sensor_connection_status_label: QLabel,
        agent_indicator_label: QLabel,
        agent_endpoint_button: QPushButton,
        agent_copy_button: QPushButton,
        estop_button: QPushButton,
        parent=None,
    ):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        self.battery_label = QLabel("Battery: Unknown")
        self.battery_subscription = None
        self._recording_button = sensor_recording_button
        self._recording_button_base_style = sensor_recording_button.styleSheet()
        self._recording_button_active = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)

        robot_status = QWidget()
        robot_layout = QGridLayout(robot_status)
        robot_layout.setContentsMargins(0, 0, 0, 0)
        robot_layout.setHorizontalSpacing(12)
        robot_layout.setVerticalSpacing(8)

        for label in (
            status_label,
            command_status_label,
            self.battery_label,
        ):
            self._configure_status_label(label)

        navigation_mode_label.setAlignment(
            Qt.AlignVCenter | Qt.AlignLeft
        )
        navigation_mode_label.setSizePolicy(
            QSizePolicy.Maximum,
            QSizePolicy.Preferred,
        )
        navigation_mode_label.setStyleSheet("")

        for label in (
            buffer_label,
            visible_label,
        ):
            label.setSizePolicy(
                QSizePolicy.Expanding,
                QSizePolicy.Preferred,
            )

        buffer_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        visible_label.setMinimumWidth(190)
        self.battery_label.setSizePolicy(
            QSizePolicy.Maximum,
            QSizePolicy.Preferred,
        )

        robot_layout.addWidget(status_label, 0, 0)
        robot_layout.addWidget(command_status_label, 0, 1)
        robot_layout.addWidget(self.battery_label, 0, 2)
        robot_layout.addWidget(estop_button, 0, 4, 2, 1)

        robot_layout.addWidget(buffer_label, 1, 0)
        robot_layout.addWidget(visible_label, 1, 1)
        robot_layout.addWidget(navigation_mode_label, 1, 2)

        robot_layout.setColumnStretch(0, 2)
        robot_layout.setColumnStretch(1, 2)
        robot_layout.setColumnStretch(2, 2)
        robot_layout.setColumnStretch(3, 1)

        estop_button.setMinimumHeight(54)
        status_label.setWordWrap(True)
        buffer_label.setWordWrap(True)
        visible_label.setWordWrap(True)

        sensor_strip = QFrame()
        sensor_strip.setObjectName("sensorStatusStrip")
        sensor_strip.setFrameShape(QFrame.StyledPanel)
        sensor_strip.setStyleSheet(
            "QFrame#sensorStatusStrip {"
            " background: palette(alternate-base);"
            " border: 1px solid palette(mid);"
            " border-radius: 4px;"
            "}"
        )
        sensor_layout = QHBoxLayout(sensor_strip)
        sensor_layout.setContentsMargins(8, 5, 8, 5)
        sensor_layout.setSpacing(5)

        sensor_title = QLabel("Sensor system")
        sensor_title_font = sensor_title.font()
        sensor_title_font.setBold(True)
        sensor_title.setFont(sensor_title_font)
        sensor_layout.addWidget(sensor_title)
        sensor_layout.addWidget(self._separator())

        sensor_layout.addWidget(QLabel("micro-ROS:"))
        sensor_layout.addWidget(agent_indicator_label)
        sensor_layout.addWidget(agent_endpoint_button)
        sensor_layout.addWidget(agent_copy_button)
        sensor_layout.addWidget(self._separator())

        sensor_layout.addWidget(QLabel("Attachment:"))
        sensor_layout.addWidget(sensor_indicator_label)
        sensor_layout.addWidget(sensor_status_label)
        sensor_layout.addWidget(sensor_confirm_button)
        sensor_layout.addWidget(self._separator())

        sensor_layout.addWidget(QLabel("Head:"))
        sensor_layout.addWidget(sensor_connection_indicator_label)
        sensor_layout.addWidget(sensor_connection_status_label)
        sensor_layout.addWidget(self._separator())

        sensor_layout.addWidget(QLabel("Recording:"))
        sensor_layout.addWidget(sensor_recording_button)
        sensor_layout.addWidget(open_measurements_button)
        sensor_layout.addStretch()

        sensor_recording_indicator_label.hide()
        sensor_status_label.setSizePolicy(
            QSizePolicy.Maximum,
            QSizePolicy.Preferred,
        )

        layout.addWidget(robot_status)
        layout.addWidget(sensor_strip)

        self.sensor_widget = sensor_strip
        self.agent_widget = sensor_strip
        self.hardware_widget = sensor_strip
        self.sensor_strip = sensor_strip
        self.grid_layout = robot_layout

        self._recording_style_timer = QTimer(self)
        self._recording_style_timer.timeout.connect(
            self._refresh_recording_button_style
        )
        self._recording_style_timer.start(100)
        self._refresh_recording_button_style()
        self._init_battery_subscription(parent)

    def _refresh_recording_button_style(self) -> None:
        active = self._recording_button.text().strip().lower() == "stop"
        if active == self._recording_button_active:
            return
        self._recording_button_active = active
        self._recording_button.setStyleSheet(
            self.RECORDING_ACTIVE_STYLE
            if active
            else self._recording_button_base_style
        )

    @staticmethod
    def _separator() -> QFrame:
        separator = QFrame()
        separator.setFrameShape(QFrame.VLine)
        separator.setFrameShadow(QFrame.Sunken)
        separator.setFixedHeight(22)
        return separator

    def _init_battery_subscription(self, parent) -> None:
        node = getattr(parent, "node", None)
        if node is None:
            return
        self.battery_subscription = node.create_subscription(
            Float32,
            "fault_detector/state/battery_percentage",
            self._process_battery_percentage,
            LATCHED_QOS,
        )

    def _process_battery_percentage(self, message: Float32) -> None:
        self.battery_label.setText(f"Battery: {message.data:.0f}%")

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
