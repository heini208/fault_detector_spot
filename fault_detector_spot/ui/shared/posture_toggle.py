"""Shared feedback-driven stand/sit button for manual movement tabs."""

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QPushButton
from fault_detector_msgs.msg import OperationalIntent
from fault_detector_spot.navigation.posture_state_source import PostureState


class PostureToggle(QPushButton):
    def __init__(self, ui, source):
        super().__init__(ui)
        self.ui = ui
        self.source = source
        self.clicked.connect(self._toggle)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(250)
        self.refresh()

    def refresh(self):
        state = self.source.posture() if self.source is not None else None
        if state is PostureState.SITTING:
            text, color = "● Sitting — Stand", "#C62828"
            detail = "Driver reports sitting. Click to stand."
        elif state is PostureState.STANDING:
            text, color = "● Standing — Sit", "#2E7D32"
            detail = "Driver reports standing. Click to sit."
        else:
            text, color = "● Posture unknown", "#757575"
            detail = "Standing/sitting feedback is unavailable, stale, or ambiguous."
        self.setText(text)
        self.setStyleSheet(f"color: {color}; font-weight: bold;")
        self.setToolTip(detail)
        self.setEnabled(state in (PostureState.SITTING, PostureState.STANDING))

    def _toggle(self):
        state = self.source.posture() if self.source is not None else None
        if state is PostureState.SITTING:
            self.ui.handle_simple_operation(OperationalIntent.INTENT_STAND_UP)
        elif state is PostureState.STANDING:
            self.ui.handle_simple_operation(OperationalIntent.INTENT_SIT_DOWN)
        self.refresh()
