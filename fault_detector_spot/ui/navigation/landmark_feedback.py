"""Compact disclosure of current recovery status and the last landmark event."""

from collections import deque
from datetime import datetime

from PyQt5.QtCore import QSize, Qt, pyqtSlot
from PyQt5.QtWidgets import QLabel

from fault_detector_spot.ui.shared.expandable_status_label import ExpandableStatusLabel


class LandmarkFeedbackLabel(ExpandableStatusLabel):
    def __init__(self, parent=None):
        self._current_status = ""
        self._previous_statuses = deque(maxlen=2)
        super().__init__("Last landmark event: none reported", parent)
        self.setFixedWidth(20)
        self.setAlignment(Qt.AlignCenter)
        self.setAccessibleDescription(
            "Click or press Enter to show and copy current landmark status and the last event."
        )

    @pyqtSlot(str)
    def show_status(self, detail):
        if detail == self._current_status:
            return
        if self._current_status:
            self._previous_statuses.appendleft(self._current_status)
        self._current_status = detail
        self._refresh_tooltip()
        self._refresh_open_message()

    def _toggle_details(self):
        super()._toggle_details()
        if self._popup is not None:
            self._popup.setWindowTitle("Landmark status and recent messages")

    def _refresh_open_message(self):
        if self._popup is not None and self._popup.isVisible():
            self._message_view.setPlainText(self._message())

    def _message(self):
        historical = super()._message()
        if self._current_status:
            sections = [f"Current landmark status:\n{self._current_status}"]
            sections.extend(
                f"Previous status {index}:\n{detail}"
                for index, detail in enumerate(self._previous_statuses, 1)
            )
            return "\n\n".join([*sections, historical])
        return historical

    def sizeHint(self):
        return QSize(20, super().sizeHint().height())

    def _refresh_preview(self):
        expanded = self._popup is not None and self._popup.isVisible()
        QLabel.setText(self, "▾" if expanded else "▸")

    def hideEvent(self, event):
        if self._popup is not None:
            self._popup.close()
        super().hideEvent(event)

    @pyqtSlot(object)
    def show_event(self, event):
        try:
            timestamp = datetime.fromtimestamp(event.recorded_at).astimezone().isoformat(timespec="seconds")
        except (OverflowError, OSError, ValueError):
            timestamp = "unknown time"
        outcome = {
            "requested": "relocalization requested; awaiting confirmation",
            "confirmed": "relocalization confirmed",
            "failed": "relocalization failed",
            "refined": "landmark position refined",
        }[event.event]
        self.setText(
            f"Last landmark event: Tag {event.tag_id} · {outcome} · {event.map_id} · {timestamp}"
        )
        self.setToolTip(event.detail)
        self.setStyleSheet("color: #b35c00;" if event.event == "failed" else "")
        self._refresh_open_message()
