"""Compact disclosure of the latest historical landmark event."""

from datetime import datetime

from PyQt5.QtCore import QSize, Qt, pyqtSlot
from PyQt5.QtWidgets import QLabel

from fault_detector_spot.ui.shared.expandable_status_label import ExpandableStatusLabel


class LandmarkFeedbackLabel(ExpandableStatusLabel):
    def __init__(self, parent=None):
        super().__init__("Last landmark event: none reported", parent)
        self.setFixedWidth(20)
        self.setAlignment(Qt.AlignCenter)
        self.setAccessibleDescription(
            "Click or press Enter to show and copy the last landmark event."
        )

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
