"""Persistent, readable display of the latest historical landmark event."""

from datetime import datetime

from PyQt5.QtCore import pyqtSlot

from fault_detector_spot.ui.shared.expandable_status_label import ExpandableStatusLabel


class LandmarkFeedbackLabel(ExpandableStatusLabel):
    def __init__(self, parent=None):
        super().__init__("Last landmark event: none reported", parent)

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
