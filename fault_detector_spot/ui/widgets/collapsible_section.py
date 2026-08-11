"""Compact collapsible section widget."""

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QFrame,
    QToolButton,
    QVBoxLayout,
    QWidget,
)


class CollapsibleSection(QWidget):
    """Show optional content behind a compact disclosure button."""

    def __init__(self, title, content, expanded=False, parent=None):
        super().__init__(parent)
        self.toggle_button = QToolButton()
        self.toggle_button.setText(title)
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(expanded)
        self.toggle_button.setToolButtonStyle(
            Qt.ToolButtonTextBesideIcon
        )
        self.toggle_button.toggled.connect(self._set_expanded)

        self.content_frame = QFrame()
        content_layout = QVBoxLayout(self.content_frame)
        content_layout.setContentsMargins(10, 4, 0, 4)
        content_layout.addWidget(content)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addWidget(self.toggle_button)
        layout.addWidget(self.content_frame)

        self._set_expanded(expanded)

    def _set_expanded(self, expanded):
        self.content_frame.setVisible(expanded)
        arrow = Qt.DownArrow if expanded else Qt.RightArrow
        self.toggle_button.setArrowType(arrow)
