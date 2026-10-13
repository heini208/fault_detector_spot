"""Fixed-height status preview with a selectable, copyable detail popup."""

from html import escape

from PyQt5.QtCore import QEvent, QPoint, QSize, Qt
from PyQt5.QtWidgets import (
    QApplication, QDialog, QHBoxLayout, QLabel, QPlainTextEdit,
    QPushButton, QSizePolicy, QVBoxLayout,
)


class ExpandableStatusLabel(QLabel):
    """Keep existing label updates while preventing message-driven resizing."""

    def __init__(self, text="", parent=None, *, elide_mode=Qt.ElideRight):
        super().__init__(parent)
        self._full_text = ""
        self._detail = ""
        self._popup = None
        self._elide_mode = elide_mode
        self.setTextFormat(Qt.PlainText)
        self.setWordWrap(False)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setMinimumWidth(0)
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setAccessibleDescription("Press Enter to expand and copy the full status message.")
        self._update_height()
        self.setText(text)

    def setText(self, text):
        self._full_text = str(text)
        self._detail = ""
        self.setAccessibleName(self._full_text)
        self._refresh_preview()
        self._refresh_tooltip()

    def text(self):
        return self._full_text

    def clear(self):
        self.setText("")

    def setToolTip(self, text):
        # Command status already supplies its detailed feedback this way.
        self._detail = str(text)
        self._refresh_tooltip()

    def _message(self):
        if self._detail and self._detail != self._full_text:
            return f"{self._full_text}\n\n{self._detail}" if self._full_text else self._detail
        return self._full_text

    def _refresh_tooltip(self):
        # Escape errors containing <, > or ampersands rather than treating
        # diagnostic text as HTML. Very long messages remain readable on click.
        preview = self._message()
        if len(preview) > 1000:
            preview = preview[:1000] + "…"
        super().setToolTip(
            "<qt>" + escape(preview).replace("\n", "<br>")
            + "<br><br>Click or press Enter to expand and copy the full message.</qt>"
        )

    def _refresh_preview(self):
        prefix = "▾ " if self._popup is not None and self._popup.isVisible() else "▸ "
        summary = " ".join(self._full_text.split())
        available = max(0, self.contentsRect().width() - 18)
        super().setText(self.fontMetrics().elidedText(prefix + summary, self._elide_mode, available))

    def sizeHint(self):
        return QSize(260, self.fontMetrics().height() + 14)

    def minimumSizeHint(self):
        return QSize(0, self.sizeHint().height())

    def _update_height(self):
        self.setFixedHeight(self.sizeHint().height())

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh_preview()

    def changeEvent(self, event):
        super().changeEvent(event)
        if hasattr(self, "_full_text") and event.type() in (
            QEvent.FontChange, QEvent.ApplicationFontChange, QEvent.StyleChange,
        ):
            self._update_height()
            self._refresh_preview()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._toggle_details()
            event.accept()
        else:
            super().mouseReleaseEvent(event)

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Return, Qt.Key_Enter, Qt.Key_Space):
            self._toggle_details()
            event.accept()
        else:
            super().keyPressEvent(event)

    def _toggle_details(self):
        if self._popup is not None and self._popup.isVisible():
            self._popup.close()
            return
        if self._popup is None:
            # A modeless tool window keeps the other controls interactive.
            self._popup = QDialog(self, Qt.Tool)
            self._popup.setWindowTitle("Status message")
            self._popup.installEventFilter(self)
            layout = QVBoxLayout(self._popup)
            self._message_view = QPlainTextEdit()
            self._message_view.setReadOnly(True)
            self._message_view.setAccessibleName("Full status message")
            layout.addWidget(self._message_view)
            buttons = QHBoxLayout()
            copy_button = QPushButton("Copy full message")
            copy_button.clicked.connect(
                lambda: QApplication.clipboard().setText(self._message_view.toPlainText())
            )
            close_button = QPushButton("Close")
            close_button.clicked.connect(self._popup.close)
            buttons.addWidget(copy_button)
            buttons.addStretch()
            buttons.addWidget(close_button)
            layout.addLayout(buttons)
        # Hold a snapshot while open so new feedback cannot interrupt reading
        # or copying. Closing and reopening shows the latest message.
        self._message_view.setPlainText(self._message())
        anchor = self.mapToGlobal(QPoint(0, self.height()))
        screen = QApplication.screenAt(anchor) or QApplication.primaryScreen()
        bounds = screen.availableGeometry().adjusted(8, 8, -8, -8)
        width, height = min(640, bounds.width()), min(300, bounds.height())
        self._popup.resize(width, height)
        self._popup.move(
            max(bounds.left(), min(anchor.x(), bounds.right() - width + 1)),
            max(bounds.top(), min(anchor.y(), bounds.bottom() - height + 1)),
        )
        self._popup.show()
        self._message_view.setFocus()
        self._refresh_preview()

    def eventFilter(self, watched, event):
        if watched is self._popup and event.type() == QEvent.Hide:
            self._refresh_preview()
        return super().eventFilter(watched, event)
