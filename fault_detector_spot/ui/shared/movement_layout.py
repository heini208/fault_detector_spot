"""Layout primitives shared by the manual movement tabs."""

from PyQt5.QtWidgets import QGroupBox, QHBoxLayout, QVBoxLayout


def control_group(title, *rows):
    group = QGroupBox(title)
    layout = QVBoxLayout(group)
    layout.setContentsMargins(12, 12, 12, 10)
    layout.setSpacing(8)
    for row in rows:
        layout.addLayout(row)
    wrapper = QHBoxLayout()
    wrapper.addWidget(group)
    return wrapper


def emphasize_action(button):
    """Give the main explicit movement action a consistent visual weight."""
    button.setMinimumHeight(32)
    button.setStyleSheet(
        "QPushButton:enabled { background: palette(highlight); color: palette(highlighted-text); "
        "font-weight: bold; border: none; border-radius: 4px; padding: 6px 12px; }"
    )
