from abc import ABC, abstractmethod

from PyQt5.QtWidgets import QMessageBox


class UIControlHelper(ABC):
    def __init__(self, parent_ui: "Fault_Detector_UI"):
        self.ui = parent_ui
        self.node = self.ui.node
        self.status_label = getattr(parent_ui, "status_label", None)
        self.init_ros_communication()
        self.rows = self.make_rows()

    @abstractmethod
    def make_rows(self) -> list:
        """Return a list of QLayouts to add to the parent layout."""
        pass

    def init_ros_communication(self) -> list:
        """Return a list of QLayouts to add to the parent layout."""
        pass

    def add_rows(self, layout):
        for row in self.make_rows():
            layout.addLayout(row)

    # Convenience dialog methods
    def show_info(self, title: str, message: str):
        QMessageBox.information(self.ui, title, message)

    def show_warning(self, title: str, message: str):
        QMessageBox.warning(self.ui, title, message)

    def ask_question(self, title: str, message: str) -> bool:
        reply = QMessageBox.question(
            self.ui, title, message, QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        return reply

    def show_setup_unavailable(self, workflow: str) -> bool:
        """Report a setup workflow awaiting its coordinator API."""
        if hasattr(self.ui, "show_setup_unavailable"):
            return self.ui.show_setup_unavailable(workflow)
        if self.status_label is not None:
            self.status_label.setText(
                f"{workflow} is pending its coordinator API"
            )
        return False
