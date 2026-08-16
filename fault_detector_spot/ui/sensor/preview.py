"""Standalone preview window for the sensor mount UI."""

import sys

from PyQt5.QtWidgets import QApplication, QMainWindow

from .controls import SensorControls


class SensorUiPreview(QMainWindow):
    """Show the presentation-only sensor mount workspace."""

    def __init__(self):
        """Build the standalone sensor mount preview."""
        super().__init__()
        self.setWindowTitle("Fault Detector Spot - Sensor Mount UI Preview")
        self.resize(1000, 720)

        controls = SensorControls()
        controls.add_preview_mount(
            "Hall probe mount",
            "bmm150_mount",
            "bmm150_mount_probe",
            "Calibrated",
        )
        controls.add_preview_mount(
            "Multi-sensor mount",
            "multi_sensor_mount",
            "multi_sensor_mount_probe",
            "Not calibrated",
        )
        self.setCentralWidget(controls)
        self.sensor_controls = controls


def main():
    """Run the standalone sensor mount UI preview."""
    app = QApplication.instance() or QApplication(sys.argv)
    preview = SensorUiPreview()
    preview.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
