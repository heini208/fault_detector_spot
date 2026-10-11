"""File selection and presentation for sensor-tip transform imports."""

from PyQt5.QtWidgets import QDialog, QFileDialog

from fault_detector_spot.inspection.model.sensor_models import (
    rpy_degrees_from_quaternion,
)
from fault_detector_spot.inspection.repository.sensor_head_import import (
    load_sensor_head_transform,
)


def import_sensor_head_transform(controls) -> None:
    """Read a chosen package and fill the current form after validation."""
    if not controls.import_mount_button.isEnabled():
        return
    dialog = QFileDialog(controls, "Import mount_creator ZIP")
    dialog.setFileMode(QFileDialog.ExistingFile)
    dialog.setAcceptMode(QFileDialog.AcceptOpen)
    dialog.setNameFilter("Sensor head packages (*.sensorhead)")
    dialog.setLabelText(QFileDialog.Accept, "Import")
    if dialog.exec_() != QDialog.Accepted:
        return
    paths = dialog.selectedFiles()
    if not paths or not controls.import_mount_button.isEnabled():
        return
    try:
        pose = load_sensor_head_transform(paths[0])
        rotation = rpy_degrees_from_quaternion(pose.orientation)
    except (OSError, ValueError) as exception:
        controls.transform_status_value.setText(f"Import failed: {exception}")
        return
    controls.apply_imported_transform(
        (pose.position.x, pose.position.y, pose.position.z),
        rotation,
    )
