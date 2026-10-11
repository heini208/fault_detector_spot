"""Offline coverage of the file-picker-to-editable-transform workflow."""

import json
import hashlib
import os
import struct
from types import SimpleNamespace
from zipfile import ZipFile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication, QDialog, QFileDialog

from fault_detector_spot.inspection.model.sensor_models import (
    quaternion_from_rpy_degrees,
)
from fault_detector_spot.ui.sensor.controls import SensorControls
from fault_detector_spot.ui.sensor.import_dialog import (
    import_sensor_head_transform,
)


@pytest.fixture(scope="module", autouse=True)
def application():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def controls(application):
    widget = SensorControls()
    widget.import_requested.connect(
        lambda: import_sensor_head_transform(widget)
    )
    widget.mount_id_field.setText("probe")
    widget.display_name_field.setText("My sensor")
    yield widget
    widget.close()
    widget.deleteLater()


@pytest.fixture
def package(tmp_path):
    rotation = quaternion_from_rpy_degrees(12.5, -31.25, 84.75)
    manifest = {
        "schema_version": 1,
        "reference_frame": "hand",
        "units": "m",
        "display_name": "Exporter name",
        "hand_to_probe": {
            "translation": [0.046238241757377466, -0.00107, -0.07917394514872324],
            "quaternion": [rotation.x, rotation.y, rotation.z, rotation.w],
        },
        "hand_to_geometry": {
            "translation": [9.0, 8.0, 7.0],
            "quaternion": [0.0, 0.0, 0.0, 1.0],
        },
    }
    path = tmp_path / "head.sensorhead"
    with ZipFile(path, "w") as archive:
        archive.writestr("manifest.json", json.dumps(manifest))
    return path


@pytest.fixture
def package_with_geometry(package):
    with ZipFile(package) as archive:
        manifest = json.loads(archive.read("manifest.json"))
    mesh = b"\0" * 80 + struct.pack("<I", 1) + struct.pack(
        "<12fH", 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0,
    )
    manifest["collision_mesh"] = {
        "path": "collision.stl",
        "byte_count": len(mesh),
        "sha256": hashlib.sha256(mesh).hexdigest(),
    }
    with ZipFile(package, "w") as archive:
        archive.writestr("manifest.json", json.dumps(manifest))
        archive.writestr("collision.stl", mesh)
    return package


def select_file(monkeypatch, path, accepted=True, during_dialog=None):
    def execute(dialog):
        assert dialog.fileMode() == QFileDialog.ExistingFile
        assert dialog.acceptMode() == QFileDialog.AcceptOpen
        assert dialog.labelText(QFileDialog.Accept) == "Import"
        assert dialog.nameFilters() == [
            "Sensor head packages (*.sensorhead)"
        ]
        if during_dialog is not None:
            during_dialog()
        return QDialog.Accepted if accepted else QDialog.Rejected

    monkeypatch.setattr(QFileDialog, "exec_", execute)
    monkeypatch.setattr(QFileDialog, "selectedFiles", lambda self: [str(path)])


def field_values(controls):
    return tuple(
        field.text()
        for field in (*controls.translation_fields, *controls.rotation_fields)
    )


def test_import_fills_probe_transform_preserving_identity_and_channels(
    controls, package, monkeypatch,
):
    select_file(monkeypatch, package)
    original_channels = controls._channel_values()
    saves = []
    controls.create_requested.connect(saves.append)

    controls.import_mount_button.click()

    values = tuple(float(value) for value in field_values(controls))
    assert values == pytest.approx(
        (0.046238241757377466, -0.00107, -0.07917394514872324,
         12.5, -31.25, 84.75),
        rel=0, abs=1e-12,
    )
    assert controls.mount_id_field.text() == "probe"
    assert controls.display_name_field.text() == "My sensor"
    assert controls.probe_frame_field.text() == "probe_probe"
    assert controls._channel_values() == original_channels
    assert saves == []
    assert all(not field.isReadOnly() for field in controls.translation_fields)
    controls.translation_fields[0].setText("0.051")
    controls.save_mount_button.click()
    assert len(saves) == 1
    assert saves[0].translation_m[0] == 0.051
    assert saves[0].rotation_degrees == pytest.approx((12.5, -31.25, 84.75))


@pytest.mark.parametrize("outcome", ("cancel", "invalid", "missing"))
def test_unsuccessful_import_preserves_existing_values(
    controls, tmp_path, monkeypatch, outcome,
):
    controls.translation_fields[0].setText("0.1234")
    controls.rotation_fields[2].setText("45.0")
    controls.set_collision_geometry_available(True)
    geometry_before = controls.collision_geometry_status.text()
    before = field_values(controls)
    status_before = controls.transform_status_value.text()
    path = tmp_path / "invalid.zip"
    if outcome != "missing":
        path.write_bytes(b"Not a ZIP archive")
    select_file(monkeypatch, path, accepted=outcome != "cancel")

    controls.import_mount_button.click()

    assert field_values(controls) == before
    assert controls.collision_geometry_status.text() == geometry_before
    if outcome == "cancel":
        assert controls.transform_status_value.text() == status_before
    else:
        assert controls.transform_status_value.text().startswith("Import failed:")


def test_import_is_disabled_during_save_and_restored_on_failure(controls):
    controls.mark_sensor_save_pending()
    controls.apply_attachment_state(SimpleNamespace(
        selected_sensor_id="", status="active",
    ))

    assert not controls.import_mount_button.isEnabled()

    controls.finish_sensor_save(False, "Save failed")
    assert controls.import_mount_button.isEnabled()


def test_import_rechecks_attachment_after_dialog(controls, package, monkeypatch):
    definition = SimpleNamespace(
        sensor_id="probe", display_name="My sensor", probe_frame="probe_probe",
        position=(0.1, 0.2, 0.3), rotation_degrees=(0.0, 0.0, 0.0), channels=(),
    )
    controls.apply_definitions((definition,))
    controls.apply_attachment_state(SimpleNamespace(
        selected_sensor_id="", status="active",
    ))
    controls.mount_table.selectRow(0)
    controls.edit_selected_sensor()
    before = field_values(controls)
    select_file(
        monkeypatch, package,
        during_dialog=lambda: controls.apply_attachment_state(SimpleNamespace(
            selected_sensor_id="probe", status="active",
        )),
    )

    controls.import_mount_button.click()

    assert not controls.import_mount_button.isEnabled()
    assert field_values(controls) == before


def test_discard_resets_imported_transform(controls, package, monkeypatch):
    select_file(monkeypatch, package)
    controls.import_mount_button.click()

    controls.discard_mount_button.click()

    assert tuple(float(value) for value in field_values(controls)) == (0.0,) * 6
    assert controls.import_mount_button.isEnabled()


def test_geometry_indicator_tracks_import_and_discard(
    controls, package_with_geometry, monkeypatch,
):
    assert "No collision geometry" in controls.collision_geometry_status.text()
    assert "#c62828" in controls.collision_geometry_status.styleSheet()
    select_file(monkeypatch, package_with_geometry)

    controls.import_mount_button.click()

    assert "found in imported file" in controls.collision_geometry_status.text()
    assert "#2e7d32" in controls.collision_geometry_status.styleSheet()
    controls.mark_sensor_save_pending()
    controls.finish_sensor_save(False, "Save failed")
    assert "found in imported file" in controls.collision_geometry_status.text()

    controls.discard_mount_button.click()

    assert "No collision geometry" in controls.collision_geometry_status.text()
    assert "#c62828" in controls.collision_geometry_status.styleSheet()


def test_transform_only_import_clears_previous_geometry_indicator(
    controls, package, monkeypatch,
):
    controls.set_collision_geometry_available(True)
    select_file(monkeypatch, package)

    controls.import_mount_button.click()

    assert "No collision geometry" in controls.collision_geometry_status.text()
    assert float(controls.translation_fields[0].text()) == pytest.approx(
        0.046238241757377466
    )


def test_opening_saved_mount_clears_draft_geometry_status(controls):
    controls.apply_definitions((SimpleNamespace(
        sensor_id="saved", display_name="Saved mount", probe_frame="saved_probe",
        position=(0.1, 0.2, 0.3), rotation_degrees=(0.0, 0.0, 0.0), channels=(),
    ),))
    controls.apply_attachment_state(SimpleNamespace(
        selected_sensor_id="", status="active",
    ))
    controls.set_collision_geometry_available(True)
    controls.mount_table.selectRow(0)

    controls.edit_selected_sensor()

    assert controls.mount_id_field.text() == "saved"
    assert "No collision geometry" in controls.collision_geometry_status.text()
