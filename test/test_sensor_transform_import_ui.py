"""Offline coverage of mount package import and sensor definition drafts."""

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
from fault_detector_spot.inspection.repository.sensor_head_import import (
    load_sensor_head,
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
    vertices = ((0, 0, 0), (0.01, 0, 0), (0, 0.01, 0), (0, 0, 0.01))
    faces = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
    mesh = b"\0" * 80 + struct.pack("<I", len(faces)) + b"".join(
        struct.pack(
            "<12fH", 0, 0, 0,
            *(coordinate for index in face for coordinate in vertices[index]),
            0,
        )
        for face in faces
    )
    manifest = {
        "schema_version": 1,
        "reference_frame": "hand",
        "units": "m",
        "display_name": "Exporter name",
        "mounting_template_id": "spot-hand-reference-v1",
        "mounting_template_revision": 1,
        "reference_model_fingerprint": (
            "a5746b4090dafe81c140c62d69be15dc682d5835d5384c6bbda8592e53391f46"
        ),
        "mounting_template_verified": False,
        "configuration_method": "manual",
        "clearance_m": 0,
        "hand_to_probe": {
            "translation": [0.046238241757377466, -0.00107, -0.07917394514872324],
            "quaternion": [rotation.x, rotation.y, rotation.z, rotation.w],
        },
        "hand_to_geometry": {
            "translation": [0.09, 0.08, 0.07],
            "quaternion": [0.0, 0.0, 0.0, 1.0],
        },
        "collision_mesh": {
            "path": "collision.stl",
            "byte_count": len(mesh),
            "sha256": hashlib.sha256(mesh).hexdigest(),
        },
    }
    path = tmp_path / "head.sensorhead"
    with ZipFile(path, "w") as archive:
        archive.writestr("manifest.json", json.dumps(manifest))
        archive.writestr("collision.stl", mesh)
    return path


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
    imported = load_sensor_head(package)
    assert saves[0].head_geometry == imported.head_geometry
    assert saves[0].collision_mesh == imported.collision_mesh
    assert saves[0].orientation == tuple(
        getattr(imported.hand_to_probe.orientation, axis) for axis in "xyzw"
    )


@pytest.mark.parametrize("outcome", ("cancel", "invalid", "missing"))
def test_unsuccessful_import_preserves_existing_values(
    controls, package, tmp_path, monkeypatch, outcome,
):
    select_file(monkeypatch, package)
    controls.import_mount_button.click()
    controls.translation_fields[0].setText("0.1234")
    controls.rotation_fields[2].setText("45.0")
    intent_before = controls._definition_intent()
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
    assert controls._definition_intent() == intent_before
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
    assert not controls.save_mount_button.isEnabled()

    controls.finish_sensor_save(False, "Save failed")
    assert controls.import_mount_button.isEnabled()
    assert controls.save_mount_button.isEnabled()


def test_upload_progress_cannot_overwrite_the_save_result(controls):
    controls.mark_sensor_save_pending()

    controls.set_mesh_upload_progress(48 * 1024, 164 * 1024)

    assert controls.transform_status_value.text() == (
        "Uploading collision geometry: 48 / 164 KiB"
    )
    controls.finish_sensor_save(False, "Upload failed")
    controls.set_mesh_upload_progress(96 * 1024, 164 * 1024)
    assert controls.transform_status_value.text() == "Upload failed"


def test_main_upload_progress_handles_controls_not_yet_constructed(controls):
    from fault_detector_spot.ui.fault_detector_ui import Fault_Detector_UI

    ui = SimpleNamespace()
    handler = Fault_Detector_UI._process_sensor_mesh_upload_progress

    handler(ui, 0, 1024)

    ui.sensor_controls = controls
    controls.mark_sensor_save_pending()
    handler(ui, 512, 1024)
    assert "Uploading collision geometry:" in controls.transform_status_value.text()


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
    controls.mount_id_field.setText("other")
    controls.display_name_field.setText("Other sensor")
    intent = controls._definition_intent()
    assert intent.head_geometry is None
    assert intent.collision_mesh == b""
    assert intent.orientation is None


def test_geometry_indicator_tracks_import_and_discard(
    controls, package, monkeypatch,
):
    assert "No collision geometry" in controls.collision_geometry_status.text()
    assert "#c62828" in controls.collision_geometry_status.styleSheet()
    select_file(monkeypatch, package)

    controls.import_mount_button.click()

    assert "ready to save" in controls.collision_geometry_status.text()
    assert "#a66300" in controls.collision_geometry_status.styleSheet()
    intent_before = controls._definition_intent()
    controls.mark_sensor_save_pending()
    controls.finish_sensor_save(False, "Save failed")
    assert "ready to save" in controls.collision_geometry_status.text()
    assert controls._definition_intent() == intent_before

    controls.discard_mount_button.click()

    assert "No collision geometry" in controls.collision_geometry_status.text()
    assert "#c62828" in controls.collision_geometry_status.styleSheet()


@pytest.mark.parametrize("problem", ("missing", "checksum", "placement"))
def test_invalid_geometry_does_not_partially_apply_a_new_tip(
    controls, package, tmp_path, monkeypatch, problem,
):
    select_file(monkeypatch, package)
    controls.import_mount_button.click()
    before = controls._definition_intent()
    with ZipFile(package) as archive:
        manifest = json.loads(archive.read("manifest.json"))
        mesh = archive.read("collision.stl")
    manifest["hand_to_probe"]["translation"][0] = 1.23
    if problem == "missing":
        del manifest["collision_mesh"]
    elif problem == "checksum":
        manifest["collision_mesh"]["sha256"] = "0" * 64
    else:
        manifest["hand_to_geometry"]["quaternion"] = [0, 0, 0, 0]
    path = tmp_path / "invalid.sensorhead"
    with ZipFile(path, "w") as archive:
        archive.writestr("manifest.json", json.dumps(manifest))
        archive.writestr("collision.stl", mesh)
    select_file(monkeypatch, path)

    controls.import_mount_button.click()

    assert controls._definition_intent() == before
    assert "Import failed:" in controls.transform_status_value.text()


@pytest.mark.parametrize("has_geometry", (True, False))
def test_opening_saved_mount_replaces_draft_geometry(
    controls, package, monkeypatch, has_geometry,
):
    imported = load_sensor_head(package)
    geometry = imported.head_geometry if has_geometry else None
    controls.apply_definitions((SimpleNamespace(
        sensor_id="saved", display_name="Saved mount", probe_frame="saved_probe",
        position=(0.1, 0.2, 0.3), orientation=(0.0, 0.0, 0.0, 1.0),
        rotation_degrees=(0.0, 0.0, 0.0), channels=(), head_geometry=geometry,
    ),))
    controls.apply_attachment_state(SimpleNamespace(
        selected_sensor_id="", status="active",
    ))
    select_file(monkeypatch, package)
    controls.import_mount_button.click()
    controls.mount_table.selectRow(0)

    controls.edit_selected_sensor()

    assert controls.mount_id_field.text() == "saved"
    intent = controls._definition_intent()
    assert intent.head_geometry == geometry
    assert intent.collision_mesh == b""
    if has_geometry:
        assert "geometry saved" in controls.collision_geometry_status.text()
        assert "#2e7d32" in controls.collision_geometry_status.styleSheet()
    else:
        assert "No collision geometry" in controls.collision_geometry_status.text()
    controls.translation_fields[0].setText("0.123")
    assert controls._definition_intent().head_geometry == geometry


def test_import_preserves_full_tip_precision_until_corresponding_fields_change(
    controls, package, monkeypatch,
):
    with ZipFile(package) as archive:
        pose = json.loads(archive.read("manifest.json"))["hand_to_probe"]
    select_file(monkeypatch, package)
    controls.import_mount_button.click()

    intent = controls._definition_intent()

    assert intent.translation_m == tuple(pose["translation"])
    assert intent.orientation == tuple(pose["quaternion"])
    controls.translation_fields[0].setText("0.123")
    intent = controls._definition_intent()
    assert intent.translation_m == (0.123, *pose["translation"][1:])
    assert intent.orientation == tuple(pose["quaternion"])
    controls.rotation_fields[2].setText("25")
    intent = controls._definition_intent()
    assert intent.orientation is None
    assert intent.rotation_degrees[2] == 25
    assert intent.head_geometry == load_sensor_head(package).head_geometry
