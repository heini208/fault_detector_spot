"""Offline geometry, model-compatibility and scene-readback checks."""

from copy import deepcopy
from dataclasses import replace
import hashlib
import struct
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Pose
import numpy as np
import pytest
from shape_msgs.msg import SolidPrimitive

from fault_detector_spot.inspection.model.sensor_geometry import (
    MOUNTING_TEMPLATE_ID,
    MOUNTING_TEMPLATE_REVISION,
    REFERENCE_MODEL_FINGERPRINT,
    SensorHeadGeometry,
)
from fault_detector_spot.inspection.model.sensor_models import quaternion_from_rpy_degrees
from fault_detector_spot.manipulation import sensor_head_geometry as geometry
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.geometry.transforms import pose_to_matrix


@pytest.fixture
def model():
    root = ET.Element("robot", name="spot")
    for name in ("arm_link_wr0", "hand", "sensor_head_slot"):
        ET.SubElement(root, "link", name=name)
    for name, meshes in (
        ("arm_link_wr1", ("arm_link_wr1.obj",)),
        ("arm_link_fngr", geometry._FINGER_MESHES),
    ):
        link = ET.SubElement(root, "link", name=name)
        for filename in meshes:
            shape = ET.SubElement(ET.SubElement(link, "collision"), "geometry")
            ET.SubElement(shape, "mesh", filename=geometry._COLLISION_PREFIX + filename)
    for child, parent, joint_type, translation in (
        ("hand", "arm_link_wr1", "fixed", "0.19557 0 0"),
        ("sensor_head_slot", "hand", "fixed", "0 0 0"),
        ("arm_link_fngr", "arm_link_wr1", "revolute", "0.11745 0 0.01482"),
    ):
        joint = ET.SubElement(root, "joint", name=f"to_{child}", type=joint_type)
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz=translation)
    return root


@pytest.fixture
def mesh_bytes():
    points = ((0., 0., 0.), (.1, 0., 0.), (0., .1, 0.), (0., 0., .1))
    faces = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
    return bytes(80) + struct.pack("<I", 4) + b"".join(
        struct.pack(
            "<12fH", 0, 0, 0,
            *(coordinate for index in face for coordinate in points[index]), 0,
        )
        for face in faces
    )


@pytest.fixture
def metadata(mesh_bytes):
    return SensorHeadGeometry(
        collision_sha256=hashlib.sha256(mesh_bytes).hexdigest(),
        collision_byte_count=len(mesh_bytes),
        hand_to_geometry=PoseData(
            Vector3Data(-.11, .042, -.065), quaternion_from_rpy_degrees(90, -1, 0),
        ),
        mounting_template_id=MOUNTING_TEMPLATE_ID,
        mounting_template_revision=MOUNTING_TEMPLATE_REVISION,
        reference_model_fingerprint=REFERENCE_MODEL_FINGERPRINT,
        mounting_template_verified=False,
    )


def test_custom_geometry_keeps_vertices_in_source_frame_and_places_once(metadata, mesh_bytes):
    attached = geometry.sensor_head_attachment(metadata, mesh_bytes)

    assert attached.link_name == "sensor_head_slot"
    assert attached.object.id == "fault_detector_sensor_head"
    assert attached.object.header.frame_id == "hand"
    assert set(attached.touch_links) == {"hand", "arm_link_wr1", "sensor_head_slot"}
    assert np.allclose(pose_to_matrix(attached.object.pose), np.eye(4))
    assert attached.object.mesh_poses[0].position.x == -.11
    assert len(attached.object.meshes[0].vertices) == 4
    assert len(attached.object.meshes[0].triangles) == 4
    assert max(point.x for point in attached.object.meshes[0].vertices) == pytest.approx(.1)


def test_custom_geometry_rechecks_content_identity(metadata, mesh_bytes):
    with pytest.raises(ValueError, match="checksum"):
        geometry.sensor_head_attachment(replace(metadata, collision_sha256="0" * 64), mesh_bytes)


def test_supported_model_accepts_slot_and_retained_wrist_and_finger(model):
    geometry.validate_sensor_head_model(ET.tostring(model, encoding="unicode"))


@pytest.mark.parametrize("damage", (
    "slot_missing", "slot_parent", "slot_offset", "slot_collision", "hand_offset",
    "stock_left_in_wrist", "wrist_removed", "finger_removed", "finger_parent",
))
def test_model_validation_rejects_incompatible_collision_model(model, damage):
    slot = model.find("link[@name='sensor_head_slot']")
    slot_joint = model.find("joint[@name='to_sensor_head_slot']")
    wrist = model.find("link[@name='arm_link_wr1']")
    if damage == "slot_missing":
        model.remove(slot)
    elif damage == "slot_parent":
        slot_joint.find("parent").set("link", "arm_link_wr1")
    elif damage == "slot_offset":
        slot_joint.find("origin").set("xyz", "0.001 0 0")
    elif damage == "slot_collision":
        ET.SubElement(slot, "collision")
    elif damage == "hand_offset":
        model.find("joint[@name='to_hand']/origin").set("xyz", "0.2 0 0")
    elif damage == "stock_left_in_wrist":
        collision = deepcopy(wrist.find("collision"))
        collision.find("geometry/mesh").set("filename", geometry._COLLISION_PREFIX + "front_jaw.obj")
        wrist.append(collision)
    elif damage == "wrist_removed":
        wrist.remove(wrist.find("collision"))
    elif damage == "finger_removed":
        finger = model.find("link[@name='arm_link_fngr']")
        finger.remove(finger.find("collision"))
    else:
        model.find("joint[@name='to_arm_link_fngr']/parent").set("link", "hand")

    with pytest.raises(ValueError, match="does not support"):
        geometry.validate_sensor_head_model(ET.tostring(model, encoding="unicode"))


def test_stock_profile_uses_original_files_and_inverse_hand_joint(model, tmp_path, monkeypatch):
    source = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
    directory = tmp_path / "meshes" / "arm" / "collision"
    directory.mkdir(parents=True)
    digests = {}
    for name in ("front_jaw.obj", "middle_jaw.obj", "jaw_tooth.obj"):
        (directory / name).write_bytes(source)
        digests[name] = hashlib.sha256(source).hexdigest()
    monkeypatch.setattr(geometry, "_STOCK_MESH_SHA256", digests)
    description = ET.tostring(model, encoding="unicode")

    attached = geometry.stock_head_attachment(description, tmp_path)

    assert len(attached.object.meshes) == 3
    assert [pose.position.x for pose in attached.object.mesh_poses] == [-.19557] * 3
    assert {"arm_link_wr0", "arm_link_fngr"}.issubset(attached.touch_links)
    assert attached.object.meshes[0].vertices[1].x == 1
    (directory / "front_jaw.obj").write_bytes(b"changed")
    with pytest.raises(ValueError, match="Unsupported stock jaw mesh"):
        geometry.stock_head_attachment(description, tmp_path)


def test_readback_accepts_equivalent_frame_and_quaternion_sign(metadata, mesh_bytes):
    expected = geometry.sensor_head_attachment(metadata, mesh_bytes)
    actual = deepcopy(expected)
    actual.object.header.frame_id = "sensor_head_slot"
    actual.touch_links.reverse()
    pose = actual.object.mesh_poses[0]
    for name in ("x", "y", "z", "w"):
        setattr(pose.orientation, name, -getattr(pose.orientation, name))

    geometry.validate_attached_geometry(actual, expected)


def test_readback_accepts_equivalent_object_pose_split(metadata, mesh_bytes):
    expected = geometry.sensor_head_attachment(metadata, mesh_bytes)
    actual = deepcopy(expected)
    actual.object.pose = actual.object.mesh_poses[0]
    actual.object.mesh_poses = [Pose()]
    actual.object.mesh_poses[0].orientation.w = 1.

    geometry.validate_attached_geometry(actual, expected)


@pytest.mark.parametrize("damage", (
    "link", "id", "frame", "touch", "vertices", "faces", "pose", "primitive", "missing_pose",
))
def test_readback_rejects_changed_collision_attachment(metadata, mesh_bytes, damage):
    expected = geometry.sensor_head_attachment(metadata, mesh_bytes)
    actual = deepcopy(expected)
    if damage == "link":
        actual.link_name = "arm_link_wr1"
    elif damage == "id":
        actual.object.id = "other"
    elif damage == "frame":
        actual.object.header.frame_id = "body"
    elif damage == "touch":
        actual.touch_links.append("body")
    elif damage == "vertices":
        actual.object.meshes[0].vertices[0].x += .001
    elif damage == "faces":
        actual.object.meshes[0].triangles[0].vertex_indices = [0, 0, 0]
    elif damage == "pose":
        actual.object.mesh_poses[0].position.x += .001
    elif damage == "primitive":
        actual.object.primitives = [SolidPrimitive()]
    else:
        actual.object.mesh_poses = []

    with pytest.raises(ValueError):
        geometry.validate_attached_geometry(actual, expected)
