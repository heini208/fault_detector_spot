"""Build the stock or custom collision body for the supported Spot jaw slot."""

import hashlib
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import numpy as np
from shape_msgs.msg import Mesh, MeshTriangle

from fault_detector_spot.inspection.model.sensor_geometry import (
    MAX_COLLISION_BYTES,
    SensorHeadGeometry,
    validate_collision_mesh,
)
from fault_detector_spot.inspection.model.sensor_models import quaternion_from_rpy_degrees
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.geometry.transforms import (
    inverse_pose,
    pose_data_to_pose,
    pose_to_matrix,
)


SENSOR_HEAD_OBJECT_ID = "fault_detector_sensor_head"
SENSOR_HEAD_SLOT_LINK = "sensor_head_slot"
CUSTOM_TOUCH_LINKS = ("hand", "arm_link_wr1", SENSOR_HEAD_SLOT_LINK)
# Preserve the original wrist link's SRDF adjacency rules for the stock jaw only.
STOCK_TOUCH_LINKS = (*CUSTOM_TOUCH_LINKS, "arm_link_wr0", "arm_link_fngr")
_COLLISION_PREFIX = "package://spot_description/meshes/arm/collision/"
_STOCK_MESH_SHA256 = {
    "front_jaw.obj": "29218801fefb0be66ae82c010000485d0ad03f5239b249961ddd6404357d6521",
    "middle_jaw.obj": "888a08c25355facdde9e143e9f928400ad56752c66393c00307278e40643156e",
    "jaw_tooth.obj": "317ee856d0c2d2cc83e435d141a03a38a26bcf56d4fcde666740e8abc20ab9cb",
}
_FINGER_MESHES = (
    "left_hinge.obj", "left_finger.obj", "left_tooth.obj",
    "right_hinge.obj", "right_finger.obj", "right_tooth.obj",
)


def sensor_head_attachment(
    geometry: SensorHeadGeometry, mesh_bytes: bytes,
) -> AttachedCollisionObject:
    """Place validated mount geometry once, relative to the hand datum."""
    geometry.validate()
    validate_collision_mesh(
        mesh_bytes, geometry.collision_sha256, geometry.collision_byte_count,
    )
    vertices = np.array([
        triangle[3:12]
        for triangle in struct.iter_unpack("<12fH", memoryview(mesh_bytes)[84:])
    ]).reshape(-1, 3)
    vertices, indices = np.unique(vertices, axis=0, return_inverse=True)
    mesh = _mesh_message(vertices, indices.reshape(-1, 3))
    return _attachment(
        [mesh], [pose_data_to_pose(geometry.hand_to_geometry)], CUSTOM_TOUCH_LINKS,
    )


def stock_head_attachment(
    robot_description: str, asset_root: Path = None,
) -> AttachedCollisionObject:
    """Restore the original three jaw meshes using the model's hand transform.

    ``asset_root`` is the spot_description package share directory. Runtime
    callers normally use the installed package; tests can supply a fixture.
    """
    model = _validated_model(robot_description)
    hand_joint = _joint_to(model, "hand")
    hand_to_wrist = inverse_pose(_origin_pose(hand_joint.find("origin")))
    if asset_root is None:
        asset_root = Path(get_package_share_directory("spot_description"))
    directory = Path(asset_root) / "meshes" / "arm" / "collision"
    meshes = [
        _stock_mesh(directory / name, digest)
        for name, digest in _STOCK_MESH_SHA256.items()
    ]
    return _attachment(meshes, [hand_to_wrist] * len(meshes), STOCK_TOUCH_LINKS)


def _attachment(meshes, poses, touch_links) -> AttachedCollisionObject:
    attached = AttachedCollisionObject()
    attached.link_name = SENSOR_HEAD_SLOT_LINK
    attached.touch_links = list(touch_links)
    attached.object.id = SENSOR_HEAD_OBJECT_ID
    attached.object.header.frame_id = "hand"
    attached.object.pose = pose_data_to_pose(PoseData.identity())
    attached.object.meshes = meshes
    attached.object.mesh_poses = poses
    attached.object.operation = CollisionObject.ADD
    return attached


def _mesh_message(vertices, faces) -> Mesh:
    return Mesh(
        vertices=[Point(x=float(x), y=float(y), z=float(z)) for x, y, z in vertices],
        triangles=[MeshTriangle(vertex_indices=[int(index) for index in face]) for face in faces],
    )


def _stock_mesh(path: Path, expected_sha256: str) -> Mesh:
    """Read only the known triangulated OBJ assets supplied by spot_description."""
    with path.open("rb") as stream:
        data = stream.read(MAX_COLLISION_BYTES + 1)
    if len(data) > MAX_COLLISION_BYTES or hashlib.sha256(data).hexdigest() != expected_sha256:
        raise ValueError(f"Unsupported stock jaw mesh: {path.name}")
    vertices, faces = [], []
    for line in data.decode("utf-8").splitlines():
        fields = line.split()
        if fields and fields[0] == "v":
            vertices.append([float(value) for value in fields[1:4]])
        elif fields and fields[0] == "f":
            faces.append([int(value.split("/")[0]) - 1 for value in fields[1:]])
    if (
        not vertices or not faces
        or any(len(face) != 3 for face in faces)
        or any(index < 0 or index >= len(vertices) for face in faces for index in face)
        or not np.isfinite(vertices).all()
    ):
        raise ValueError(f"Invalid stock jaw mesh: {path.name}")
    return _mesh_message(vertices, faces)


def validate_sensor_head_model(robot_description: str) -> None:
    """Require the jaw-slot model while preserving wrist and moving-finger shapes."""
    _validated_model(robot_description)


def _validated_model(robot_description: str):
    try:
        model = ET.fromstring(robot_description)
        links = {link.get("name"): link for link in model.findall("link")}
        if model.tag != "robot" or len(links) != len(model.findall("link")):
            raise ValueError("Invalid robot links")
        if "arm_link_wr0" not in links:
            raise ValueError("Missing wrist parent link")
        for name in ("hand", SENSOR_HEAD_SLOT_LINK):
            if links[name].find("collision") is not None:
                raise ValueError(f"{name} must have no built-in collision geometry")
        _require_joint(model, SENSOR_HEAD_SLOT_LINK, "hand", "fixed", (0, 0, 0))
        # This is the supported export datum, not a configurable geometry offset.
        _require_joint(model, "hand", "arm_link_wr1", "fixed", (.19557, 0, 0))
        _require_joint(
            model, "arm_link_fngr", "arm_link_wr1", "revolute", (.11745, 0, .01482),
        )
        _require_meshes(links["arm_link_wr1"], ("arm_link_wr1.obj",))
        _require_meshes(links["arm_link_fngr"], _FINGER_MESHES)
        return model
    except (ET.ParseError, KeyError, AttributeError, TypeError, ValueError) as error:
        raise ValueError(
            f"MoveIt robot model does not support the sensor jaw slot: {error}"
        ) from error


def _joint_to(model, child):
    matches = [
        joint for joint in model.findall("joint")
        if joint.find("child").get("link") == child
    ]
    if len(matches) != 1:
        raise ValueError(f"Expected one joint to {child}")
    return matches[0]


def _require_joint(model, child, parent, joint_type, translation) -> None:
    joint = _joint_to(model, child)
    if joint.get("type") != joint_type or joint.find("parent").get("link") != parent:
        raise ValueError(f"Unexpected joint for {child}")
    expected = np.eye(4)
    expected[:3, 3] = translation
    actual = pose_to_matrix(_origin_pose(joint.find("origin")))
    if not np.allclose(actual, expected, rtol=0, atol=1e-9):
        raise ValueError(f"Unexpected mounting datum for {child}")


def _require_meshes(link, expected) -> None:
    collisions = link.findall("collision")
    names = []
    for collision in collisions:
        mesh = collision.find("geometry/mesh")
        if mesh is None:
            raise ValueError("Expected stock collision meshes")
        scale = [float(value) for value in mesh.get("scale", "1 1 1").split()]
        if scale != [1, 1, 1]:
            raise ValueError("Expected unscaled stock collision meshes")
        names.append(mesh.get("filename"))
        placement = pose_to_matrix(_origin_pose(collision.find("origin")))
        if not np.allclose(placement, np.eye(4), rtol=0, atol=1e-9):
            raise ValueError("Stock collision mesh placement changed")
    if sorted(names) != sorted(_COLLISION_PREFIX + name for name in expected):
        raise ValueError(f"Unexpected permanent collision geometry on {link.get('name')}")


def _origin_pose(origin):
    if origin is None:
        return pose_data_to_pose(PoseData.identity())
    xyz = [float(value) for value in origin.get("xyz", "0 0 0").split()]
    rpy = [float(value) for value in origin.get("rpy", "0 0 0").split()]
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError("URDF origin must have three coordinates and angles")
    return pose_data_to_pose(PoseData(
        Vector3Data(*xyz), quaternion_from_rpy_degrees(*np.degrees(rpy)),
    ))


def validate_attached_geometry(actual, expected) -> None:
    """Check scene readback, including geometry, placement and allowed contacts."""
    if (
        actual.link_name != SENSOR_HEAD_SLOT_LINK
        or actual.object.id != SENSOR_HEAD_OBJECT_ID
        or actual.object.header.frame_id not in ("hand", SENSOR_HEAD_SLOT_LINK)
        or set(actual.touch_links) != set(expected.touch_links)
        or actual.object.primitives or actual.object.planes
        or len(actual.object.meshes) != len(expected.object.meshes)
        or len(actual.object.mesh_poses) != len(actual.object.meshes)
    ):
        raise ValueError("Sensor head attachment in MoveIt does not match the selected mount")
    actual_origin = pose_to_matrix(actual.object.pose)
    expected_origin = pose_to_matrix(expected.object.pose)
    for mesh, wanted, pose, wanted_pose in zip(
        actual.object.meshes, expected.object.meshes,
        actual.object.mesh_poses, expected.object.mesh_poses,
    ):
        vertices = [(point.x, point.y, point.z) for point in mesh.vertices]
        wanted_vertices = [(point.x, point.y, point.z) for point in wanted.vertices]
        if (
            len(vertices) != len(wanted_vertices)
            or not np.allclose(vertices, wanted_vertices, rtol=0, atol=1e-9)
            or [list(face.vertex_indices) for face in mesh.triangles]
            != [list(face.vertex_indices) for face in wanted.triangles]
            or not np.allclose(
                actual_origin @ pose_to_matrix(pose),
                expected_origin @ pose_to_matrix(wanted_pose), rtol=0, atol=1e-9,
            )
        ):
            raise ValueError("Sensor head mesh or placement in MoveIt changed")
