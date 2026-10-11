"""Check installed collision assets against the Spot geometry they represent."""

from collections import Counter
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest


HIPS = ("front_left_hip", "front_right_hip", "rear_left_hip", "rear_right_hip")


def read_obj(path):
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if fields and fields[0] == "v":
            vertices.append([float(value) for value in fields[1:4]])
        elif fields and fields[0] == "f":
            faces.append([int(value.split("/")[0]) - 1 for value in fields[1:]])
    return np.asarray(vertices), np.asarray(faces)


@pytest.mark.parametrize("link_name", (*HIPS, "arm_link_hr0"))
def test_closed_collision_envelopes_cover_their_source_geometry(link_name):
    package = Path(get_package_share_directory("spot_moveit_config"))
    source = Path(get_package_share_directory("spot_description"))
    model = ET.parse(package / "config/spot.urdf").getroot()
    collision, = model.findall(f"link[@name='{link_name}']/collision")
    assert collision.find("origin") is None
    mesh = collision.find("geometry/mesh")
    relative = f"config/collision/{link_name}.obj"
    assert mesh.attrib == {"filename": f"package://spot_moveit_config/{relative}"}
    vertices, faces = read_obj(package / relative)
    assert np.isfinite(vertices).all()
    assert faces.shape[1] == 3 and len(faces) < 2000
    assert faces.min() >= 0 and faces.max() < len(vertices)

    edges = Counter((a, b) for face in faces for a, b in zip(face, np.roll(face, -1)))
    assert all(count == 1 and edges[b, a] == 1 for (a, b), count in edges.items())
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    assert (lengths > 0).all()
    normals /= lengths[:, None]
    offsets = np.einsum("ij,ij->i", normals, triangles[:, 0])
    assert (vertices.mean(axis=0) @ normals.T - offsets < 0).all()

    paths = [f"meshes/base/visual/{link_name}.obj"] if link_name in HIPS else [
        "meshes/arm/visual/arm_link_sh1.obj",
        "meshes/arm/visual/arm_link_hr0.obj",
        "meshes/arm/collision/arm_link_hr0.obj",
    ]
    for path in paths:
        points, _ = read_obj(source / path)
        assert np.max(points @ normals.T - offsets) <= 1e-8, path


def test_shared_upper_arm_frame_and_hip_collision_permissions():
    package = Path(get_package_share_directory("spot_moveit_config"))
    model = ET.parse(package / "config/spot.urdf").getroot()
    joint = model.find("joint[@name='arm_hr0']")
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == "arm_link_sh1"
    assert joint.find("child").get("link") == "arm_link_hr0"
    origin = joint.find("origin")
    assert [float(value) for value in origin.get("xyz").split()] == [0, 0, 0]
    assert [float(value) for value in origin.get("rpy").split()] == [0, 0, 0]
    semantic = ET.parse(package / "config/spot.srdf").getroot()
    for pair in semantic.findall("disable_collisions"):
        links = {pair.get("link1"), pair.get("link2")}
        if links.intersection(HIPS):
            assert not any(name.startswith("arm_") for name in links)
            assert not links.intersection({"hand", "sensor_head_slot"})
