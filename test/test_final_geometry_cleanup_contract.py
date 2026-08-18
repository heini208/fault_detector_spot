"""Contract guards for shared transform boundary hardening."""

from pathlib import Path


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def source(relative):
    return (ROOT / relative).read_text(encoding="utf-8")


def test_shared_transform_boundary_fails_closed_on_invalid_geometry():
    transforms = source("shared/geometry/transforms.py")

    assert "Quaternion contains a non-finite value" in transforms
    assert "Pose position contains a non-finite value" in transforms
    assert "Transform matrix has an invalid homogeneous row" in transforms
    assert "Transform rotation matrix is not orthonormal" in transforms
    assert "Transform rotation matrix is not proper" in transforms
