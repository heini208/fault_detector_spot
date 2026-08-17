"""Contract guards for the final geometry helper cleanup."""

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


def test_probe_setup_state_adapter_no_longer_writes_legacy_surface_verification():
    adapter = source("inspection/setup/probe_setup_state_adapter.py")

    assert "surface_verification_state" not in adapter
    assert "surface_verification_request_id" not in adapter
    assert "surface_recovery_required" not in adapter
