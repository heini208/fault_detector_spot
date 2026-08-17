"""Guard application orchestration placement."""

from pathlib import Path


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def test_application_orchestration_uses_application_layer_paths():
    expected = (
        ROOT / "application/coordinators/setup_coordinator.py",
        ROOT / "application/coordinators/navigation_setup_coordinator.py",
        ROOT / "application/coordinators/probe_setup_coordinator.py",
        ROOT / "application/coordinators/probe_refinement_finalization_coordinator.py",
        ROOT / "application/api/navigation_setup_api.py",
        ROOT / "application/api/probe_setup_api.py",
        ROOT / "application/api/probe_setup_motion_api.py",
        ROOT / "application/api/probe_refinement_finalization_api.py",
    )
    obsolete = (
        ROOT / "application/api/probe_surface_verification_api.py",
        ROOT / "application/setup/setup_coordinator.py",
        ROOT / "navigation/setup/navigation_setup_coordinator.py",
        ROOT / "navigation/setup/navigation_setup_api.py",
        ROOT / "inspection/setup/probe_setup_coordinator.py",
        ROOT / "inspection/setup/probe_refinement_finalization.py",
        ROOT / "inspection/setup/probe_setup_api.py",
        ROOT / "inspection/setup/probe_setup_motion_api.py",
        ROOT / "inspection/setup/probe_surface_verification_api.py",
        ROOT / "inspection/setup/probe_refinement_finalization_api.py",
    )

    assert all(path.is_file() for path in expected)
    assert all(not path.exists() for path in obsolete)
