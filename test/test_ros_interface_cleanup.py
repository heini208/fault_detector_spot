"""Regression guards for the reduced ROS interface surface."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def _source(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_setup_close_endpoints_share_one_service_contract():
    sources = (
        "fault_detector_spot/application/api/navigation_setup_api.py",
        "fault_detector_spot/application/api/probe_setup_api.py",
        "fault_detector_spot/ui/ros/navigation_setup_client.py",
        "fault_detector_spot/ui/ros/probe_setup_client.py",
    )
    combined = "\n".join(_source(path) for path in sources)

    assert "CloseSetup" in combined
    assert "CloseNavigationSetup" not in combined
    assert "CloseProbeSetup" not in combined
    assert "fault_detector/application/close_navigation_setup" in combined
    assert "fault_detector/application/close_probe_setup" in combined


def test_live_object_state_is_the_active_object_state_contract():
    adapter = _source(
        "fault_detector_spot/inspection/live_object_state_adapter.py"
    )
    publisher = _source(
        "fault_detector_spot/behaviour_tree/nodes/inspection/"
        "publish_live_inspection_object.py"
    )
    combined = adapter + "\n" + publisher

    assert "LiveInspectionObjectState" in combined
    assert "InspectionObjectStateArray" not in combined
