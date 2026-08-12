"""Regression guards for the reduced ROS interface surface."""

import re
from pathlib import Path

from fault_detector_msgs.msg import LiveInspectionObjectState


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
    package_root = ROOT / "fault_detector_spot"
    combined = "\n".join(
        path.read_text(encoding="utf-8")
        for path in package_root.rglob("*.py")
    )

    assert LiveInspectionObjectState.__name__ == "LiveInspectionObjectState"
    assert re.search(r"\bLiveInspectionObjectState\b", combined)
    assert not re.search(r"\bInspectionObjectStateArray\b", combined)
    assert not re.search(r"\bInspectionObjectState\b", combined)
