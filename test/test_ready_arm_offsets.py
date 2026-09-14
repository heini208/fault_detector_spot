"""Lock configurable Ready-arm position offsets."""

from pathlib import Path

import yaml

ROOT = Path(__file__).parents[1]
EXECUTOR = ROOT / "fault_detector_spot" / "manipulation" / "arm_movement_executor.py"
CONFIG = ROOT / "config" / "arm_motion.yaml"


def test_ready_arm_applies_forward_and_lift_offsets():
    source = EXECUTOR.read_text(encoding="utf-8")

    assert 'READY_FORWARD_DISTANCE_PARAMETER = "arm.ready_forward_distance_m"' in source
    assert "target_hand.pose.position.x += self.ready_forward_distance_m" in source
    assert "target_hand.pose.position.z += self.ready_lift_distance_m" in source


def test_ready_arm_default_config_moves_forward_and_up():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    parameters = config["/**"]["ros__parameters"]

    assert parameters["arm.ready_forward_distance_m"] > 0.0
    assert parameters["arm.ready_lift_distance_m"] > 0.0
