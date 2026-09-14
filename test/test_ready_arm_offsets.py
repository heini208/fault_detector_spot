"""Ready-arm offsets are supplied by YAML and applied to the target pose."""

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import TransformStamped

from fault_detector_spot.manipulation import (
    arm_movement_executor as executor_module,
)
from fault_detector_spot.manipulation.arm_motion_parameters import (
    ArmMotionParameters,
)
from fault_detector_spot.manipulation.arm_state_source import (
    ArmStowState,
)


def test_ready_arm_applies_yaml_forward_and_lift_offsets(monkeypatch):
    config = ArmMotionParameters()
    current = TransformStamped()
    current.transform.translation.x = 0.2
    current.transform.translation.z = 0.4
    current.transform.rotation.w = 1.0
    captured = []
    monkeypatch.setattr(
        executor_module.RobotCommandBuilder,
        "arm_pose_command",
        lambda *args: captured.append(args),
    )
    monkeypatch.setattr(executor_module, "convert", lambda *_: None)
    executor = executor_module.ArmMovementExecutor(
        SimpleNamespace(lookup_a_tform_b=lambda *_args, **_kwargs: current),
        arm_state_source=SimpleNamespace(
            stow_state=lambda: ArmStowState.STOWED,
            is_stale=lambda: False,
        ),
        action_client=SimpleNamespace(
            wait_for_server=lambda **_: True,
            send_goal_async=lambda _: object(),
        ),
    )

    update = executor.prepare()
    assert update.outcome is executor_module.ArmMovementOutcome.RUNNING
    assert captured[0][0] == pytest.approx(
        0.2 + config.get("ready_forward_distance_m")
    )
    assert captured[0][2] == pytest.approx(
        0.4 + config.get("ready_lift_distance_m")
    )
