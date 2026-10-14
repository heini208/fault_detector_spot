"""Ready-arm targets are fixed in body, independent of the starting pose."""

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import TransformStamped

from fault_detector_spot.manipulation import (
    arm_movement_executor as executor_module,
    arm_command_builder as command_builder_module,
)
from fault_detector_spot.manipulation.arm_motion_parameters import (
    ArmMotionParameters,
)
from fault_detector_spot.manipulation.arm_state_source import (
    ArmStowState,
)


@pytest.mark.parametrize("start_x", [.2, .8])
@pytest.mark.parametrize("state", [ArmStowState.STOWED, ArmStowState.DEPLOYED])
def test_ready_arm_uses_fixed_yaml_body_pose(monkeypatch, start_x, state):
    config = ArmMotionParameters()
    current = TransformStamped()
    current.transform.translation.x = start_x
    current.transform.translation.z = 0.4
    current.transform.rotation.w = 1.0
    captured = []
    monkeypatch.setattr(
        command_builder_module.RobotCommandBuilder,
        "arm_pose_command",
        lambda *args: captured.append(args),
    )
    monkeypatch.setattr(command_builder_module, "convert", lambda *_: None)
    executor = executor_module.ArmMovementExecutor(
        SimpleNamespace(lookup_a_tform_b=lambda *_args, **_kwargs: current),
        arm_state_source=SimpleNamespace(
            stow_state=lambda: state,
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
        config.get("ready_position_x_m")
    )
    assert captured[0][2] == pytest.approx(
        config.get("ready_position_z_m")
    )

    assert captured[0][1] == config.get("ready_position_y_m")
    assert captured[0][3:7] == (1., 0., 0., 0.)
    assert captured[0][7] == "body"
