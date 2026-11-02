"""Validate movement TF preparation and shared RobotCommand resources."""

from pathlib import Path

ROOT = Path(__file__).parents[1]


def _read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def _method_source(source, method_name):
    marker = f"    def {method_name}("
    body = source.split(marker, 1)[1]
    next_method = body.find("\n    def ")
    if next_method >= 0:
        body = body[:next_method]
    return body


def test_behavior_tree_uses_one_robot_command_resource_owner():
    resources = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "robot_command_resources.py"
    )
    helper = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "helper_initializer.py"
    )
    assert resources.count("ActionClientWrapper(") == 1
    assert resources.count("TFListenerWrapper(") == 1
    assert "get_arm_movement_executor(" in resources
    assert "get_base_movement_executor(" in resources
    assert "get_arm_joint_state_source(" in resources
    assert "get_arm_contact_telemetry(" in resources
    assert "executor.guarded_probe_execution.contact_telemetry" in resources
    assert "get_posture_state_source(" in resources
    assert "self.robot_command_resources = RobotCommandResources()" in helper


def test_shared_robot_command_resources_have_explicit_teardown():
    from unittest.mock import Mock
    from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import RobotCommandResources

    resources = RobotCommandResources()
    expected = []
    for field, method in (
        ("_tf_listener", "shutdown"), ("_arm_contact_telemetry", "close"),
        ("_arm_state_source", "destroy"), ("_arm_joint_state_source", "destroy"),
        ("_posture_state_source", "destroy"),
    ):
        owner = Mock()
        setattr(resources, field, owner)
        expected.append(getattr(owner, method))
    arm, base, client = Mock(), Mock(), Mock()
    resources._arm_movement_executors[""] = arm
    resources._base_movement_executors[""] = base
    resources._clients[""] = client
    expected.extend((arm.shutdown, base.shutdown, client.destroy))
    resources.report_shutdown_confirmed()
    resources.close()
    resources.close()
    for release in expected:
        release.assert_called_once()


def test_movement_preparation_exceptions_fail_the_behaviour():
    source = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "movement_behaviour.py"
    )
    update = _method_source(source, "update")
    assert "except Exception as exception:" in update
    assert "return self._fail(" in update


def test_move_goal_tf_lookups_are_nonblocking():
    paths = (
        "fault_detector_spot/application/behaviour_tree/commands/"
        "move_command.py",
        "fault_detector_spot/application/behaviour_tree/commands/"
        "move_to_tag_command.py",
        "fault_detector_spot/manipulation/probe_motion_planner.py",
        "fault_detector_spot/navigation/base_motion_planner.py",
    )
    for path in paths:
        source = _read(path)
        assert "timeout_sec=2" not in source
        assert "timeout_sec=0.0" in source


def test_legacy_move_command_action_is_removed():
    assert not (
        ROOT
        / "fault_detector_spot/application/behaviour_tree/behaviours/"
        "move_command_action.py"
    ).exists()
