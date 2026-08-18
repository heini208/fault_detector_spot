"""Validate move-command TF lifecycle and callback safety."""

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


def test_action_reset_preserves_initialized_resources():
    source = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "spot_action.py"
    )
    reset = _method_source(source, "_reset_state")

    assert "self.initialized = False" not in reset


def test_action_initialization_reuses_ros_resources():
    spot_action = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "spot_action.py"
    )
    move_action = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "move_command_action.py"
    )

    assert "if self._client is None:" in spot_action
    assert "get_action_client" in spot_action
    assert "get_action_client" in move_action
    assert "get_tf_listener" in move_action


def test_behavior_tree_uses_one_robot_command_resource_owner():
    resources = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "robot_command_resources.py"
    )
    helper = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "helper_initializer.py"
    )
    runner = _read(
        "fault_detector_spot/application/behaviour_tree/runner.py"
    )

    assert resources.count("ActionClientWrapper(") == 1
    assert resources.count("TFListenerWrapper(") == 1
    assert "self.robot_command_resources = RobotCommandResources()" in helper
    assert "helper_initializer.close()" in runner
    assert runner.count("robot_command_resources=") >= 10


def test_shared_robot_command_resources_have_explicit_teardown():
    resources = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "robot_command_resources.py"
    )

    close = _method_source(resources, "close")
    assert "tf_listener.shutdown" in close
    assert "client.destroy" in close


def test_terminal_goal_paths_do_not_send_cancel_requests():
    source = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "spot_action.py"
    )
    acceptance = _method_source(
        source,
        "_phase_wait_for_acceptance",
    )
    result = _method_source(source, "_phase_wait_for_result")

    assert "_cancel_goal" not in acceptance
    assert "_cancel_goal" not in result


def test_interrupted_active_goal_still_requests_cancellation():
    source = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "spot_action.py"
    )
    terminate = _method_source(source, "terminate")

    assert "new_status == Status.INVALID" in terminate
    assert "self._cancel_goal(self.goal_handle)" in terminate


def test_move_goal_preparation_exceptions_fail_the_behavior():
    source = _read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "move_command_action.py"
    )
    phase = _method_source(source, "_phase_send_goal")

    assert "except Exception as exception:" in phase
    assert "self._reset_state()" in phase
    assert "return Status.FAILURE" in phase


def test_move_goal_tf_lookups_are_nonblocking():
    paths = (
        "fault_detector_spot/application/behaviour_tree/commands/"
        "move_command.py",
        "fault_detector_spot/application/behaviour_tree/commands/"
        "move_to_tag_command.py",
        "fault_detector_spot/manipulation/behaviours/"
        "manipulator_move_relative_action.py",
    )

    for path in paths:
        source = _read(path)
        assert "timeout_sec=2" not in source
        assert "timeout_sec=0.0" in source
