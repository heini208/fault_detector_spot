"""Lock the end-to-end sit-down command route."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_sit_down_command_is_wired_through_all_spot_layers():
    command_ids = read(
        "fault_detector_spot/application/commanding/command_ids.py"
    )
    adapter = read(
        "fault_detector_spot/application/ros/"
        "operational_intent_adapter.py"
    )
    subscriber = read(
        "fault_detector_spot/application/behaviour_tree/behaviours/"
        "command_subscriber.py"
    )
    runner = read(
        "fault_detector_spot/application/behaviour_tree/runner.py"
    )
    behaviour = read(
        "fault_detector_spot/navigation/behaviours/"
        "sit_down_behaviour.py"
    )
    executor = read(
        "fault_detector_spot/navigation/base_movement_executor.py"
    )
    ui = read(
        "fault_detector_spot/ui/shared/posture_toggle.py"
    )

    assert 'SIT_DOWN = "sit_down"' in command_ids
    assert (
        "OperationalIntent.INTENT_SIT_DOWN: CommandID.SIT_DOWN"
        in adapter
    )
    assert "CommandID.SIT_DOWN: self._simple_command" in subscriber
    assert "CommandID.SIT_DOWN" in runner
    assert "SitDownBehaviour(" in runner
    assert "return self.executor.sit()" in behaviour
    assert "def sit(" in executor
    assert "RobotCommandBuilder.synchro_sit_command()" in executor
    assert "self.ui.handle_simple_operation(OperationalIntent.INTENT_SIT_DOWN)" in ui
