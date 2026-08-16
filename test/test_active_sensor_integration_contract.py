"""Contract guards for authoritative active-sensor motion ownership."""

from pathlib import Path


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def source(relative):
    return (ROOT / relative).read_text(encoding="utf-8")


def test_application_installs_one_attachment_authority_on_motion_paths():
    application = source("application/api/application_api_node.py")

    assert "self.sensor_attachment_controller = SensorAttachmentController(" in (
        application
    )
    assert "self.command_controller.add_request_preparer(" in application
    assert (
        "self.probe_setup_coordinator.geometry_editor."
        "set_sensor_attachment_controller("
    ) in application
    assert (
        "self.probe_setup_coordinator.refinement_controller."
        "set_sensor_attachment_controller("
    ) in application


def test_probe_geometry_and_live_orientation_use_confirmed_attachment():
    geometry = source("inspection/setup/probe_geometry_editor.py")
    api = source("application/api/probe_setup_api.py")

    assert "attachment = self._active_attachment()" in geometry
    assert "attachment.hand_to_probe()" in geometry
    assert "routine.sensor_id" not in geometry
    assert "confirmed_attachment()" in api
    assert "routine.sensor_id" not in api


def test_surface_verification_reserves_one_attachment_for_whole_run():
    runner = source(
        "application/coordinators/probe_surface_verification_runner.py"
    )

    assert "reserve_confirmed_attachment()" in runner
    assert "attachment.sensor_id" in runner
    assert "routine.sensor_id" not in runner


def test_tag_motion_converts_probe_target_to_hand_once():
    factory = source("inspection/setup/probe_setup_motion.py")
    command = source("manipulation/commands/manipulator_to_tag_command.py")

    assert "probe_pose_to_hand_pose" not in factory
    assert "_probe_target_to_hand_target" in command
    assert "tf.inverse_matrix(hand_to_probe_matrix)" in command


def test_future_probe_execution_configuration_accepts_active_sensor_snapshot():
    target = source("inspection/execution/probe_execution_target.py")
    session = source("inspection/execution/probe_execution_session.py")

    assert "routine.sensor_id !=" not in target
    assert "sensor_definition: SensorDefinition" in session
    assert "sensor_repository.load(routine.sensor_id)" not in session
