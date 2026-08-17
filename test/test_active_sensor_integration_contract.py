"""Contract guards for authoritative active-sensor motion ownership."""

from pathlib import Path


ROOT = Path(__file__).parents[1] / "fault_detector_spot"


def source(relative):
    return (ROOT / relative).read_text(encoding="utf-8")


def test_application_injects_one_attachment_authority_at_construction():
    application = source("application/api/application_api_node.py")

    assert "self.sensor_attachment_controller = SensorAttachmentController(" in (
        application
    )
    assert "self.command_controller.add_request_preparer(" in application
    assert "sensor_attachment_controller=(" in application
    assert "set_sensor_attachment_controller" not in application


def test_probe_geometry_and_live_orientation_use_confirmed_attachment():
    geometry = source("inspection/setup/probe_geometry_editor.py")
    api = source("application/api/probe_setup_api.py")
    coordinator = source(
        "application/coordinators/probe_setup_coordinator.py"
    )

    assert "attachment = self._active_attachment()" in geometry
    assert "attachment.hand_to_probe()" in geometry
    assert "routine.sensor_id" not in geometry
    assert "calculate_surface_orientation(" in api
    assert "motion_attachment()" in coordinator
    assert "attachment.hand_to_probe().orientation" in coordinator
    assert "routine.sensor_id" not in api


def test_close_surface_runtime_resolves_active_attachment_without_setup_context():
    runtime = source(
        "inspection/execution/probe_surface_runtime_state.py"
    )
    action = source(
        "manipulation/behaviours/manipulator_move_close_to_surface_action.py"
    )
    application = source("application/api/application_api_node.py")

    assert "def active_attachment(" in runtime
    assert "active_sensor_id" in runtime
    assert "attachment_revision" in runtime
    assert "probe_setup_coordinator" not in runtime
    assert "setup_context" not in runtime
    assert "active_attachment()" in action
    assert "ExecuteOperation" in application
    assert "ProbeSurfaceVerificationApi" not in application


def test_tag_motion_converts_probe_target_to_hand_once():
    factory = source("inspection/setup/probe_setup_motion.py")
    command = source("manipulation/commands/manipulator_to_tag_command.py")

    assert "probe_pose_to_hand_pose" not in factory
    assert "_probe_target_to_hand_target" in command
    assert "transform_to_pose_data(hand_to_probe)" in command
    assert "inverse_pose(hand_to_probe_pose)" in command
    assert "compose_poses(" in command
    assert "tf_transformations" not in command


def test_future_probe_execution_configuration_accepts_active_sensor_snapshot():
    target = source("inspection/execution/probe_execution_target.py")
    session = source("inspection/execution/probe_execution_session.py")

    assert "routine.sensor_id !=" not in target
    assert "attachment: MotionAttachmentSnapshot" in session
    assert "attachment_revision" in session
    assert "sensor_repository.load(routine.sensor_id)" not in session


def test_refinement_reserves_one_attachment_until_workflow_end():
    controller = source(
        "application/coordinators/probe_refinement_controller.py"
    )

    assert "acquire_motion_attachment()" in controller
    assert "self._attachment_reservations" in controller
    assert "self._release_attachment(draft)" in controller
    assert "active.sensor_id" not in controller
    assert "active.motion_sensor_id" in controller

    finalization = source(
        "application/coordinators/probe_finalization_controller.py"
    )
    assert "self.refinement_controller.end(draft)" in finalization


def test_transport_and_workflows_use_public_coordinator_boundaries():
    api = source("application/api/probe_setup_api.py")
    application = source("application/api/application_api_node.py")
    setup = source("application/coordinators/setup_coordinator.py")
    workflow_paths = (
        "application/coordinators/navigation_setup_coordinator.py",
        "application/coordinators/probe_reference_capture_coordinator.py",
        "application/coordinators/probe_refinement_controller.py",
        "application/coordinators/probe_setup_coordinator.py",
        "application/api/navigation_setup_api.py",
        "application/api/probe_reference_capture_api.py",
    )
    workflows = "\n".join(source(path) for path in workflow_paths)

    assert ".refinement_controller" not in api
    assert ".object_repository" not in api
    assert "coordinator.reference_repository" not in api
    assert "self.preview_source = preview_source" in api
    assert "ProbeReferencePreviewSource(reference_repository)" in application
    assert "ProbeSurfaceVerificationApi" not in application
    assert ".setup_coordinator.is_current" not in api
    assert "def cancel_operation(" in setup
    assert "def require_command_lane_idle(" in setup
    assert ".setup_coordinator.command_controller" not in workflows
    assert "probe_setup_coordinator.setup_coordinator" not in workflows
    assert "self.coordinator.setup_coordinator" not in workflows
    assert "coordinator._context_lock" not in workflows
    assert "coordinator._selected_draft" not in workflows
    assert "set_sensor_attachment_controller" not in workflows
