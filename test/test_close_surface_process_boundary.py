"""Lock close-surface execution into the shared arm movement boundary."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_launch_has_no_dedicated_close_surface_process():
    launch = read("launch/fault_detector_launch.py")
    setup = read("setup.py")

    assert 'executable="move_close_to_surface_node"' not in launch
    assert "manipulation.move_close_to_surface_node:main" not in setup


def test_close_surface_bt_behaviour_only_adapts_shared_execution():
    behaviour = read(
        "fault_detector_spot/manipulation/behaviours/"
        "move_close_to_surface_behaviour.py"
    )
    execution = read(
        "fault_detector_spot/manipulation/"
        "move_close_to_surface_execution.py"
    )

    assert "class MoveCloseToSurfaceBehaviour(ArmMovementBehaviour)" in behaviour
    assert "MoveCloseToSurfaceExecution" in behaviour
    assert "._execution.start(" in behaviour
    assert "._execution.poll(" in behaviour
    assert "._execution.cancel()" in behaviour
    assert "guarded_probe(" not in behaviour
    assert ".probe(" not in behaviour
    assert "_phase ==" not in behaviour
    assert "self.executor.cancel(" not in behaviour

    assert "class MoveCloseToSurfaceExecution" in execution
    assert "guarded_probe(" in execution
    assert ".probe(" in execution
    assert "ArmMovementOutcome.CONTACT" in execution
    assert "py_trees" not in execution
    assert "RobotCommandBuilder" not in execution
    assert "send_goal_async" not in execution
    assert "cancel_goal_async" not in execution


def test_probe_surface_source_only_owns_surface_sensing_and_attachment():
    source = read(
        "fault_detector_spot/inspection/sensing/probe_surface_source.py"
    )

    assert "class ProbeSurfaceSource(RuntimeSource)" in source
    assert "surface_distance_samples" in source
    assert "active_attachment" in source
    assert "END_EFFECTOR_FORCE_TOPIC" not in source
    assert "Vector3Stamped" not in source
    assert "current_hand_pose_execution" not in source
    assert "current_probe_pose_execution" not in source


def test_close_surface_execution_uses_shared_planner_for_live_arm_geometry():
    execution = read(
        "fault_detector_spot/manipulation/"
        "move_close_to_surface_execution.py"
    )

    assert "probe_motion_planner.current_pose(" in execution
    assert "probe_motion_planner.current_hand_pose(" in execution
