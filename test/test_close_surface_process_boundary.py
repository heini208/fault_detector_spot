"""Lock close-surface runtime ownership outside the behavior tree."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_launch_starts_dedicated_close_surface_process():
    launch = read("launch/fault_detector_launch.py")
    setup = read("setup.py")

    assert 'executable="move_close_to_surface_node"' in launch
    assert "manipulation.move_close_to_surface_node:main" in setup


def test_bt_close_surface_behavior_is_transport_only():
    client = read(
        "fault_detector_spot/manipulation/behaviours/"
        "manipulator_move_close_to_surface_action.py"
    )

    assert "ActionClient(" in client
    assert "MoveCloseToSurface.Goal" in client
    assert "ProbeSurfaceRuntimeStateSource" not in client
    assert "Image" not in client
    assert "Vector3Stamped" not in client
    assert "tf2_ros" not in client
    assert "RobotCommand" not in client


def test_server_owns_one_long_lived_runtime_source_and_robot_client():
    server = read(
        "fault_detector_spot/manipulation/move_close_to_surface_node.py"
    )

    assert server.count("ProbeSurfaceRuntimeStateSource(") == 1
    assert server.count("ActionClientWrapper(") == 1
    assert "MoveCloseToSurfaceOperation(" in server
    assert "MultiThreadedExecutor(num_threads=4)" in server


def test_runtime_source_unregisters_tf_listener_on_shutdown():
    runtime = read(
        "fault_detector_spot/inspection/execution/"
        "probe_surface_runtime_state.py"
    )

    assert "self._tf_listener.unregister()" in runtime
    assert "self._tf_listener = None" in runtime


def test_operation_has_no_behavior_tree_or_blackboard_dependency():
    operation = read(
        "fault_detector_spot/inspection/execution/"
        "move_close_to_surface_operation.py"
    )

    assert "py_trees" not in operation
    assert "blackboard" not in operation
    assert "MoveCloseToSurfaceStatus" in operation
