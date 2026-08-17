"""Build and run the main robot behavior tree."""

import signal
import sys
import time
from typing import Callable

import py_trees
import py_trees_ros
import rclpy
from py_trees.behaviours import CheckBlackboardVariableValue
from py_trees.common import ComparisonExpression
from py_trees.decorators import EternalGuard, StatusToBlackboard
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor

from fault_detector_spot.application.behaviour_tree import (
    BaseGetGoalTag,
    BaseMoveRelativeAction,
    BaseMoveToTagAction,
    BufferStatusPublisher,
    CheckTagReachability,
    CloseGripperAction,
    CommandManager,
    CommandSubscriber,
    DetectVisibleTags,
    EnableLocalization,
    EnableSLAM,
    HandCameraTagDetection,
    HelperInitializer,
    LandmarkRelocalizer,
    ManipulatorGetGoalTag,
    ManipulatorMoveArmAction,
    ManipulatorMoveCloseToSurfaceAction,
    ManipulatorMoveRelativeAction,
    NavigateToGoalPose,
    NewCommandGuard,
    PublishLiveInspectionObject,
    PublishTagStates,
    PublishZeroVel,
    ReadyArmActionSimple,
    ResetEstopFlag,
    ResolveLiveInspectionObject,
    SetWaypointAsGoal,
    StandUpActionSimple,
    StopMapping,
    StowArmActionSimple,
    SwapMap,
    ToggleGripperAction,
    VisibleTagToMap,
    WaitForDuration,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.navigation.behaviours.last_localization_pose import (
    LastLocalizationPose,
)


helper_initializer: HelperInitializer = None
stop_mapping_behavior = None
stop_mapping_tree = None


def read_parameter(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def start_tree_ticking(tree, period_ms=50.0):
    tree.timer = tree.node.create_timer(
        period_ms / 1000.0,
        tree.tick,
        clock=Clock(clock_type=ClockType.STEADY_TIME),
    )
    return tree.timer


def create_root(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    return create_behavior_tree(node)


def create_behavior_tree(node: rclpy.node.Node):
    root = py_trees.composites.Parallel(
        "FaultDetectorSpot",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    root.add_children([
        build_sensing_tree(node),
        build_buffered_command_tree(node),
        build_publisher_tree(node),
    ])
    return root


def build_sensing_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    base_frame = read_parameter(
        node,
        "tag_sensing.base_frame",
        "body",
    )
    base_frame_pattern = read_parameter(
        node,
        "tag_sensing.base_frame_pattern",
        r"(?<!filtered_)fiducial_(\d+)",
    )
    hand_tag_frame_prefix = read_parameter(
        node,
        "tag_sensing.hand_tag_frame_prefix",
        "tag36h11:",
    )
    hand_detection_topic = read_parameter(
        node,
        "tag_sensing.hand_detection_topic",
        "/detections",
    )
    base_max_age_sec = float(
        read_parameter(
            node,
            "tag_sensing.base_max_age_sec",
            1.5,
        )
    )
    hand_max_age_sec = float(
        read_parameter(
            node,
            "tag_sensing.hand_max_age_sec",
            1.0,
        )
    )
    hand_tf_pending_sec = float(
        read_parameter(
            node,
            "tag_sensing.hand_tf_pending_sec",
            0.5,
        )
    )
    hand_max_hamming = int(
        read_parameter(
            node,
            "tag_sensing.hand_max_hamming",
            0,
        )
    )
    hand_min_decision_margin = float(
        read_parameter(
            node,
            "tag_sensing.hand_min_decision_margin",
            0.0,
        )
    )
    probe_max_age_sec = float(
        read_parameter(
            node,
            "tag_sensing.probe_max_age_sec",
            1.5,
        )
    )
    active_object_id = read_parameter(
        node,
        "inspection.active_object_id",
        "",
    )
    active_routine_id = read_parameter(
        node,
        "inspection.active_routine_id",
        "",
    )
    inspection_object_root = read_parameter(
        node,
        "inspection.object_root",
        "",
    )
    inspection_execution_frame = read_parameter(
        node,
        "inspection.execution_frame",
        "odom",
    )

    sensing_seq = py_trees.composites.Parallel(
        "Sensing",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    cmd_sub = CommandSubscriber(name="UI Command Listener")
    pose_sub = LastLocalizationPose(name="LastLocalizationPose")
    tag_scan_sequence = py_trees.composites.Sequence(
        name="ScanForTags",
        memory=True,
    )
    detect = DetectVisibleTags(
        name="DetectBaseTags",
        frame_pattern=base_frame_pattern,
        target_frame=base_frame,
        max_age_sec=base_max_age_sec,
    )
    hand_detect = HandCameraTagDetection(
        name="DetectHandTags",
        detection_topic=hand_detection_topic,
        target_frame=base_frame,
        tag_frame_prefix=hand_tag_frame_prefix,
        max_age_sec=hand_max_age_sec,
        tf_pending_sec=hand_tf_pending_sec,
        max_hamming=hand_max_hamming,
        min_decision_margin=hand_min_decision_margin,
    )
    live_object_resolver = ResolveLiveInspectionObject(
        object_id=active_object_id,
        routine_id=active_routine_id,
        execution_frame=inspection_execution_frame,
        maximum_age_sec=probe_max_age_sec,
        object_root=inspection_object_root or None,
        name="ResolveLiveInspectionObject",
    )
    live_object_publisher = PublishLiveInspectionObject(
        name="PublishLiveInspectionObject",
    )
    in_range_checker = CheckTagReachability(
        name="CheckTagReachability"
    )
    tag_publisher = PublishTagStates(
        name="TagPublisher"
    )
    slam_helper = get_helper_container(node).slam_helper
    world_frame_transformer = VisibleTagToMap(
        slam_helper=slam_helper,
        name="VisibleTagToMap",
    )
    tag_scan_sequence.add_children([
        detect,
        hand_detect,
        live_object_resolver,
        live_object_publisher,
        in_range_checker,
        tag_publisher,
        world_frame_transformer,
    ])
    sensing_seq.add_children([
        tag_scan_sequence,
        cmd_sub,
        pose_sub,
    ])
    return sensing_seq


def build_buffered_command_tree(
    node: rclpy.node.Node,
) -> py_trees.behaviour.Behaviour:
    buffered_command_tree = py_trees.composites.Parallel(
        "BufferedCommandTree",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    command_tree = build_repeat_guarded_cancelable_command_tree(node)
    buffer = CommandManager(name="CommandManager")
    buffered_command_tree.add_children([
        buffer,
        command_tree,
    ])
    return buffered_command_tree


def get_helper_container(node: rclpy.node.Node):
    global helper_initializer
    if helper_initializer is None:
        helper_initializer = HelperInitializer(
            name="InitHelpers",
            node=node,
        )
        helper_initializer.setup(timeout=10)
        helper_initializer.tick_once()
    return helper_initializer


def build_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    command_selector = py_trees.composites.Selector(
        name="CommandSelector",
        memory=True,
    )
    slam_helper = get_helper_container(node).slam_helper
    specs = [
        (
            CommandID.STOW_ARM,
            lambda n: StowArmActionSimple(name="StowArmAction"),
        ),
        (
            CommandID.READY_ARM,
            lambda n: ReadyArmActionSimple(name="ReadyArmAction"),
        ),
        (
            CommandID.TOGGLE_GRIPPER,
            lambda n: ToggleGripperAction(name="ToggleGripperAction"),
        ),
        (CommandID.MOVE_ARM_TO_TAG, build_manipulator_goal_tree),
        (CommandID.MOVE_BASE_TO_TAG, build_base_goal_tree),
        (
            CommandID.MOVE_BASE_RELATIVE,
            lambda n: BaseMoveRelativeAction(
                name="BaseMoveRelativeAction"
            ),
        ),
        (
            CommandID.MOVE_ARM_RELATIVE,
            lambda n: ManipulatorMoveRelativeAction(
                name="MoveArmRelativeAction"
            ),
        ),
        (
            CommandID.MOVE_CLOSE_TO_SURFACE,
            lambda n: ManipulatorMoveCloseToSurfaceAction(
                name="MoveCloseToSurfaceAction"
            ),
        ),
        (
            CommandID.STAND_UP,
            lambda n: StandUpActionSimple(name="StandUpAction"),
        ),
        (
            CommandID.WAIT_TIME,
            lambda n: WaitForDuration(name="WaitForDuration"),
        ),
        (
            CommandID.CLOSE_GRIPPER,
            lambda n: CloseGripperAction(),
        ),
        (
            CommandID.STOP_BASE,
            lambda n: PublishZeroVel(),
        ),
        (
            CommandID.START_SLAM,
            lambda n: EnableSLAM(slam_helper),
        ),
        (
            CommandID.START_LOCALIZATION,
            lambda n: EnableLocalization(slam_helper),
        ),
        (
            CommandID.SWAP_MAP,
            lambda n: SwapMap(slam_helper),
        ),
        (
            CommandID.STOP_MAPPING,
            lambda n: StopMapping(slam_helper),
        ),
        (
            CommandID.MOVE_TO_WAYPOINT,
            build_navigate_to_goal_pose_tree,
        ),
    ]

    for command_id, constructor in specs:
        command_selector.add_child(
            make_simple_command_sequence(
                node,
                command_id,
                constructor,
            )
        )
    return command_selector


def build_cancelable_command_tree(
    node: rclpy.node.Node,
) -> py_trees.behaviour.Behaviour:
    cancel_check = match_command_checker(CommandID.EMERGENCY_CANCEL)
    stop_base = PublishZeroVel(name="StopBase")
    stop_mapping = StopMapping(
        get_helper_container(node).slam_helper,
        with_save=False,
        name="ESTOP MAPPING",
    )
    stow_cancel = StowArmActionSimple(name="StowArmCancel")
    close_gripper = CloseGripperAction(name="CloseGripperAction")
    reset_estop = ResetEstopFlag(name="ResetEStopFlag")

    cancel_seq = py_trees.composites.Sequence(
        "CancelSequence",
        memory=True,
    )
    cancel_seq.add_children([
        cancel_check,
        stop_base,
        stop_mapping,
        stow_cancel,
        close_gripper,
        reset_estop,
    ])
    normal_tree = build_command_tree(node)

    def not_emergency(blackboard):
        return not blackboard.estop_flag

    emergency_guard = EternalGuard(
        name="EmergencyGuard",
        child=normal_tree,
        condition=not_emergency,
        blackboard_keys={"estop_flag"},
    )
    root = py_trees.composites.Selector(
        "CancelableCommandSelector",
        memory=True,
    )
    root.add_children([
        cancel_seq,
        emergency_guard,
    ])
    return StatusToBlackboard(
        name="CommandTree→BB",
        child=root,
        variable_name="command_tree_status",
    )


def build_publisher_tree(
    node: rclpy.node.Node,
) -> py_trees.behaviour.Behaviour:
    cmd_pub = BufferStatusPublisher(name="CommandStatusPublisher")
    init_pose_pub = LandmarkRelocalizer(
        get_helper_container(node).slam_helper,
        name="InitPosePublisher",
    )
    publisher_tree = py_trees.composites.Parallel(
        "PublisherTree",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    publisher_tree.add_children([
        cmd_pub,
        init_pose_pub,
    ])
    return publisher_tree


def build_repeat_guarded_cancelable_command_tree(
    node: rclpy.node.Node,
) -> py_trees.behaviour.Behaviour:
    guard = NewCommandGuard(name="NewCommandGuard")
    guarded_sequence = py_trees.composites.Sequence(
        name="GuardedCommands",
        memory=True,
    )
    cancelable_command_tree = build_cancelable_command_tree(node)
    guarded_sequence.add_children([
        guard,
        cancelable_command_tree,
    ])
    return guarded_sequence


def make_simple_command_sequence(
    node: rclpy.node.Node,
    command_id: CommandID,
    behaviour_ctor: Callable[
        [rclpy.node.Node],
        py_trees.behaviour.Behaviour,
    ],
) -> py_trees.composites.Sequence:
    seq_name = f"{command_id.name.title().replace('_', '')}Sequence"
    seq = py_trees.composites.Sequence(
        seq_name,
        memory=True,
    )
    check = CheckBlackboardVariableValue(
        name=f"Check_{command_id.name}",
        check=ComparisonExpression(
            variable="last_command",
            value=command_id,
            operator=lambda command, candidate: (
                command is not None
                and command.command_id == candidate
            ),
        ),
    )
    child = behaviour_ctor(node)
    seq.add_children([
        check,
        child,
    ])
    return seq


def match_command_checker(
    command_id: int,
) -> CheckBlackboardVariableValue:
    return CheckBlackboardVariableValue(
        name=f"Check_{command_id}",
        check=ComparisonExpression(
            variable="last_command",
            value=command_id,
            operator=lambda command, candidate: (
                command is not None
                and command.command_id == candidate
            ),
        ),
    )


def build_manipulator_goal_tree(
    node: rclpy.node.Node,
) -> py_trees.behaviour.Behaviour:
    manipulation = py_trees.composites.Sequence(
        "ManipulationSequence",
        memory=True,
    )
    get_goal = ManipulatorGetGoalTag(
        name="GetGoalTagPosition"
    )
    move_arm = ManipulatorMoveArmAction(
        name="MoveArm"
    )
    manipulation.add_children([
        get_goal,
        move_arm,
    ])
    return manipulation


def build_base_goal_tree(
    node: rclpy.node.Node,
) -> py_trees.behaviour.Behaviour:
    base_sequence = py_trees.composites.Sequence(
        "BaseMoveToTagSequence",
        memory=True,
    )
    get_goal = BaseGetGoalTag(
        name="BaseGetGoalTag"
    )
    move_base = BaseMoveToTagAction(
        name="BaseMoveToTagAction"
    )
    base_sequence.add_children([
        get_goal,
        move_base,
    ])
    return base_sequence


def build_navigate_to_goal_pose_tree(
    node: rclpy.node.Node,
) -> py_trees.behaviour.Behaviour:
    sequence = py_trees.composites.Sequence(
        "NavigateToWaypoint",
        memory=True,
    )
    set_goal = SetWaypointAsGoal(
        name="SetWaypointAsGoal"
    )
    navigate = NavigateToGoalPose(
        name="NavigateToGoalPose"
    )
    sequence.add_children([
        set_goal,
        navigate,
    ])
    return sequence


def ctrl_c_handler(sig, frame):
    global stop_mapping_behavior, stop_mapping_tree

    if stop_mapping_behavior is None or stop_mapping_tree is None:
        return

    node = stop_mapping_behavior.helper.node
    node.get_logger().info(
        "Ctrl-C received, running StopMapping behaviour..."
    )
    while (
        stop_mapping_behavior.update()
        != py_trees.common.Status.SUCCESS
    ):
        time.sleep(0.1)
    node.get_logger().info(
        "StopMapping completed, exiting..."
    )


def main(args=None):
    global stop_mapping_behavior, stop_mapping_tree

    rclpy.init(args=args)
    node = rclpy.create_node("bt_driver")
    root = create_root(node)
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=False,
    )

    try:
        tree.setup(
            node=node,
            timeout=15.0,
        )
    except py_trees_ros.exceptions.TimedOutError as exception:
        node.get_logger().error(
            f"Behavior tree setup failed: {exception}"
        )
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    start_tree_ticking(tree)

    slam_helper = get_helper_container(tree.node).slam_helper
    stop_mapping_behavior = StopMapping(
        slam_helper,
        with_save=False,
    )
    stop_mapping_behavior.setup(node=tree.node)
    stop_mapping_tree = tree
    signal.signal(signal.SIGINT, ctrl_c_handler)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(tree.node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()

    tree.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
