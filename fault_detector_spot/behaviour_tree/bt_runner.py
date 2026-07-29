# In fault_detector_spot/behaviour_tree/bt_runner.py
import signal
import sys
import time
from typing import Callable

import py_trees
import py_trees_ros
import rclpy
from fault_detector_spot.behaviour_tree import (
    DetectVisibleTags,
    HandCameraTagDetection,
    PublishTagStates,
    ManipulatorGetGoalTag,
    ManipulatorMoveArmAction,
    CommandSubscriber,
    NewCommandGuard,
    StowArmActionSimple,
    StandUpActionSimple,
    ReadyArmActionSimple,
    CheckTagReachability,
    PublishZeroVel,
    CommandManager,
    ResetEstopFlag,
    WaitForDuration,
    BufferStatusPublisher,
    ManipulatorMoveRelativeAction,
    ToggleGripperAction, CloseGripperAction, HelperInitializer, DeleteWaypoint, StopMapping, SwapMap, DeleteMap,
    InitializeEmptyMap, EnableLocalization, EnableSLAM, SaveCurrentPoseAsGoal, AddGoalPoseAsWaypoint, SetWaypointAsGoal,
    NavigateToGoalPose, SetTagAsGoal, AddGoalPoseAsLandmark, VisibleTagToMap, LandmarkRelocalizer, DeleteLandmark,
    BaseGetGoalTag, BaseMoveToTagAction, BaseMoveRelativeAction,
    CaptureInspectionObjectReferenceView, CreateInspectionDefinition,
    DeleteInspectionDefinition,
    ResolveLiveInspectionObject, PublishLiveInspectionObject,
)
from fault_detector_spot.behaviour_tree.commands.command_ids import CommandID
from fault_detector_spot.behaviour_tree.nodes.sensing.last_localization_pose import LastLocalizationPose
from fault_detector_spot.behaviour_tree.nodes.utility.publish_initial_ui_info_once import PublishInitialUIInfoOnce
from py_trees.behaviours import CheckBlackboardVariableValue
from py_trees.common import ComparisonExpression
from py_trees.decorators import StatusToBlackboard, EternalGuard

helper_initializer: HelperInitializer = None

def read_parameter(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)

    return node.get_parameter(name).value

def create_root(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    return create_behavior_tree(node)


def create_behavior_tree(node: rclpy.node.Node):

    root = py_trees.composites.Parallel(
        "FaultDetectorSpot",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )

    root.add_children([
        build_sensing_tree(node),
        build_buffered_command_tree(node),
        build_publisher_tree(node)
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
        r"filtered_fiducial_(\d+)",
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
            0.25,
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

    sensing_seq = py_trees.composites.Parallel("Sensing", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    cmd_sub = CommandSubscriber(name="UI Command Listener")

    pose_sub = LastLocalizationPose(name="LastLocalizationPose")

    tag_scan_sequence = py_trees.composites.Sequence(
        name="ScanForTags",
        memory=True
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

    slam_helper = get_helper_container(
        node
    ).slam_helper

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

    sensing_seq.add_children([tag_scan_sequence, cmd_sub, pose_sub])
    return sensing_seq


def build_buffered_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    buffered_command_tree = py_trees.composites.Parallel(
        "BufferedCommandTree",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    command_tree = build_repeat_guarded_cancelable_command_tree(node)

    buffer = CommandManager(name="CommandManager")
    buffered_command_tree.add_children([
        buffer,
        command_tree
    ])
    return buffered_command_tree


def get_helper_container(node: rclpy.node.Node):
    global helper_initializer
    if helper_initializer is None:
        helper_initializer = HelperInitializer(name="InitHelpers", node=node, use_simulation=False)
        helper_initializer.setup(timeout=10)
        helper_initializer.tick_once()
    return helper_initializer


def build_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    command_selector = py_trees.composites.Selector(
        name="CommandSelector",
        memory=True
    )

    helper_initializer = get_helper_container(node)
    slam_helper = helper_initializer.slam_helper

    specs = [
        (CommandID.STOW_ARM, lambda n: StowArmActionSimple(name="StowArmAction")),
        (CommandID.READY_ARM, lambda n: ReadyArmActionSimple(name="ReadyArmAction")),
        (CommandID.TOGGLE_GRIPPER, lambda n: ToggleGripperAction(name="ToggleGripperAction")),
        (CommandID.MOVE_ARM_TO_TAG, build_manipulator_goal_tree),
        (CommandID.MOVE_BASE_TO_TAG, build_base_goal_tree),
        (CommandID.MOVE_BASE_RELATIVE, lambda n: BaseMoveRelativeAction(name="BaseMoveRelativeAction")),
        (CommandID.MOVE_ARM_RELATIVE, lambda n: ManipulatorMoveRelativeAction(name="MoveArmRelativeAction")),
        (CommandID.STAND_UP, lambda n: StandUpActionSimple(name="StandUpAction")),
        (CommandID.WAIT_TIME, lambda n: WaitForDuration(name="WaitForDuration")),
        (CommandID.CLOSE_GRIPPER, lambda n: CloseGripperAction()),
        (CommandID.STOP_BASE, lambda n: PublishZeroVel()),
        # Mapping commands
        (CommandID.START_SLAM, lambda n: EnableSLAM(slam_helper)),
        (CommandID.START_LOCALIZATION, lambda n: EnableLocalization(slam_helper)),
        (CommandID.CREATE_MAP, lambda n: InitializeEmptyMap(slam_helper)),
        (CommandID.DELETE_MAP, lambda n: DeleteMap()),
        (CommandID.SWAP_MAP, lambda n: SwapMap(slam_helper)),
        (CommandID.STOP_MAPPING, lambda n: StopMapping(slam_helper)),
        (CommandID.ADD_CURRENT_POSITION_WAYPOINT, build_current_pose_as_waypoint_tree),
        (CommandID.ADD_TAG_AS_LANDMARK, build_tag_pose_as_landmark_tree),
        (CommandID.DELETE_WAYPOINT, lambda n: DeleteWaypoint(slam_helper)),
        (CommandID.MOVE_TO_WAYPOINT, build_navigate_to_goal_pose_tree),
        (CommandID.DELETE_LANDMARK, lambda n: DeleteLandmark(slam_helper)),
        (
            CommandID.CREATE_INSPECTION_OBJECT,
            lambda n: build_inspection_creation_behavior(
                n,
                CommandID.CREATE_INSPECTION_OBJECT,
            ),
        ),
        (
            CommandID.CREATE_INSPECTION_ROUTINE,
            lambda n: build_inspection_creation_behavior(
                n,
                CommandID.CREATE_INSPECTION_ROUTINE,
            ),
        ),
        (
            CommandID.DELETE_INSPECTION_OBJECT,
            lambda n: build_inspection_deletion_behavior(
                n,
                CommandID.DELETE_INSPECTION_OBJECT,
            ),
        ),
        (
            CommandID.DELETE_INSPECTION_ROUTINE,
            lambda n: build_inspection_deletion_behavior(
                n,
                CommandID.DELETE_INSPECTION_ROUTINE,
            ),
        ),
        (
            CommandID.CAPTURE_INSPECTION_OBJECT_REFERENCE_VIEW,
            build_capture_reference_view_behavior,
        ),
    ]

    for cmd_id, ctor in specs:
        command_selector.add_child(make_simple_command_sequence(node, cmd_id, ctor))

    return command_selector


def build_inspection_creation_behavior(
    node: rclpy.node.Node,
    command_id: CommandID,
) -> py_trees.behaviour.Behaviour:
    """Build an explicit object or routine creation behavior."""
    object_root = read_parameter(
        node,
        "inspection.object_root",
        "",
    )
    return CreateInspectionDefinition(
        command_id=command_id,
        object_root=object_root or None,
    )


def build_inspection_deletion_behavior(
    node: rclpy.node.Node,
    command_id: CommandID,
) -> py_trees.behaviour.Behaviour:
    """Build an explicit object or routine deletion behavior."""
    object_root = read_parameter(
        node,
        "inspection.object_root",
        "",
    )
    return DeleteInspectionDefinition(
        command_id=command_id,
        object_root=object_root or None,
    )


def build_capture_reference_view_behavior(
    node: rclpy.node.Node,
) -> py_trees.behaviour.Behaviour:
    """Build the configured inspection reference-view capture."""
    object_root = read_parameter(
        node,
        "inspection.object_root",
        "",
    )
    return CaptureInspectionObjectReferenceView(
        rgb_topic=read_parameter(
            node,
            "inspection.reference_view_rgb_topic",
            "/camera/hand/image",
        ),
        depth_topic=read_parameter(
            node,
            "inspection.reference_view_depth_topic",
            "/depth_registered/hand/image",
        ),
        rgb_camera_info_topic=read_parameter(
            node,
            "inspection.reference_view_rgb_camera_info_topic",
            "/camera/hand/camera_info",
        ),
        depth_camera_info_topic=read_parameter(
            node,
            "inspection.reference_view_depth_camera_info_topic",
            "/depth_registered/hand/camera_info",
        ),
        base_tag_topic=read_parameter(
            node,
            "inspection.reference_view_base_tag_topic",
            "fault_detector/state/visible_tags",
        ),
        object_root=object_root or None,
        synchronization_queue_size=int(read_parameter(
            node,
            "inspection.reference_view_sync_queue_size",
            10,
        )),
        maximum_input_age_sec=float(read_parameter(
            node,
            "inspection.reference_view_maximum_input_age_sec",
            1.5,
        )),
        maximum_timestamp_skew_sec=float(read_parameter(
            node,
            "inspection.reference_view_maximum_timestamp_skew_sec",
            0.05,
        )),
        collection_duration_sec=float(read_parameter(
            node,
            "inspection.reference_view_collection_duration_sec",
            1.0,
        )),
        fixed_frame=read_parameter(
            node,
            "inspection.reference_view_fixed_frame",
            "odom",
        ),
        transform_timeout_sec=float(read_parameter(
            node,
            "inspection.reference_view_transform_timeout_sec",
            0.05,
        )),
        capture_timeout_sec=float(read_parameter(
            node,
            "inspection.reference_view_capture_timeout_sec",
            3.0,
        )),
        capture_max_attempts=int(read_parameter(
            node,
            "inspection.reference_view_capture_max_attempts",
            3,
        )),
    )


def build_cancelable_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    cancel_check = match_command_checker(CommandID.EMERGENCY_CANCEL)

    stop_base = PublishZeroVel(name="StopBase")
    stop_mapping = StopMapping(get_helper_container(node).slam_helper, with_save=False, name="ESTOP MAPPING")
    stow_cancel = StowArmActionSimple(name="StowArmCancel")
    close_gripper = CloseGripperAction(name="CloseGripperAction")
    reset_estop = ResetEstopFlag(name="ResetEStopFlag")

    cancel_seq = py_trees.composites.Sequence("CancelSequence", memory=True)
    cancel_seq.add_children([cancel_check, stop_base, stop_mapping, stow_cancel, close_gripper, reset_estop])

    normal_tree = build_command_tree(node)

    def not_emergency(blackboard):
        return not blackboard.estop_flag

    emergency_guard = EternalGuard(
        name="EmergencyGuard",
        child=normal_tree,
        condition=not_emergency,
        blackboard_keys={"estop_flag"}
    )

    root = py_trees.composites.Selector("CancelableCommandSelector", memory=True)
    root.add_children([cancel_seq, emergency_guard])

    command_tree_with_flag = StatusToBlackboard(
        name="CommandTree→BB",
        child=root,
        variable_name="command_tree_status"
    )
    return command_tree_with_flag


def build_publisher_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    initial_ui_info = PublishInitialUIInfoOnce(name="PublishInitialUIInfoOnce")

    # Other publishers
    cmd_pub = BufferStatusPublisher(name="CommandStatusPublisher")

    init_pose_pub = LandmarkRelocalizer(get_helper_container(node).slam_helper, name="InitPosePublisher")
    # Parallel so both can exist simultaneously
    publisher_tree = py_trees.composites.Parallel(
        "PublisherTree",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    publisher_tree.add_children([initial_ui_info, cmd_pub, init_pose_pub])

    return publisher_tree


def build_repeat_guarded_cancelable_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    guard = NewCommandGuard(name="NewCommandGuard")
    guarded_sequence = py_trees.composites.Sequence(
        name="GuardedCommands",
        memory=True
    )

    cancelable_command_tree = build_cancelable_command_tree(node)
    guarded_sequence.add_children([guard, cancelable_command_tree])
    return guarded_sequence


def make_simple_command_sequence(
        node: rclpy.node.Node,
        command_id: CommandID,
        behaviour_ctor: Callable[[rclpy.node.Node], py_trees.behaviour.Behaviour]
) -> py_trees.composites.Sequence:
    seq_name = f"{command_id.name.title().replace('_', '')}Sequence"
    seq = py_trees.composites.Sequence(seq_name, memory=True)

    check = CheckBlackboardVariableValue(
        name=f"Check_{command_id.name}",
        check=ComparisonExpression(
            variable="last_command",
            value=command_id,
            operator=lambda cmd, cid: cmd is not None and cmd.command_id == cid
        )
    )
    child = behaviour_ctor(node)
    seq.add_children([check, child])
    return seq


def match_command_checker(command_id: int) -> CheckBlackboardVariableValue:
    return CheckBlackboardVariableValue(
        name=f"Check_{command_id}",
        check=ComparisonExpression(
            variable="last_command",
            value=command_id,
            operator=lambda cmd, cid:
            (cmd is not None and cmd.command_id == cid)
        )
    )


def build_manipulator_goal_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    manipulation = py_trees.composites.Sequence("ManipulationSequence", memory=True)
    get_goal = ManipulatorGetGoalTag(name="GetGoalTagPosition")

    move_arm = ManipulatorMoveArmAction(name="MoveArm")

    manipulation.add_children([get_goal, move_arm])

    return manipulation


def build_base_goal_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    """
    Build the behaviour sequence to move the base relative to a visible tag.
    """
    base_sequence = py_trees.composites.Sequence("BaseMoveToTagSequence", memory=True)

    get_goal = BaseGetGoalTag(name="BaseGetGoalTag")
    move_base = BaseMoveToTagAction(name="BaseMoveToTagAction")

    base_sequence.add_children([get_goal, move_base])
    return base_sequence

def build_current_pose_as_waypoint_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    sequence = py_trees.composites.Sequence("SaveCurrentPoseAsLandmark", memory=True)
    get_goal = SaveCurrentPoseAsGoal(name="SaveCurrentPoseAsGoal")

    add_waypoint = AddGoalPoseAsWaypoint(get_helper_container(node).slam_helper, name="AddGoalPoseAsWaypoint")
    sequence.add_children([get_goal, add_waypoint])
    return sequence


def build_tag_pose_as_landmark_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    slam_helper = get_helper_container(node).slam_helper
    sequence = py_trees.composites.Sequence("SaveTagAsLandmark", memory=True)
    get_goal = SetTagAsGoal(name="SetTagAsGoal")

    add_waypoint = AddGoalPoseAsLandmark(slam_helper=slam_helper, name="AddGoalPoseAsLandmark")
    sequence.add_children([get_goal, add_waypoint])
    return sequence


def build_navigate_to_goal_pose_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    sequence = py_trees.composites.Sequence("NavigateToWaypoint", memory=True)
    set_goal = SetWaypointAsGoal(name="SetWaypointAsGoal")

    navigate = NavigateToGoalPose(name="NavigateToGoalPose")
    sequence.add_children([set_goal, navigate])

    return sequence


def ctrl_c_handler(sig, frame):
    """
    Trigger StopMapping behaviour until Slam Toolbox terminates.
    """
    global stop_mapping_behavior, stop_mapping_tree

    if stop_mapping_behavior is None or stop_mapping_tree is None:
        return

    node = stop_mapping_behavior.helper.node
    node.get_logger().info("Ctrl-C received, running StopMapping behaviour...")

    # Tick StopMapping until it returns SUCCESS
    while stop_mapping_behavior.update() != py_trees.common.Status.SUCCESS:
        time.sleep(0.1)  # 100ms between ticks

    node.get_logger().info("StopMapping completed, exiting...")
    # then allow main finally block to run


def main(args=None):
    global stop_mapping_behavior, stop_mapping_tree

    rclpy.init(args=args)
    node = rclpy.create_node("bt_driver")

    root = create_root(node)
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=False
    )

    try:
        tree.setup(
            node=node,
            timeout=15.0
        )
    except py_trees_ros.exceptions.TimedOutError as exception:
        node.get_logger().error(
            f"Behavior tree setup failed: {exception}"
        )
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)


    tree.tick_tock(period_ms=50.0)

    slam_helper = get_helper_container(tree.node).slam_helper
    stop_mapping_behavior = StopMapping(slam_helper, with_save=False)
    stop_mapping_behavior.setup(node=tree.node)
    stop_mapping_tree = tree

    # Register Ctrl-C handler
    signal.signal(signal.SIGINT, ctrl_c_handler)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
