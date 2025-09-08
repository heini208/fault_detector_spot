# In fault_detector_spot/behaviour_tree/bt_runner.py
import os
import signal
import time
from typing import Callable

import rclpy
import py_trees
import py_trees_ros
import sys

from fault_detector_msgs.msg import StringArray
from fault_detector_spot.behaviour_tree.nodes.mapping.monitor_slam_process import MonitorSLAMProcess
from fault_detector_spot.behaviour_tree.nodes.utility.publish_initial_ui_info_once import PublishInitialUIInfoOnce
from fault_detector_spot.behaviour_tree.nodes.utility.run_once import RunOnce
from py_trees.behaviours import CheckBlackboardVariableValue
from py_trees.common import ComparisonExpression
from py_trees.decorators import StatusToBlackboard, FailureIsSuccess, EternalGuard
from fault_detector_spot.behaviour_tree.commands.command_ids import CommandID

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
    ToggleGripperAction, CloseGripperAction, EnableSLAM,
    DeleteMap,
    InitializeEmptyMap, SwapMap, EnableLocalization, StopMapping, AddGoalPoseAsWaypoint, SaveCurrentPoseAsGoal,
    DeleteWaypoint, SetWaypointAsGoal, NavigateToGoalPose)


def create_root() -> py_trees.behaviour.Behaviour:
    root = create_behavior_tree()
    return root


def create_behavior_tree():
    node = rclpy.create_node("bt_driver")

    root = py_trees.composites.Parallel(
        "FaultDetectorSpot",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )

    root.add_children([
        build_sensing_tree(node),
        build_buffered_command_tree(node),
        build_publisher_tree(node),
    ])
    return root


def build_sensing_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    sensing_seq = py_trees.composites.Parallel("Sensing", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    cmd_sub = CommandSubscriber(name="UI Command Listener")
    cmd_sub.setup(node=node)

    tag_scan_sequence = py_trees.composites.Sequence(
        name="ScanForTags",
        memory=True
    )

    detect = DetectVisibleTags(name="Detect Tags", frame_pattern=r"filtered_fiducial_(\d+)")
    detect.setup(node=node)

    # hand_detect = HandCameraTagDetection(name="HandCameraTagDetection", hand_camera_topic="/Spot/right_head_depth/image", hand_camera_info_topic="/Spot/right_head_depth/camera_info", target_frame="base_link", source_frame="right head depth")
    # detect.setup(node=node)

    in_range_checker = CheckTagReachability(name="CheckTagReachability")
    detect.setup(node=node)

    tag_publisher = PublishTagStates(name="TagPublisher")
    detect.setup(node=node)

    tag_scan_sequence.add_children([detect, in_range_checker, tag_publisher])

    sensing_seq.add_children([tag_scan_sequence, cmd_sub])
    return sensing_seq


def build_buffered_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    buffered_command_tree = py_trees.composites.Parallel(
        "BufferedCommandTree",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    command_tree = build_repeat_guarded_cancelable_command_tree(node)

    buffer = CommandManager(name="CommandManager")
    buffer.setup()
    buffered_command_tree.add_children([
        buffer,
        command_tree
    ])
    return buffered_command_tree


def build_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    command_selector = py_trees.composites.Selector(
        name="CommandSelector",
        memory=True
    )

    specs = [
        (CommandID.STOW_ARM, lambda n: StowArmActionSimple(name="StowArmAction")),
        (CommandID.READY_ARM, lambda n: ReadyArmActionSimple(name="ReadyArmAction")),
        (CommandID.TOGGLE_GRIPPER, lambda n: ToggleGripperAction(name="ToggleGripperAction")),
        (CommandID.MOVE_ARM_TO_TAG, build_manipulator_goal_tree),
        (CommandID.MOVE_ARM_RELATIVE, lambda n: ManipulatorMoveRelativeAction(name="MoveArmRelativeAction")),
        (CommandID.STAND_UP, lambda n: StandUpActionSimple(name="StandUpAction")),
        (CommandID.WAIT_TIME, lambda n: WaitForDuration(name="WaitForDuration")),
        (CommandID.CLOSE_GRIPPER, lambda n: CloseGripperAction()),
        (CommandID.STOP_BASE, lambda n: PublishZeroVel()),
        (CommandID.START_SLAM, lambda n: EnableSLAM(launch_file="slam_sim_merged_launch.py")),
        (CommandID.START_LOCALIZATION, lambda n: EnableLocalization(launch_file="localization_sim_merged_launch.py")),
        (CommandID.CREATE_MAP, lambda n: InitializeEmptyMap(launch_file="slam_sim_merged_launch.py")),
        (CommandID.DELETE_MAP, lambda n: DeleteMap()),
        (CommandID.SWAP_MAP, lambda n: SwapMap(slam_launch="slam_sim_merged_launch.py")),
        (CommandID.STOP_MAPPING, lambda n: StopMapping()),
        (CommandID.ADD_CURRENT_POSITION_WAYPOINT, build_current_pose_as_landmark_tree),
        (CommandID.DELETE_WAYPOINT, lambda n: DeleteWaypoint()),
        (CommandID.MOVE_TO_WAYPOINT, build_navigate_to_goal_pose_tree),
    ]

    for cmd_id, ctor in specs:
        command_selector.add_child(make_simple_command_sequence(node, cmd_id, ctor))

    return command_selector


def build_cancelable_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    cancel_check = match_command_checker(CommandID.EMERGENCY_CANCEL)

    stop_base = PublishZeroVel(name="StopBase")
    stow_cancel = StowArmActionSimple(name="StowArmCancel")
    close_gripper = CloseGripperAction(name="CloseGripperAction")
    reset_estop = ResetEstopFlag(name="ResetEStopFlag")
    stop_base.setup(node=node)
    stow_cancel.setup(node=node)
    close_gripper.setup(node=node)
    reset_estop.setup(node=node)

    cancel_seq = py_trees.composites.Sequence("CancelSequence", memory=True)
    cancel_seq.add_children([cancel_check, stop_base, stow_cancel, close_gripper, reset_estop])

    normal_tree = build_command_tree(node)

    def not_emergency(blackboard):
        return not blackboard.estop_flag

    emergency_guard = EternalGuard(
        name="EmergencyGuard",
        child=normal_tree,
        condition=not_emergency,
        blackboard_keys={"estop_flag"}  # now triggers as soon as estop_flag==True
    )

    root = py_trees.composites.Selector("CancelableCommandSelector", memory=True)
    root.add_children([cancel_seq, emergency_guard])

    # Wrap in StatusToBlackboard to get a flag on the blackboard status
    command_tree_with_flag = StatusToBlackboard(
        name="CommandTree→BB",
        child=root,
        variable_name="command_tree_status"
    )
    return command_tree_with_flag


def build_publisher_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    initial_ui_info = PublishInitialUIInfoOnce(name="PublishInitialUIInfoOnce")
    initial_ui_info.setup(node=node)

    # Other publishers
    cmd_pub = BufferStatusPublisher(name="CommandStatusPublisher")
    cmd_pub.setup(node=node)

    # Parallel so both can exist simultaneously
    publisher_tree = py_trees.composites.Parallel(
        "PublisherTree",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    publisher_tree.add_children([initial_ui_info, cmd_pub])

    return publisher_tree


def build_repeat_guarded_cancelable_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    guard = NewCommandGuard(name="NewCommandGuard")
    guard.setup(node=node)
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
    if hasattr(child, "setup"):
        child.setup(node=node)
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
    get_goal.setup(node=node)

    move_arm = ManipulatorMoveArmAction(name="MoveArm")
    move_arm.setup(node=node)

    manipulation.add_children([get_goal, move_arm])

    return manipulation


def build_current_pose_as_landmark_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    sequence = py_trees.composites.Sequence("SaveCurrentPoseAsLandmark", memory=True)
    get_goal = SaveCurrentPoseAsGoal(name="SaveCurrentPoseAsGoal")
    get_goal.setup(node=node)

    add_waypoint = AddGoalPoseAsWaypoint(name="AddGoalPoseAsWaypoint")
    add_waypoint.setup(node=node)
    sequence.add_children([get_goal, add_waypoint])
    return sequence


def build_navigate_to_goal_pose_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    sequence = py_trees.composites.Sequence("NavigateToWaypoint", memory=True)
    set_goal = SetWaypointAsGoal(name="SetWaypointAsGoal")
    set_goal.setup(node=node)

    navigate = NavigateToGoalPose(name="NavigateToGoalPose")
    navigate.setup(node=node)
    sequence.add_children([set_goal, navigate])

    return sequence


def init_ui(self):
    """
    Initialize the UI with default states and publish the initial map list.
    This should be called once after ROS communication is set up.
    """
    # Update map dropdown locally (in case map_list topic hasn't published yet)
    self.update_map_dropdown()

    # Set current map to None initially
    self.current_map = None
    self.current_map_label.setText("Current Map: None")
    self.mode_none.setChecked(True)

    # Publish initial map list to ROS so other nodes/UI can subscribe
    map_files = []
    if os.path.isdir(self.recordings_dir):
        for f in sorted(os.listdir(self.recordings_dir)):
            if f.endswith(".db"):
                map_files.append(f[:-3])

    msg = StringArray()
    msg.names = map_files
    self.complex_command_publisher.publish(msg)


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
    rclpy.init(args=args)

    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    tree.setup(timeout=15.0)
    tree.tick_tock(period_ms=50.0)

    # Create StopMapping behaviour instance
    global stop_mapping_behavior, stop_mapping_tree
    stop_mapping_behavior = StopMapping(with_save=True)
    stop_mapping_behavior.setup(node=tree.node)
    stop_mapping_tree = tree

    # Register Ctrl-C handler
    signal.signal(signal.SIGINT, ctrl_c_handler)

    # Normal spinning
    try:
        rclpy.spin(tree.node)
    finally:
        tree.shutdown()
        rclpy.shutdown()



if __name__ == "__main__":
    main()
