# In fault_detector_spot/behaviour_tree/bt_runner.py
from typing import Callable

import rclpy
import py_trees
import py_trees_ros
import sys

from py_trees.behaviours import CheckBlackboardVariableValue
from py_trees.common import ComparisonExpression
from py_trees.decorators import StatusToBlackboard, FailureIsSuccess, EternalGuard
from fault_detector_spot.behaviour_tree.command_ids import CommandID

from fault_detector_spot.behaviour_tree import (
    DetectVisibleTags,
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
)

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
        build_buffered_command_tree(node)
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

    in_range_checker = CheckTagReachability(name="CheckTagReachability")
    detect.setup(node=node)

    tag_scan_sequence.add_children([detect, in_range_checker])

    sensing_seq.add_children([tag_scan_sequence, cmd_sub])
    return sensing_seq

def build_buffered_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    buffered_command_tree = py_trees.composites.Parallel(
        "BufferedCommandTree",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    command_tree = build_repeat_guarded_cancelable_command_tree(node)
    # Wrap in StatusToBlackboard to get a flag on the blackboard status
    command_tree_with_flag = StatusToBlackboard(
        name="CommandTree→BB",
        child=command_tree,
        variable_name="command_tree_status"
    )

    buffer = CommandManager(name="CommandManager")
    buffer.setup()
    buffered_command_tree.add_children([
        buffer,
        command_tree_with_flag
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
        (CommandID.MOVE_TO_TAG, build_manipulator_goal_tree),
        (CommandID.STAND_UP, lambda n: StandUpActionSimple(name="StandUpAction")),
    ]

    for cmd_id, ctor in specs:
        command_selector.add_child(make_simple_command_sequence(node, cmd_id, ctor))

    return command_selector

def build_cancelable_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    cancel_check = match_command_checker(CommandID.EMERGENCY_CANCEL)

    stop_base    = PublishZeroVel(name="StopBase")
    stow_cancel  = StowArmActionSimple(name="StowArmCancel")
    stop_base.setup(node=node)
    stow_cancel.setup(node=node)

    cancel_seq = py_trees.composites.Sequence("CancelSequence", memory=True)
    cancel_seq.add_children([cancel_check, stop_base, stow_cancel])

    normal_tree = build_command_tree(node)

    def not_emergency(blackboard):
        cmd = getattr(blackboard, 'last_command_received', None)
        return not (cmd is not None and cmd.id == CommandID.EMERGENCY_CANCEL)

    emergency_guard = EternalGuard(
        name="EmergencyGuard",
        child=normal_tree,
        condition=not_emergency,
        blackboard_keys={"last_command_received"}  # watch this key
    )

    root = py_trees.composites.Selector("CancelableCommandSelector", memory=True)
    root.add_children([cancel_seq, emergency_guard])
    return root

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
            operator=lambda cmd, cid: cmd is not None and cmd.id == cid
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
            (cmd is not None and cmd.id == cid)
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


def main(args=None):
    rclpy.init(args=args)

    root = create_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )

    try:
        tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        print(f"Failed to setup the tree: {e}")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    # Tick the tree at 10 Hz
    tree.tick_tock(period_ms=100.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()