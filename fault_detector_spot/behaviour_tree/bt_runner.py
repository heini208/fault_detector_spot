# In fault_detector_spot/behaviour_tree/bt_runner.py

import rclpy
import py_trees
import py_trees_ros
import sys
import operator

from py_trees.behaviours import CheckBlackboardVariableValue
from rclpy.node import Node

from fault_detector_spot.behaviour_tree import (
    DetectVisibleTags,
    ManipulatorGetGoalTag,
    ManipulatorMoveArmAction,
    ManipulatorCommandSubscriber,
    NewCommandGuard,
    StowArmAction,
    StandUpAction,
    ReadyArmAction,
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
        build_command_tree(node)
    ])
    return root

def build_sensing_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    sensing_seq = py_trees.composites.Parallel("Sensing", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    detect = DetectVisibleTags(name="Detect Tags", frame_pattern=r"filtered_fiducial_(\d+)")
    detect.setup(node=node)

    cmd_sub = ManipulatorCommandSubscriber(name="UI Command Listener")
    cmd_sub.setup(node=node)


    sensing_seq.add_children([detect, cmd_sub])
    return sensing_seq

def build_command_tree(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:

    command_selector = py_trees.composites.Selector(
        name="CommandSelector",
        memory=True
    )

    stow_arm_sequence = stow_arm_command_sequence(node)
    command_selector.add_child(stow_arm_sequence)

    ready_arm_sequence = ready_arm_command_sequence(node)
    command_selector.add_child(ready_arm_sequence)

    move_to_tag_sequence = move_to_tag_command_sequence(node)
    command_selector.add_child(move_to_tag_sequence)

    stand_up_sequence = stand_up_command_sequence(node)
    command_selector.add_child(stand_up_sequence)

    guard = NewCommandGuard(name="NewCommandGuard")
    guard.setup(node=node)
    guarded_sequence = py_trees.composites.Sequence(
        name = "GuardedCommands",
        memory = True
    )

    guarded_sequence.add_children([guard, command_selector])
    return guarded_sequence

def move_to_tag_command_sequence(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    move_tag_seq = py_trees.composites.Sequence("MoveToTagSequence", memory=True)
    move_to_tag_behavior = build_manipulator_goal_tree(node)
    move_tag_check = match_command_checker("move_to_tag")
    move_tag_seq.add_children([move_tag_check, move_to_tag_behavior])
    return move_tag_seq

def stow_arm_command_sequence(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    stow_arm_seq = py_trees.composites.Sequence("StowArmSequence", memory=True)
    stow_arm_check = match_command_checker("stow_arm")
    stow_arm_action = StowArmAction(name="StowArmAction")
    stow_arm_action.setup(node=node)

    stow_arm_seq.add_children([stow_arm_check, stow_arm_action])
    return stow_arm_seq

def ready_arm_command_sequence(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    ready_arm_seq = py_trees.composites.Sequence("ReadyArmSequence", memory=True)
    ready_arm_check = match_command_checker("ready_arm")
    ready_arm_action = ReadyArmAction(name="ReadyArmAction")
    ready_arm_action.setup(node=node)

    ready_arm_seq.add_children([ready_arm_check, ready_arm_action])
    return ready_arm_seq

def stand_up_command_sequence(node: rclpy.node.Node) -> py_trees.behaviour.Behaviour:
    stand_up_seq = py_trees.composites.Sequence("StandUpSequence", memory=True)
    stand_up_check = match_command_checker("stand_up")
    stand_up_action = StandUpAction(name="StandUpAction")
    stand_up_action.setup(node=node)

    stand_up_seq.add_children([stand_up_check, stand_up_action])
    return stand_up_seq

def match_command_checker(command_id: int) -> CheckBlackboardVariableValue:
    return py_trees.behaviours.CheckBlackboardVariableValue(
    name=f"Check_{command_id}",
    check=py_trees.common.ComparisonExpression(
        variable="last_command_id",
        value=command_id,
        operator=operator.eq
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