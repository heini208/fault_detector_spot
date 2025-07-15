# In fault_detector_spot/behaviour_tree/bt_runner.py

import rclpy
import py_trees
import py_trees_ros
import sys
from rclpy.node import Node

from fault_detector_spot.behaviour_tree.nodes.detect_visible_tags import DetectVisibleTags
from fault_detector_spot.behaviour_tree.nodes.manipulator_get_goal_tag import ManipulatorGetGoalTag
from fault_detector_spot.behaviour_tree.nodes.manipulator_move_arm_action import ManipulatorMoveArmAction
from fault_detector_spot.behaviour_tree.nodes.manipulator_command_subscriber import ManipulatorCommandSubscriber
def create_root() -> py_trees.behaviour.Behaviour:
    root = create_behavior_tree()
    return root

def create_behavior_tree():
    root = py_trees.composites.Parallel(
        name="FaultDetectorSpot",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[
            # Detect tags and listen for UI commands in parallel
            py_trees.composites.Parallel(
                name="Sensing",
                policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
                children=[
                    DetectVisibleTags(name="Detect Tags", frame_pattern=r"filtered_fiducial_(\d+)"),
                    ManipulatorCommandSubscriber(name="UI Command Listener"),
                ]
            ),

            # Execute manipulation sequence when tags and commands are available
            py_trees.composites.Sequence(
                name="Manipulation",
                memory=True,
                children=[
                    # Your manipulation nodes here
                    ManipulatorGetGoalTag(name="Get Goal Tag"),
                    ManipulatorMoveArmAction(name="Move Arm")
                ]
            )
        ]
    )
    return root


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