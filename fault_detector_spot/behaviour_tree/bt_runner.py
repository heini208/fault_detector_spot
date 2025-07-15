# In fault_detector_spot/behaviour_tree/bt_runner.py

import rclpy
import py_trees
import py_trees_ros
import sys
from rclpy.node import Node

from fault_detector_spot.behaviour_tree.nodes.detect_visible_tags import DetectVisibleTags

def create_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(
        name="FaultDetectorSpot",
        memory=True,
        children=[
            # In your bt_runner.py:
            DetectVisibleTags(name="Detect Tags", frame_pattern=r"filtered_fiducial_(\d+)"),
            py_trees.behaviours.Success(name="Placeholder Action")
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