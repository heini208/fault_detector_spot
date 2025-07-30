# fault_detector_spot/behaviour_tree/nodes/check_tag_reachability.py

import math
import py_trees
import rclpy.time
import tf2_ros
from fault_detector_msgs.msg import TagElement, TagElementArray
from typing import Dict, Optional

class CheckTagReachability(py_trees.behaviour.Behaviour):
    """
    Filters visible_tags by whether they lie within the robot arm's reach.
    Uses a static transform from 'body' -> 'arm_link_sh0' (looked up once in setup)
    """
    def __init__(self,
                 name: str = "CheckTagReachability",
                 body_frame: str = "body",
                 arm_base_frame: str = "arm_link_sh0",
                 arm_reach: float = 0.984,
                 tolerance: float = 0.15):
        super().__init__(name)
        self.body_frame       = body_frame
        self.arm_base_frame   = arm_base_frame
        self.maximum_reach = arm_reach - tolerance
        self.node: Optional[rclpy.node.Node] = None
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        self.arm_base_offset = None
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        if not self.node:
            raise RuntimeError(f"{self.name}: no ROS node provided")

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)


        self._check_transform_exists()
        # register output BB key
        self.blackboard.register_key(
            key="reachable_tags", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="visible_tags", access=py_trees.common.Access.READ
        )
        # initialize
        reachable: Dict[int, TagElement] = {}
        self.blackboard.reachable_tags = reachable
    def _check_transform_exists(self) -> bool:
        try:
            # target_frame='body', source_frame='arm_link_sh0'
            t = self.tf_buffer.lookup_transform(
                self.body_frame, self.arm_base_frame,
                rclpy.time.Time()
            )
            tr = t.transform.translation
            self.arm_base_offset = (tr.x, tr.y, tr.z)
            self.logger.info(f"{self.name}: got arm base offset {self.arm_base_offset}")
        except Exception as e:
            self.arm_base_offset = (0.0, 0.0, 0.0)

    def update(self) -> py_trees.common.Status:
        if self.arm_base_offset == (0.0, 0.0, 0.0):
            self._check_transform_exists()

        reachable: Dict[int, TagElement] = {}

        if self.blackboard.exists("visible_tags") and self.blackboard.visible_tags:
            visible = self.blackboard.visible_tags
        else:
            self.blackboard.reachable_tags = reachable
            self.feedback_message = f"No tags visible"
            return py_trees.common.Status.SUCCESS

        x_offset, y_offset, z_offset = self.arm_base_offset

        for tag_id, tag in visible.items():
            p = tag.pose.pose.position
            # compute vector from arm base to tag
            dx = p.x - x_offset
            dy = p.y - y_offset
            dz = p.z - z_offset
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dist <= self.maximum_reach:
                reachable[tag_id] = tag

        self.blackboard.reachable_tags = reachable
        self.feedback_message = (
            f"Reachable: {sorted(reachable.keys())} "
            f"(max {self.maximum_reach:.2f}m)"
        )
        self.feedback_message = f"Offset = {self.arm_base_offset}, Reachable tags: {sorted(self.blackboard.reachable_tags.keys())}"
        return py_trees.common.Status.SUCCESS


    def terminate(self, new_status: py_trees.common.Status):
        pass
