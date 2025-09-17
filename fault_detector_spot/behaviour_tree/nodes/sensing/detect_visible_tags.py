import re
from typing import Dict, Optional

import py_trees
import rclpy
import tf2_ros
from fault_detector_msgs.msg import TagElement


class DetectVisibleTags(py_trees.behaviour.Behaviour):
    """
    Monitors the TF tree for AprilTag frames, updates the blackboard with
    visible tags and their poses.
    """

    def __init__(self, name: str = "DetectVisibleTags", frame_pattern: str = r"tag_(\d+)"):
        super(DetectVisibleTags, self).__init__(name)
        self.node: Optional[rclpy.node.Node] = None
        self.tf_buffer: Optional[tf2_ros.Buffer] = None
        self.tf_listener: Optional[tf2_ros.TransformListener] = None
        self.blackboard = self.attach_blackboard_client()
        self.frame_pattern = re.compile(frame_pattern)

    def setup(self, **kwargs):
        """
        Initialise ROS-specific components: the node, TF listener.
        """
        try:
            self.node = kwargs['node']
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
            self.blackboard.register_key(
                key="visible_tags", access=py_trees.common.Access.WRITE
            )
            self.logger.info("DetectVisibleTags node initialized.")
        except KeyError as e:
            self.logger.error(f"Could not retrieve node from kwargs: {e}")

    def update(self) -> py_trees.common.Status:
        """
        Periodically called to check for tags, update the blackboard.
        This behaviour always returns SUCCESS.
        """
        visible_tags = self._get_visible_tags_from_tf()

        self.blackboard.visible_tags = visible_tags
        self.feedback_message = f"Visible tags: {sorted(visible_tags.keys())}"
        return py_trees.common.Status.SUCCESS

    def _get_visible_tags_from_tf(self) -> Dict[int, TagElement]:
        """
        Scans the TF tree for frames matching the tag pattern and returns them.
        """
        if not self.tf_buffer:
            return {}

        visible_tags_dict = {}

        try:
            yaml_str = self.tf_buffer.all_frames_as_yaml()
        except Exception as e:
            self.logger.warn(f"Couldn't list TF frames: {e}")
            return {}

        # Process each line of the YAML string
        for line in yaml_str.splitlines():
            # Look for tag frame patterns in the line
            matches = self.frame_pattern.finditer(line)
            for match in matches:
                frame_name = match.group(0)  # Complete frame name
                tag_id = int(match.group(1))  # Just the ID number

                try:
                    # Get transform from robot base to tag
                    transform = self.tf_buffer.lookup_transform(
                        'body', frame_name, rclpy.time.Time()
                    )
                    tag_element = self._create_tag_element(tag_id, transform)
                    visible_tags_dict[tag_id] = tag_element
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.logger.debug(f"Could not get transform for {frame_name}: {e}")

        return visible_tags_dict

    @staticmethod
    def _create_tag_element(tag_id: int, transform) -> TagElement:
        """Creates a TagElement message from a given transform."""
        tag_element = TagElement()
        tag_element.id = tag_id
        tag_element.pose.header = transform.header
        tag_element.pose.pose.position.x = transform.transform.translation.x
        tag_element.pose.pose.position.y = transform.transform.translation.y
        tag_element.pose.pose.position.z = transform.transform.translation.z
        tag_element.pose.pose.orientation = transform.transform.rotation
        return tag_element