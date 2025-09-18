import typing

import py_trees
import tf2_ros
from fault_detector_msgs.msg import TagElement
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration


class VisibleTagToMap(py_trees.behaviour.Behaviour):
    def __init__(self, slam_helper, name="VisibleTagToMap"):
        super().__init__(name)
        self.slam_helper = slam_helper
        self.node = None
        self.blackboard = self.attach_blackboard_client()
        self.tags_in_map = set()
        self.last_published_pose = None

    def setup(self, **kwargs: typing.Any) -> None:
        self.node = kwargs['node']
        self.blackboard.register_key("visible_tags", access=py_trees.common.Access.READ)
        self.blackboard.register_key("visible_tags_map_frame", access=py_trees.common.Access.WRITE)

        # Initialize empty dict to avoid KeyError
        self.blackboard.visible_tags_map_frame = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def update(self):
        if not self.slam_helper.nav2_helper.is_running():
            self.feedback_message = "Localization not running"
            return py_trees.common.Status.SUCCESS

        map_name = self.slam_helper.bb.active_map_name
        if not map_name:
            self.feedback_message = "No active map"
            return py_trees.common.Status.SUCCESS

        if not self.blackboard.exists("visible_tags") or not self.blackboard.visible_tags:
            self.feedback_message = "No visible tags"
            return py_trees.common.Status.SUCCESS

        tags_in_map = self._get_visible_tags_in_map_frame()
        if not tags_in_map:
            return py_trees.common.Status.SUCCESS
        self.blackboard.visible_tags_map_frame = tags_in_map
        self.feedback_message = f"Transformed {len(tags_in_map)} tags to map frame"
        return py_trees.common.Status.SUCCESS

    def _get_visible_tags_in_map_frame(self):
        transformed_tags = {}
        for tag_id, tag_element in self.blackboard.visible_tags.items():
            pose_stamped = PoseStamped()
            pose_stamped.header = tag_element.pose.header
            pose_stamped.pose = tag_element.pose.pose

            # Check if transform to map is available
            if self.tf_buffer.can_transform(
                    "map",
                    pose_stamped.header.frame_id,
                    pose_stamped.header.stamp,
                    timeout=Duration(seconds=0.2),
            ):
                try:
                    pose_in_map = self.tf_buffer.transform(
                        pose_stamped,
                        "map",
                        timeout=Duration(seconds=0.2)
                    )

                    # Copy into a new TagElement
                    tag_in_map = TagElement()
                    tag_in_map.id = tag_id
                    tag_in_map.pose.header = pose_in_map.header
                    tag_in_map.pose.pose = pose_in_map.pose

                    transformed_tags[tag_id] = tag_in_map

                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as e:
                    self.logger.debug(f"Failed to transform tag {tag_id}: {e}")
            else:
                self.logger.debug(f"No transform available from {pose_stamped.header.frame_id} -> map for tag {tag_id}")

        return transformed_tags