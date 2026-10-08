"""One BT TF subscription preserves history and outlives its borrowers."""

from types import SimpleNamespace
from unittest.mock import Mock

import py_trees
import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import (
    RobotCommandResources,
)
from fault_detector_spot.inspection.behaviours.resolve_live_inspection_object import (
    ResolveLiveInspectionObject,
)
from fault_detector_spot.navigation.behaviours.landmark_relocalizer import (
    LandmarkRelocalizer,
)
from fault_detector_spot.sensing.behaviours.visible_tag_to_map import VisibleTagToMap


class Node:
    """Capture real listener callbacks without creating any ROS entities."""

    def __init__(self):
        self.subscriptions = []
        self.destroyed_topics = []

    def create_subscription(self, message_type, topic, callback, qos, **kwargs):
        subscription = SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        self.destroyed_topics.append(subscription.topic)

    def create_publisher(self, *args, **kwargs):
        return Mock()


def transform(stamp, x):
    message = TransformStamped()
    message.header.frame_id = "odom"
    message.child_frame_id = "body"
    message.header.stamp = Time(seconds=stamp).to_msg()
    message.transform.translation.x = float(x)
    message.transform.rotation.w = 1.0
    return message


def test_bt_consumers_share_history_without_owning_the_listener():
    py_trees.blackboard.Blackboard.clear()
    resources = RobotCommandResources()
    node = Node()
    try:
        surface = resources.get_probe_surface_source(node)
        listener = resources.get_tf_listener(node)
        buffer = listener.buffer
        runtime = SimpleNamespace(maps_dir="/unused")
        borrowers = [
            ResolveLiveInspectionObject("", "", tf_buffer=buffer),
            VisibleTagToMap(runtime, tf_buffer=buffer),
            LandmarkRelocalizer(runtime, tf_buffer=buffer, map_repository=Mock()),
        ]
        for borrower in borrowers:
            borrower.setup(node=node)

        tf_subscriptions = [
            subscription for subscription in node.subscriptions
            if subscription.topic in ("/tf", "/tf_static")
        ]
        assert sorted(subscription.topic for subscription in tf_subscriptions) == [
            "/tf", "/tf_static",
        ]
        assert surface._tf_buffer is buffer
        assert all(borrower.tf_buffer is buffer for borrower in borrowers)
        receive_tf = next(
            subscription.callback for subscription in tf_subscriptions
            if subscription.topic == "/tf"
        )
        for stamp, x in ((10.0, 1.0), (11.0, 3.0)):
            receive_tf(TFMessage(transforms=[transform(stamp, x)]))

        # Sharing must retain capture-time interpolation, not only latest poses.
        for borrower in borrowers:
            result = borrower.tf_buffer.lookup_transform(
                "odom", "body", Time(seconds=10.5),
            )
            assert result.header.stamp == Time(seconds=10.5).to_msg()
            assert result.transform.translation.x == pytest.approx(2.0)
        assert surface._lookup_pose(
            "odom", "body", Time(seconds=10.5),
        ).position.x == pytest.approx(2.0)

        surface.destroy()
        for borrower in borrowers:
            borrower.shutdown()
        assert not {"/tf", "/tf_static"}.intersection(node.destroyed_topics)
        receive_tf(TFMessage(transforms=[transform(12.0, 5.0)]))
        assert buffer.lookup_transform(
            "odom", "body", Time(),
        ).transform.translation.x == pytest.approx(5.0)

        resources.close()
        resources.close()
        assert node.destroyed_topics.count("/tf") == 1
        assert node.destroyed_topics.count("/tf_static") == 1
    finally:
        resources.close()
        py_trees.blackboard.Blackboard.clear()
