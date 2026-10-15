"""Keep localization sensing active while commands remain running."""

from types import SimpleNamespace

import py_trees
from geometry_msgs.msg import PoseWithCovarianceStamped
from rtabmap_msgs.msg import Info

from fault_detector_spot.application.behaviour_tree import runner
from fault_detector_spot.navigation.behaviours.last_localization_pose import (
    LastLocalizationPose,
)


def test_source_preserves_original_estimate_and_info_stamps_and_cleans_up():
    subscriptions = []
    destroyed = []

    def subscribe(message_type, topic, callback, qos):
        subscription = SimpleNamespace(
            message_type=message_type, topic=topic, callback=callback,
        )
        subscriptions.append(subscription)
        return subscription

    node = SimpleNamespace(
        create_subscription=subscribe,
        destroy_subscription=destroyed.append,
    )
    source = LastLocalizationPose()
    source.setup(node=node)
    try:
        assert source.blackboard.last_localization_info is None
        assert [item.topic for item in subscriptions] == ["/localization_pose", "/info"]
        pose = PoseWithCovarianceStamped()
        pose.header.stamp.sec = 12
        pose.pose.covariance[0] = 0.05
        info = Info()
        info.header.stamp.sec = 12
        info.loop_closure_id = 23
        subscriptions[0].callback(pose)
        subscriptions[1].callback(info)

        assert source.update() == py_trees.common.Status.SUCCESS
        assert source.blackboard.last_pose_estimation is pose
        assert source.blackboard.last_localization_info is info
        assert source.blackboard.last_pose_estimation.header.stamp.sec == 12
        assert source.blackboard.last_pose_estimation.pose.covariance[0] == 0.05
        assert source.blackboard.last_localization_info.header.stamp.sec == 12
    finally:
        source.shutdown()
        source.shutdown()
        source.blackboard.unregister_all_keys()
    assert destroyed == subscriptions


def test_info_is_available_before_first_localization_estimate():
    source = LastLocalizationPose()
    node = SimpleNamespace(
        create_subscription=lambda *args: object(),
        destroy_subscription=lambda subscription: None,
    )
    source.setup(node=node)
    try:
        info = Info()
        source._info_callback(info)
        assert source.update() == py_trees.common.Status.SUCCESS
        assert source.blackboard.last_localization_info is info
        assert source.blackboard.last_pose_estimation.header.stamp.sec == 0
    finally:
        source.shutdown()
        source.blackboard.unregister_all_keys()


def test_actual_root_ticks_successful_sensing_and_publishing_during_command(monkeypatch):
    counts = {"sensing": 0, "command": 0, "publisher": 0}

    class CountTicks(py_trees.behaviour.Behaviour):
        def update(self):
            counts[self.name] += 1
            if self.name == "command":
                return py_trees.common.Status.RUNNING
            return py_trees.common.Status.SUCCESS

    monkeypatch.setattr(runner, "build_sensing_tree", lambda node: CountTicks("sensing"))
    monkeypatch.setattr(runner, "build_buffered_command_tree", lambda node: CountTicks("command"))
    monkeypatch.setattr(runner, "build_publisher_tree", lambda node: CountTicks("publisher"))
    root = runner.create_behavior_tree(object())
    for _ in range(10):
        root.tick_once()
        assert root.status == py_trees.common.Status.RUNNING
    assert counts == {"sensing": 10, "command": 10, "publisher": 10}
