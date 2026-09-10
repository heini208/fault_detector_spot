"""Validate freshness and copying of authoritative tag snapshots."""

from fault_detector_msgs.msg import TagElement, TagElementArray

from fault_detector_spot.sensing.tag_state_source import TagStateSource


class FakeNode:
    def __init__(self):
        self.subscriptions = []
        self.destroyed = []

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = object()
        self.subscriptions.append(
            (subscription, message_type, topic, callback, qos)
        )
        return subscription

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)


def message_with_tag(tag_id):
    message = TagElementArray()
    tag = TagElement()
    tag.id = tag_id
    tag.pose.pose.orientation.w = 1.0
    message.elements.append(tag)
    return message


def test_reachable_snapshot_is_copied_and_expires():
    now = [100.0]
    source = TagStateSource(
        FakeNode(),
        stale_after_sec=1.5,
        monotonic_clock=lambda: now[0],
    )
    source._receive_reachable_tags(message_with_tag(7))

    tag = source.reachable_tag(7)
    snapshot = source.reachable_snapshot()

    assert tag.id == 7
    assert set(snapshot) == {7}
    assert snapshot[7] is not tag

    now[0] = 101.6

    assert source.reachable_snapshot() == {}
    assert source.reachable_tag(7) is None


def test_source_subscribes_once_to_each_authoritative_topic():
    node = FakeNode()
    source = TagStateSource(node)

    topics = [
        item[2]
        for item in node.subscriptions
    ]

    assert topics == [
        "fault_detector/state/base_tags",
        "fault_detector/state/visible_tags",
        "fault_detector/state/reachable_tags",
    ]

    source.destroy()

    assert len(node.destroyed) == 3
