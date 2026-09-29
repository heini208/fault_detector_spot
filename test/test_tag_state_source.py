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


def message_with_tag(tag_id, stamp_sec=0.0):
    message = TagElementArray()
    tag = TagElement()
    tag.id = tag_id
    tag.pose.pose.orientation.w = 1.0
    whole = int(stamp_sec)
    tag.pose.header.stamp.sec = whole
    tag.pose.header.stamp.nanosec = round(
        (float(stamp_sec) - whole) * 1e9
    )
    message.elements.append(tag)
    return message


def test_usable_snapshot_is_copied_and_expires():
    now = [100.0]
    source = TagStateSource(
        FakeNode(),
        stale_after_sec=1.5,
        monotonic_clock=lambda: now[0],
    )
    source._receive_usable_tags(message_with_tag(7))

    tag = source.usable_tag(7)
    snapshot = source.usable_snapshot()

    assert tag.id == 7
    assert set(snapshot) == {7}
    assert snapshot[7] is not tag

    now[0] = 101.6

    assert source.usable_snapshot() == {}
    assert source.usable_tag(7) is None


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
        "fault_detector/state/usable_tags",
    ]

    source.destroy()

    assert len(node.destroyed) == 3



def test_visible_tag_after_uses_observation_time_not_receipt_time():
    now = [100.0]
    source = TagStateSource(
        FakeNode(),
        stale_after_sec=1.5,
        monotonic_clock=lambda: now[0],
    )
    source._receive_visible_tags(
        message_with_tag(7, stamp_sec=50.25)
    )

    assert source.visible_tag_after(7, 50.25) is None
    assert source.visible_tag_after(7, 50.249) is not None


def test_visible_tag_after_accepts_only_new_post_boundary_observation():
    now = [100.0]
    source = TagStateSource(
        FakeNode(),
        stale_after_sec=1.5,
        monotonic_clock=lambda: now[0],
    )
    source._receive_visible_tags(
        message_with_tag(7, stamp_sec=10.6)
    )

    assert source.visible_tag_after(7, 10.6) is None

    source._receive_visible_tags(
        message_with_tag(7, stamp_sec=10.7)
    )

    tag = source.visible_tag_after(7, 10.6)
    assert tag is not None
    assert tag.id == 7


def test_visible_tag_after_rejects_missing_observation_timestamp():
    source = TagStateSource(FakeNode())
    source._receive_visible_tags(message_with_tag(7))

    assert source.visible_tag_after(7, 10.0) is None


def test_visible_tag_after_rejects_non_finite_boundary():
    source = TagStateSource(FakeNode())

    for boundary in (float("nan"), float("inf"), float("-inf")):
        try:
            source.visible_tag_after(7, boundary)
        except ValueError:
            pass
        else:
            raise AssertionError(
                "Non-finite tag observation boundary was accepted"
            )
