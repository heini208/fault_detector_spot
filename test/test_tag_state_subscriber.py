"""Validate fail-closed tag-state transfer into the behavior tree."""

from fault_detector_msgs.msg import TagElement, TagElementArray

from fault_detector_spot.sensing.behaviours import tag_state_subscriber


def message_with_tag(tag_id):
    message = TagElementArray()
    tag = TagElement()
    tag.id = tag_id
    tag.pose.pose.orientation.w = 1.0
    message.elements.append(tag)
    return message


def test_received_state_is_copied_and_expires(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(
        tag_state_subscriber.time,
        "monotonic",
        lambda: now[0],
    )
    subscriber = tag_state_subscriber.TagStateSubscriber(
        state_timeout_sec=1.5,
    )
    subscriber._receive_visible_tags(message_with_tag(7))

    snapshot = subscriber._fresh_snapshot(
        subscriber._visible_tags,
        subscriber._visible_receipt_time,
        now[0],
    )
    assert set(snapshot) == {7}
    assert snapshot is not subscriber._visible_tags

    now[0] = 101.6
    assert subscriber._fresh_snapshot(
        subscriber._visible_tags,
        subscriber._visible_receipt_time,
        now[0],
    ) == {}


def test_state_is_empty_before_first_message():
    subscriber = tag_state_subscriber.TagStateSubscriber()

    assert subscriber._fresh_snapshot({}, None, 100.0) == {}
