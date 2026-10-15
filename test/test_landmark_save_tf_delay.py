"""Asynchronous TF delivery must not prevent qualified landmark saves."""

import pytest
from fault_detector_msgs.msg import TagElementArray
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformException

from test_landmark_save_quality import localization, rig, tag  # noqa: F401


def set_map_to_body(buffer, stamp, *, x=10.0):
    transform = TransformStamped()
    transform.header.frame_id = "map"
    transform.header.stamp = Time(seconds=stamp).to_msg()
    transform.child_frame_id = "body"
    transform.transform.translation.x = x
    transform.transform.rotation.w = 1.0
    buffer.set_transform(transform, "offline_delayed_tf")


def test_continuously_new_camera_captures_qualify_with_delayed_real_tf(rig):
    buffer = Buffer()
    rig.source._tf_buffer = buffer
    set_map_to_body(buffer, 10.0)
    rig.observe()

    # Every snapshot contains a new capture. Its TF arrives one capture later,
    # so retrying only the newest observation would permanently starve saving.
    for index in range(1, 5):
        rig.advance(0.1)
        set_map_to_body(buffer, 10.0 + (index - 1) * 0.1)
        rig.observe()

    pose = rig.source.visible_tag_pose(7)
    assert pose.position.x == pytest.approx(11.0)
    assert pose.position.z == pytest.approx(0.8)


@pytest.mark.parametrize("initial_samples", [1, 2])
def test_temporary_tf_failure_preserves_accumulated_captures(rig, initial_samples):
    for _ in range(initial_samples):
        rig.observe()
        rig.advance(0.1)
    assert rig.source._landmark_windows[7].sample_count == initial_samples

    rig.buffer.transform.side_effect = TransformException("TF is still in transit")
    rig.observe()
    assert rig.source._landmark_windows[7].sample_count == initial_samples

    rig.advance(0.1)
    rig.buffer.transform.side_effect = rig.transform
    rig.observe()
    assert rig.source.visible_tag_pose(7).position.x == pytest.approx(11.0)


def test_resolved_displaced_tag_invalidates_previous_qualification(rig):
    rig.qualify()
    rig.buffer.transform.side_effect = TransformException("TF is still in transit")
    rig.observe(x=2.0)
    # The earlier validated capture remains fresh until the new capture can
    # actually be evaluated. Once transformed, the movement must reject it.
    assert rig.source.visible_tag_pose(7).position.x == pytest.approx(11.0)

    rig.advance(0.1)
    rig.buffer.transform.side_effect = rig.transform
    rig.observe(x=2.0)
    with pytest.raises(ValueError):
        rig.source.visible_tag_pose(7)


def test_resolved_invalid_pose_invalidates_previous_qualification(rig):
    rig.qualify()

    def invalid_transform(source, target, **kwargs):
        transformed = rig.transform(source, target, **kwargs)
        transformed.pose.position.x = float("nan")
        return transformed

    rig.buffer.transform.side_effect = invalid_transform
    rig.observe()
    with pytest.raises(ValueError):
        rig.source.visible_tag_pose(7)


def test_fresh_captures_waiting_for_tf_cannot_extend_old_qualification(rig):
    rig.qualify()
    rig.buffer.transform.side_effect = TransformException("TF stream stopped")
    for _ in range(17):
        rig.advance(0.1)
        rig.observe()

    # New camera messages and localization estimates continue, but no new
    # map pose has been validated since the earlier qualification expired.
    with pytest.raises(ValueError):
        rig.source.visible_tag_pose(7)


def test_tag_disappearing_clears_pending_captures(rig):
    rig.observe()
    rig.advance(0.1)
    rig.buffer.transform.side_effect = TransformException("TF is still in transit")
    rig.observe()
    rig.source._receive_visible_tags(TagElementArray())

    rig.advance(0.1)
    rig.buffer.transform.side_effect = rig.transform
    rig.observe()
    assert rig.source._landmark_windows[7].sample_count == 1
    with pytest.raises(ValueError):
        rig.source.visible_tag_pose(7)


def test_pending_captures_from_before_bad_localization_cannot_qualify(rig):
    rig.observe()
    rig.advance(0.1)
    rig.buffer.transform.side_effect = TransformException("TF is still in transit")
    rig.observe()
    rig.advance(0.1)
    now = rig.clock.now().nanoseconds / 1e9
    rig.source._receive_localization_pose(localization(now, variance=9999.0))

    rig.advance(0.1)
    rig.buffer.transform.side_effect = rig.transform
    rig.observe()
    assert rig.source._landmark_windows[7].sample_count == 1
    with pytest.raises(ValueError):
        rig.source.visible_tag_pose(7)
