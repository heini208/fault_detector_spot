"""Stationary target depth and lateral observation geometry contracts."""

from types import SimpleNamespace

import numpy as np
import pytest

from fault_detector_spot.inspection.geometry.pose import compose_poses, inverse_pose
from fault_detector_spot.inspection.geometry.sensor_tip_observation import (
    observation_translation,
    project_camera_point,
    target_camera_point,
    transform_surface_plane,
)
from fault_detector_spot.inspection.sensing import probe_surface_source
from fault_detector_spot.inspection.sensing.probe_surface_source import (
    ProbeSurfaceSource, NoNewSensorTipPoseSample,
)
from fault_detector_spot.inspection.sensing.sensor_tip_distance import measure_sensor_tip_distance
from fault_detector_spot.shared.geometry.models import PoseData, Vector3Data
from fault_detector_spot.shared.geometry.rotation import quaternion_from_matrix
from test_live_surface_distance import fitted_plane, planar_depth


def camera_pose(height=0.0):
    # Optical +Z faces probe +X; optical +Y points below probe -Z.
    return PoseData(
        Vector3Data(-0.10, 0.0, height),
        quaternion_from_matrix(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
    )


def target_image(depth_m=0.35):
    image, info = planar_depth(width=128, height=96, depth_m=depth_m)
    info.k[0] = info.k[4] = 200.0
    return image, info


def test_lowered_tip_moves_only_until_full_patch_is_visible_without_inward_motion():
    reference = PoseData.identity()
    camera = camera_pose(height=0.10)
    point = target_camera_point(reference, camera, 0.25)
    pixel = project_camera_point(point, (200.0, 200.0, 64.0, 48.0))
    assert pixel[1] > 96
    translation = observation_translation(reference, camera, 0.25, (200, 200, 64, 48), (128, 96), 36)
    assert translation.x == pytest.approx(0.0)
    assert translation.y == pytest.approx(0.0)
    assert translation.z == pytest.approx(-0.08075)
    camera.position.z += translation.z
    centered = target_camera_point(reference, camera, 0.25)
    assert project_camera_point(centered, (200.0, 200.0, 64.0, 48.0)) == pytest.approx((64, 59))


def test_observation_translation_respects_rotated_probe_axis():
    anchor = PoseData(
        Vector3Data(1.0, -2.0, 3.0),
        quaternion_from_matrix(np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])),
    )
    camera = compose_poses(anchor, camera_pose(height=0.10))
    translation = observation_translation(anchor, camera, 0.25, (200, 200, 64, 48), (128, 96), 36)
    assert translation.y == pytest.approx(0.0)
    assert translation.z == pytest.approx(-0.08075)


def test_plane_transform_preserves_normal_sign_and_round_trips():
    plane = fitted_plane(0.25)
    transformed = transform_surface_plane(plane, camera_pose(), "odom")
    assert transformed.normal.z == pytest.approx(0.0)
    restored = transform_surface_plane(transformed, inverse_pose(camera_pose()), "probe")
    assert restored.point_array() == pytest.approx(plane.point_array())
    assert restored.normal_array() == pytest.approx(plane.normal_array())


def test_measurement_refers_to_original_tip_after_camera_excursion():
    image, info = target_image()
    reference = PoseData.identity()
    observation_camera = camera_pose(height=0.01)
    sample = measure_sensor_tip_distance(image, info, reference, observation_camera, 0.25)
    assert sample.distance_m == pytest.approx(0.25, abs=1e-6)
    assert sample.surface_plane_probe.normal.x == pytest.approx(-1.0)
    assert sample.surface_plane_probe.frame_id == "probe"
    # An odom translation and rotation do not alter the original tip distance.
    anchor = camera_pose(height=1.0)
    shifted = measure_sensor_tip_distance(
        image, info, compose_poses(anchor, reference),
        compose_poses(anchor, observation_camera), 0.25,
    )
    assert shifted.distance_m == pytest.approx(sample.distance_m)
    assert shifted.surface_plane_probe.normal_array() == pytest.approx(
        sample.surface_plane_probe.normal_array(),
    )


def test_measurement_does_not_extrapolate_plane_when_target_is_outside_image():
    image, info = target_image()
    with pytest.raises(ValueError, match="outside the camera image"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(0.10), 0.25)


@pytest.mark.parametrize("depth", [0.0, 0.12])
def test_missing_or_occluded_target_cannot_be_replaced_by_surrounding_wall(depth):
    image, info = target_image()
    values = np.frombuffer(bytes(image.data), dtype="<f4").reshape(96, 128).copy()
    values[45:52, 61:68] = depth
    image.data = values.tobytes()
    with pytest.raises(ValueError, match="insufficient unoccluded|too close"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(), 0.25)


def test_foreground_plane_must_match_expected_target_distance():
    image, info = target_image(depth_m=0.31)
    with pytest.raises(ValueError, match="does not match the intended"):
        measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(), 0.40)


def test_saved_distance_checks_correspondence_without_selecting_the_fitted_patch():
    image, info = planar_depth(width=128, height=256, depth_m=0.35)
    info.k[0] = info.k[4] = 200.0
    # A different saved distance must not shift the fitted patch away from
    # the actual intersection or stop its otherwise supported measurement.
    sample = measure_sensor_tip_distance(image, info, PoseData.identity(), camera_pose(0.15), 0.16)
    assert sample.distance_m == pytest.approx(0.25, abs=1e-6)


@pytest.fixture
def source(monkeypatch):
    state = SimpleNamespace(now=100.0, ros_now=10.0, poses=[], fits=0)
    monkeypatch.setattr(probe_surface_source.time, "monotonic", lambda: state.now)
    state.source = ProbeSurfaceSource(SimpleNamespace(
        create_subscription=lambda *args, **kwargs: object(),
        destroy_subscription=lambda subscription: None,
        count_publishers=lambda topic: 1,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(state.ros_now * 1e9),
        )),
    ), None)
    image, info = target_image()
    state.source._receive_hand_depth_camera_info(info)
    state.source._receive_hand_depth(image)

    def lookup(target, source, lookup_time):
        state.poses.append((target, source, lookup_time.nanoseconds))
        return camera_pose()

    state.source._lookup_pose = lookup
    original_measure = probe_surface_source.measure_sensor_tip_distance

    def measure(*args, **kwargs):
        state.fits += 1
        return original_measure(*args, **kwargs)

    monkeypatch.setattr(probe_surface_source, "measure_sensor_tip_distance", measure)
    return state


def test_target_sample_uses_acquisition_tf_on_every_poll_and_caches_fit(source):
    for _ in range(2):
        samples = source.source.sensor_tip_distance_samples(PoseData.identity(), 0.25)
        assert samples[0].distance_m == pytest.approx(0.25, abs=1e-6)
    assert source.fits == 1
    assert source.poses == [("odom", "hand_depth", 10_000_000_000)] * 2


def test_new_saved_reference_invalidates_target_fit(source):
    reference = PoseData.identity()
    source.source.sensor_tip_distance_samples(reference, 0.25)
    reference.position.x += 0.01
    sample = source.source.sensor_tip_distance_samples(reference, 0.24)[0]
    assert source.fits == 2
    assert sample.distance_m == pytest.approx(0.24, abs=1e-6)


def test_observation_and_sample_cache_distinguish_discovery_from_saved_correspondence(source):
    image, _ = target_image(depth_m=0.48)
    image.header.stamp.nanosec = 100_000_000
    source.now += 0.1
    source.ros_now += 0.1
    source.source._receive_hand_depth(image)
    reference = PoseData.identity()
    assert not source.source.observation_view(reference, 0.25).visible
    view = source.source.observation_view(reference)
    assert view.visible
    assert view.hand_translation_odom == Vector3Data.zero()
    for discover in (False, True, False, True):
        if discover:
            samples = source.source.sensor_tip_distance_samples(
                reference,
                receipt_not_before=source.now,
            )
            assert samples[0].distance_m == pytest.approx(0.38, abs=1e-6)
        else:
            with pytest.raises(ValueError, match="does not match the intended"):
                source.source.sensor_tip_distance_samples(
                    reference, 0.25, receipt_not_before=source.now,
                )


def test_target_samples_preserve_receipt_freshness_and_count_contract(source):
    with pytest.raises(ValueError, match="post_start=0"):
        source.source.sensor_tip_distance_samples(PoseData.identity(), 0.25, receipt_not_before=100.1)
    with pytest.raises(ValueError, match="at least 2 fresh"):
        source.source.sensor_tip_distance_samples(PoseData.identity(), 0.25, minimum_samples=2)
    source.now += 0.6
    with pytest.raises(ValueError, match="within_age=0"):
        source.source.sensor_tip_distance_samples(PoseData.identity(), 0.25)
    with pytest.raises(ValueError, match="Latest valid.*stale"):
        source.source.sensor_tip_distance_samples(
            PoseData.identity(), 0.25, maximum_age_sec=6.0, maximum_latest_age_sec=0.5,
        )


def test_observation_view_and_snapshot_use_same_timestamped_frames(source):
    view = source.source.observation_view(PoseData.identity(), 0.25)
    assert view.visible
    snapshot = source.source.sensor_tip_pose_snapshot("test_sensor")
    assert len(snapshot) == 3
    assert source.poses == [
        ("odom", "hand_depth", 10_000_000_000),
        ("odom", "hand", 10_000_000_000),
        ("odom", "test_sensor_probe", 10_000_000_000),
        ("odom", "body", 10_000_000_000),
    ]


def test_pose_snapshot_refuses_pre_return_and_empty_timestamp_frames(source):
    with pytest.raises(ValueError, match="after the required start"):
        source.source.sensor_tip_pose_snapshot("test_sensor", receipt_not_before=100.1)
    image, _ = target_image()
    image.header.stamp.sec = 0
    source.source._receive_hand_depth(image)
    with pytest.raises(ValueError, match="timestamp is empty"):
        source.source.sensor_tip_pose_snapshot("test_sensor")


def test_visible_target_does_not_require_motion_to_image_center():
    camera = camera_pose(height=0.01)
    translation = observation_translation(
        PoseData.identity(), camera, 0.25, (200, 200, 64, 48), (128, 96), 36,
    )
    assert translation == Vector3Data.zero()


def test_captured_mount_geometry_needs_far_less_than_full_centering():
    camera = PoseData(
        Vector3Data(-0.114589969, 0.0182147651, 0.130271103),
        quaternion_from_matrix(np.array([
            [0.000004199, 0.136085467, 0.990697101],
            [-1.0, -0.0000006196, 0.0000043235],
            [0.0000012022, -0.990697101, 0.136085467],
        ])),
    )
    intrinsics = (552.0291012161067, 552.0291012161067, 320, 240)
    reference = PoseData.identity()
    before = project_camera_point(target_camera_point(reference, camera, 0.25), intrinsics)
    assert before[1] == pytest.approx(527.17, abs=0.01)
    delta = observation_translation(reference, camera, 0.25, intrinsics, (640, 480), 36)
    assert delta.x == pytest.approx(0, abs=1e-9)
    assert np.linalg.norm([delta.x, delta.y, delta.z]) < 0.06
    camera.position = Vector3Data(camera.position.x+delta.x, camera.position.y+delta.y, camera.position.z+delta.z)
    after = project_camera_point(target_camera_point(reference, camera, 0.25), intrinsics)
    assert after == pytest.approx((before[0], 443))


@pytest.mark.parametrize("size, margin", [((64, 64), 36), ((128, 96), -1)])
def test_invalid_observation_image_margin_is_rejected(size, margin):
    with pytest.raises(ValueError, match="patch margin"):
        observation_translation(PoseData.identity(), camera_pose(.1), .25, (200, 200, 64, 48), size, margin)


def test_pose_sample_rejects_delayed_acquisition_even_after_new_receipt(source):
    source.now = 100.2
    source.ros_now = 10.2
    image, _ = target_image()  # Acquired at ROS 10.0, delivered after movement.
    source.source._receive_hand_depth(image)
    with pytest.raises(ValueError, match="acquisition precedes"):
        source.source.sensor_tip_pose_sample(
            "test_sensor", receipt_not_before=100.1, acquisition_not_before=10.1,
        )
    assert source.poses == []


def test_pose_sample_exposes_acquisition_identity_for_all_three_transforms(source):
    source.now = 100.2
    source.ros_now = 10.3
    image, _ = target_image()
    image.header.stamp.nanosec = 200_000_000
    source.source._receive_hand_depth(image)
    sample = source.source.sensor_tip_pose_sample(
        "test_sensor", receipt_not_before=100.1, acquisition_not_before=10.1,
    )
    assert sample.stamp_seconds == pytest.approx(10.2)
    assert sample.hand == camera_pose()
    assert sample.probe == camera_pose()
    assert sample.body == camera_pose()
    assert source.poses == [
        ("odom", "hand", 10_200_000_000),
        ("odom", "test_sensor_probe", 10_200_000_000),
        ("odom", "body", 10_200_000_000),
    ]


@pytest.mark.parametrize("ros_now,match", [
    (10.6, "acquisition is stale"),
    (9.99, "timestamp is in the future"),
])
def test_pose_sample_rejects_stale_or_future_acquisition_before_tf(source, ros_now, match):
    source.ros_now = ros_now
    with pytest.raises(ValueError, match=match):
        source.source.sensor_tip_pose_sample("test_sensor")
    assert source.poses == []


def test_pose_sample_freshness_is_rechecked_after_tf_resolution(source):
    lookup = source.source._lookup_pose

    def delayed_lookup(*args, **kwargs):
        result = lookup(*args, **kwargs)
        source.ros_now += .2
        return result

    source.source._lookup_pose = delayed_lookup
    with pytest.raises(ValueError, match="acquisition is stale"):
        source.source.sensor_tip_pose_sample("test_sensor")
    assert len(source.poses) == 3


@pytest.mark.parametrize("error", ["extrapolation into the past", "extrapolation into the future"])
def test_pose_sample_does_not_fall_back_to_latest_tf(source, error):
    requested = []

    def unavailable(target, frame, lookup_time):
        requested.append((target, frame, lookup_time.nanoseconds))
        raise RuntimeError(error)

    source.source._lookup_pose = unavailable
    with pytest.raises(RuntimeError, match=error):
        source.source.sensor_tip_pose_sample("test_sensor")
    assert requested == [("odom", "hand", 10_000_000_000)]


def test_pose_sample_rejects_nonfinite_header_before_tf(source):
    _, info = target_image()
    invalid = SimpleNamespace(header=SimpleNamespace(
        frame_id="hand_depth", stamp=SimpleNamespace(sec=float("nan"), nanosec=0),
    ))
    source.source.latest_hand_depth = lambda **kwargs: (invalid, info)
    with pytest.raises(ValueError, match="timestamp is empty or nonfinite"):
        source.source.sensor_tip_pose_sample("test_sensor")
    assert source.poses == []


def test_pose_samples_keep_duplicate_and_out_of_order_acquisition_identity(source):
    first = source.source.sensor_tip_pose_sample("test_sensor")
    second = source.source.sensor_tip_pose_sample("test_sensor")
    assert first.stamp_seconds == second.stamp_seconds
    source.now += .1
    source.ros_now += .1
    older, _ = target_image()
    older.header.stamp.sec = 9
    older.header.stamp.nanosec = 900_000_000
    source.source._receive_hand_depth(older)
    latest_receipt = source.source.sensor_tip_pose_sample("test_sensor")
    assert latest_receipt.stamp_seconds == pytest.approx(9.9)
    assert latest_receipt.stamp_seconds < first.stamp_seconds


def test_consumed_pose_frame_aging_between_updates_is_no_new_evidence(source):
    accepted = source.source.sensor_tip_pose_sample("test_sensor")
    source.now += 0.7
    source.ros_now += 0.7
    source.poses.clear()
    with pytest.raises(NoNewSensorTipPoseSample, match="distinct, advancing"):
        source.source.sensor_tip_pose_sample("test_sensor", after_stamp_seconds=accepted.stamp_seconds)
    assert source.poses == []
    image, _ = target_image()
    image.header.stamp.nanosec = 400_000_000
    source.source._receive_hand_depth(image)
    sample = source.source.sensor_tip_pose_sample("test_sensor", after_stamp_seconds=accepted.stamp_seconds)
    assert sample.stamp_seconds == pytest.approx(10.4)
    assert len(source.poses) == 3


def test_unseen_stale_pose_frame_remains_invalid(source):
    source.now += 0.7
    source.ros_now += 0.7
    image, _ = target_image()
    image.header.stamp.nanosec = 100_000_000
    source.source._receive_hand_depth(image)
    with pytest.raises(ValueError, match="acquisition is stale") as failure:
        source.source.sensor_tip_pose_sample("test_sensor", after_stamp_seconds=10.0)
    assert not isinstance(failure.value, NoNewSensorTipPoseSample)
    assert source.poses == []


@pytest.mark.parametrize("threshold", [-0.1, float("nan"), float("inf")])
def test_pose_sample_rejects_invalid_previous_stamp(source, threshold):
    with pytest.raises(ValueError, match="Previous pose acquisition timestamp"):
        source.source.sensor_tip_pose_sample("test_sensor", after_stamp_seconds=threshold)


def test_pose_cutoff_uses_ros_clock_and_explicit_settle_delay(source):
    assert source.source.pose_acquisition_cutoff(.2) == pytest.approx(10.2)
    assert source.now == 100.0


@pytest.mark.parametrize("delay", [-.1, float("nan"), float("inf")])
def test_pose_cutoff_rejects_invalid_delays(source, delay):
    with pytest.raises(ValueError, match="delay must be finite and nonnegative"):
        source.source.pose_acquisition_cutoff(delay)


@pytest.mark.parametrize("threshold", [-.1, float("nan"), float("inf")])
def test_pose_sample_rejects_invalid_acquisition_threshold(source, threshold):
    with pytest.raises(ValueError, match="threshold must be finite and nonnegative"):
        source.source.sensor_tip_pose_sample("test_sensor", acquisition_not_before=threshold)
