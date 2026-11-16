"""Capture cleanup cannot invalidate subscription takes queued by ROS."""

from types import SimpleNamespace

import pytest

from fault_detector_spot.application.coordinators import (
    probe_reference_capture_coordinator as capture_module,
)
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin, CommandRequest, RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import SemanticCommand
from fault_detector_spot.application.controllers.command_controller import CommandController
from fault_detector_spot.application.coordinators.setup_coordinator import SetupCoordinator
from test_reference_view_input_synchronizer import make_synchronizer


class Node:
    def __init__(self):
        self.subscriptions = []
        self.destroyed = []

    def create_subscription(self, message_type, topic, callback, qos, *, callback_group=None):
        subscription = SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)

    def get_clock(self):
        return SimpleNamespace(now=lambda: 1)


@pytest.fixture
def coordinator(monkeypatch):
    monkeypatch.setattr(capture_module.tf2_ros, 'Buffer', lambda: object())
    detached = []
    monkeypatch.setattr(
        capture_module.tf2_ros, 'TransformListener',
        lambda *_: SimpleNamespace(unregister=lambda: detached.append(True)),
    )
    snapshot = SimpleNamespace(selected_object_id='motor', selected_routine_id='scan')
    setup = SimpleNamespace(
        snapshot=lambda _: snapshot,
        require_current=lambda _: None,
        context=lambda *_: object(),
        select_routine=lambda *_: snapshot,
    )
    value = capture_module.ProbeReferenceCaptureCoordinator(
        Node(), setup, object(), object(),
    )
    value._require_command_lane_idle = lambda: None
    value._wait_settled = lambda _: None
    value._wait_for_collections = lambda *_: None
    value._historical_reference_tags = lambda *_: (object(),)
    monkeypatch.setattr(capture_module, 'validate_multi_reference_view_capture_target', lambda *_: 7)
    value.captured_requests = []
    monkeypatch.setattr(
        capture_module, 'resolve_reference_views',
        lambda requests, *_, **__: list(requests),
    )
    value.reference_repository = SimpleNamespace(
        save_reference_views=lambda _object, _routine, captures, **_: value.captured_requests.append(captures),
    )
    value.detached = detached
    return value


@pytest.mark.parametrize('outcome', ['success', 'cancel', 'failure'])
def test_capture_reuses_handles_until_executor_shutdown(coordinator, outcome):
    def ready(*_):
        if outcome == 'cancel':
            raise capture_module.ReferenceCaptureCancelled('cancelled')
        if outcome == 'failure':
            raise ValueError('invalid capture')

    coordinator._wait_for_ready_inputs = ready
    context = SimpleNamespace(context_id='context', client_id='client')
    spec = capture_module.ProbeReferenceCaptureSpec(replace_existing=False)
    for request_id in ('first', 'second'):
        if outcome == 'success':
            coordinator.run(context, request_id, spec, lambda: False)
            assert [
                (request.slot_index, request.camera_id)
                for request in coordinator.captured_requests[-1]
            ] == [
                (0, 'frontleft'), (1, 'frontright'), (2, 'left'),
                (3, 'right'), (4, 'back'), (5, 'hand'),
            ]
        else:
            with pytest.raises((capture_module.ReferenceCaptureCancelled, ValueError)):
                coordinator.run(context, request_id, spec, lambda: False)
        assert len(coordinator.node.subscriptions) == 24
        assert coordinator.node.destroyed == []
        assert not coordinator._active_request_id
        for synchronizer in coordinator._synchronizers.values():
            assert not synchronizer.collection_active
            assert not synchronizer.ready_for_collection()
            # Simulate a take already queued by the executor at cleanup.
            # It must be harmless, without accepting stale data.
            synchronizer.rgb_subscription.callback(object())
            synchronizer.depth_subscription.callback(object())
            synchronizer.rgb_camera_info_subscription.callback(object())
            synchronizer.depth_camera_info_subscription.callback(object())
            assert not synchronizer.ready_for_collection()
    coordinator.request_shutdown()
    assert coordinator.node.destroyed == []
    # ApplicationApiNode calls close only after executor.shutdown().
    coordinator.close()
    assert len(coordinator.node.destroyed) == 24
    assert coordinator.detached == [True]
    coordinator.close()
    assert len(coordinator.node.destroyed) == 24
    assert coordinator.detached == [True]
    with pytest.raises(capture_module.ReferenceCaptureCancelled):
        coordinator.run(context, 'late', spec, lambda: False)


@pytest.mark.parametrize('active_motion', [False, True])
def test_reference_capture_allows_stop_observation_but_rejects_owned_motion(coordinator, active_motion):
    sent = []
    commands = CommandController(dispatch_request=sent.append)
    setup = SetupCoordinator(commands)
    coordinator.probe_setup_coordinator.require_configuration_idle = setup.require_configuration_idle
    coordinator._require_command_lane_idle = (
        capture_module.ProbeReferenceCaptureCoordinator._require_command_lane_idle.__get__(coordinator)
    )
    coordinator._wait_for_ready_inputs = lambda *_: None
    if active_motion:
        commands.submit(CommandRequest.create(
            SemanticCommand(command_id=CommandID.STAND_UP),
            client_id='external-ui', origin=CommandOrigin.OPERATIONAL,
            recording_policy=RecordingPolicy.EXCLUDE,
        ))
    commands.cancel_all()
    context = SimpleNamespace(context_id='context', client_id='client')
    spec = capture_module.ProbeReferenceCaptureSpec(replace_existing=False)

    if active_motion:
        with pytest.raises(RuntimeError):
            coordinator.run(context, 'capture', spec, lambda: False)
        assert not coordinator.captured_requests
        assert not coordinator.node.subscriptions
    else:
        coordinator.run(context, 'capture', spec, lambda: False)
        assert len(coordinator.captured_requests) == 1
        assert [request.command.command_id for request in sent] == [CommandID.EMERGENCY_CANCEL]
    assert not coordinator._active_request_id
    assert commands.software_stop_status.latched
    assert not commands.software_stop_status.stop_confirmed


def test_suspended_synchronizer_drops_history_and_requires_fresh_input():
    synchronizer, node = make_synchronizer()
    message = SimpleNamespace(header=SimpleNamespace(frame_id='camera'))
    for subscription in node.subscriptions:
        subscription.callback(message)
    assert synchronizer.ready_for_collection()
    sequence = synchronizer.input_sequence
    synchronizer.begin_collection(sequence + 1)
    synchronizer.suspend()
    for subscription in node.subscriptions:
        subscription.callback(message)
    assert synchronizer.input_sequence == sequence
    assert not synchronizer.collection_active
    assert not synchronizer.ready_for_collection()
    synchronizer.resume()
    assert not synchronizer.ready_for_collection()
    for subscription in node.subscriptions:
        subscription.callback(message)
    assert synchronizer.ready_for_collection()
    assert synchronizer.input_sequence == sequence + 2


@pytest.mark.parametrize('valid_after_windows', [4, None])
def test_empty_windows_do_not_exhaust_attempts_before_timeout(
    coordinator, monkeypatch, valid_after_windows,
):
    clock = [0.0]
    monkeypatch.setattr(capture_module.time, 'monotonic', lambda: clock[0])
    coordinator._wait_for_ready_inputs = lambda *_: None

    def finish_window(*_):
        clock[0] += coordinator.collection_duration_sec

    coordinator._wait_for_collections = finish_window
    windows = []

    def capture(*args, **kwargs):
        windows.append(clock[0])
        if valid_after_windows is None or len(windows) < valid_after_windows:
            raise capture_module.ReferenceViewCaptureNotReady('no images')
        return list(args[0])

    monkeypatch.setattr(capture_module, 'resolve_reference_views', capture)
    context = SimpleNamespace(context_id='context', client_id='client')
    spec = capture_module.ProbeReferenceCaptureSpec(replace_existing=False)
    if valid_after_windows is None:
        with pytest.raises(RuntimeError, match='failed after 3 attempts: frontleft: no images'):
            coordinator.run(context, 'capture', spec, lambda: False)
        assert clock[0] >= (
            coordinator.capture_timeout_sec * coordinator.capture_max_attempts
        )
    else:
        coordinator.run(context, 'capture', spec, lambda: False)
        assert len(windows) == valid_after_windows + 5
    assert not coordinator._active_request_id
    assert all(not value.collection_active for value in coordinator._synchronizers.values())


def test_capture_can_be_cancelled_between_empty_windows(coordinator, monkeypatch):
    coordinator._wait_for_ready_inputs = lambda *_: None
    cancelled = [False]

    def capture(*args, **kwargs):
        cancelled[0] = True
        raise capture_module.ReferenceViewCaptureNotReady('no images')

    monkeypatch.setattr(capture_module, 'resolve_reference_views', capture)
    with pytest.raises(capture_module.ReferenceCaptureCancelled):
        coordinator.run(
            SimpleNamespace(context_id='context', client_id='client'),
            'capture', capture_module.ProbeReferenceCaptureSpec(False),
            lambda: cancelled[0],
        )
    assert not coordinator._active_request_id
    assert all(not value.collection_active for value in coordinator._synchronizers.values())


@pytest.mark.parametrize('deliver', [True, False])
def test_collection_waits_for_delayed_fresh_pair_or_timeout(coordinator, monkeypatch, deliver):
    from test_reference_view_collection_window import make_synchronizer, make_image
    from fault_detector_spot.inspection.setup import reference_view_input_synchronizer

    clock = [0.0]
    monkeypatch.setattr(capture_module.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(reference_view_input_synchronizer.time, 'monotonic_ns',
                        lambda: int(clock[0] * 1_000_000_000))
    coordinator.node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=11_000_000_000),
    )
    synchronizer = make_synchronizer()
    synchronizers = {'hand': synchronizer}
    selected = ((0, 'hand'),)
    coordinator._begin_collections(synchronizers, selected)

    def wait(_):
        clock[0] += 0.25
        if deliver and clock[0] == 2.0:
            synchronizer._rgb_callback(make_image(100_000_000))
            synchronizer._depth_callback(make_image(105_000_000))

    coordinator._shutdown = SimpleNamespace(is_set=lambda: False, wait=wait)
    collect = capture_module.ProbeReferenceCaptureCoordinator._wait_for_collections
    if deliver:
        collect(coordinator, synchronizers, selected, lambda: False)
        assert clock[0] == 2.0
        assert synchronizer.best_snapshot(0) is not None
    else:
        with pytest.raises(capture_module.ReferenceViewCaptureNotReady,
                           match='Timed out.*hand:.*rgb_frames=0'):
            collect(coordinator, synchronizers, selected, lambda: False)
        assert clock[0] == coordinator.capture_timeout_sec


@pytest.mark.parametrize('recover', [True, False])
def test_only_failed_camera_retries_and_complete_set_is_saved_once(
    coordinator, monkeypatch, recover,
):
    clock = [0.0]
    monkeypatch.setattr(capture_module.time, 'monotonic', lambda: clock[0])
    coordinator._wait_for_ready_inputs = lambda *_: None
    seen = []
    accepted = {}

    def resolve(requests, *args, **kwargs):
        request = requests[0]
        seen.append(request.camera_id)
        clock[0] += coordinator.capture_timeout_sec
        if request.camera_id == 'frontleft' and (not recover or seen.count('frontleft') == 1):
            raise capture_module.ReferenceViewCaptureNotReady('missing frontleft')
        assert request.camera_id not in accepted
        accepted[request.camera_id] = request
        return [request]

    monkeypatch.setattr(capture_module, 'resolve_reference_views', resolve)
    context = SimpleNamespace(context_id='context', client_id='client')
    spec = capture_module.ProbeReferenceCaptureSpec(False)
    if recover:
        coordinator.run(context, 'capture', spec, lambda: False)
        assert seen.count('frontleft') == 2
        assert len(coordinator.captured_requests) == 1
        saved = coordinator.captured_requests[0]
        assert [item.slot_index for item in saved] == list(range(6))
        assert all(item is accepted[item.camera_id] for item in saved)
    else:
        with pytest.raises(RuntimeError, match='frontleft: missing frontleft'):
            coordinator.run(context, 'capture', spec, lambda: False)
        assert seen.count('frontleft') == coordinator.capture_max_attempts
        assert not coordinator.captured_requests
    for camera in ('frontright', 'left', 'right', 'back', 'hand'):
        assert seen.count(camera) == 1
