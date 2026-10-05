"""Capture cleanup cannot invalidate subscription takes queued by ROS."""

from types import SimpleNamespace

import pytest

from fault_detector_spot.application.coordinators import (
    probe_reference_capture_coordinator as capture_module,
)
from test_reference_view_input_synchronizer import make_synchronizer


class Node:
    def __init__(self):
        self.subscriptions = []
        self.destroyed = []

    def create_subscription(self, message_type, topic, callback, qos):
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
    monkeypatch.setattr(capture_module, 'capture_reference_views', lambda *_, **__: None)
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
    spec = capture_module.ProbeReferenceCaptureSpec((), False)
    for request_id in ('first', 'second'):
        if outcome == 'success':
            coordinator.run(context, request_id, spec, lambda: False)
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
