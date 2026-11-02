"""Setup waits must finish without ROS feedback during executor shutdown."""

from concurrent.futures import ThreadPoolExecutor
from threading import Event, RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_spot.application.api import application_api_node
from fault_detector_spot.application.api.navigation_setup_api import NavigationSetupApi
from fault_detector_spot.application.api.probe_setup_motion_api import ProbeSetupMotionApi
from fault_detector_spot.application.api.shutdown_callbacks import ShutdownCallbackActivity
from fault_detector_spot.application.api.async_operation import OperationAdmission
from fault_detector_spot.application.coordinators import (
    probe_reference_capture_coordinator as capture_module,
)
from fault_detector_spot.application.coordinators.probe_refinement_finalization_coordinator import (
    ProbeRefinementFinalizationCoordinator,
)
from fault_detector_spot.application.coordinators.probe_reference_capture_coordinator import (
    ProbeReferenceCaptureCoordinator,
    ReferenceCaptureCancelled,
)
from fault_detector_spot.inspection.setup.probe_setup_motion import ProbeMotionKind


@pytest.mark.parametrize("api_type", [NavigationSetupApi, ProbeSetupMotionApi])
def test_active_setup_action_exits_without_terminal_feedback(api_type):
    api = api_type.__new__(api_type)
    api._lock = RLock()
    api._shutdown = Event()
    api._shutdown_callbacks = ShutdownCallbackActivity()
    api._executions = {}
    api._admission = OperationAdmission(limit=1)
    api._early_states = {}
    api._runtime_submissions = 0
    api.coordinator = Mock()
    api._abort = Mock(return_value="aborted")
    api._motion_request = Mock()
    entered = Event()
    operation = SimpleNamespace(request_id="request")
    goal = Mock(is_cancel_requested=False)
    goal.request.intent.operation = 1
    goal.request.context_id = "context"
    goal.request.client_id = "client"

    if api_type is NavigationSetupApi:
        api._pose_waiters = set()
        def submit(**kwargs):
            entered.set()
            return operation
        api.coordinator.submit_runtime_operation.side_effect = submit
        # START_MAPPING is a runtime operation supported by this API.
        from fault_detector_msgs.msg import NavigationSetupIntent
        goal.request.intent.operation = NavigationSetupIntent.OPERATION_START_MAPPING
        call = lambda: api._execute(goal)
    else:
        api.coordinator.prepare_motion.return_value = operation
        api.coordinator.submit_motion.side_effect = lambda op: entered.set()
        call = lambda: api._execute(goal)

    from rclpy.action import GoalResponse
    assert api._accept(goal.request) == GoalResponse.ACCEPT
    coroutine = call()
    waiting = coroutine.send(None)
    assert entered.is_set()
    assert not waiting.done()
    api.request_shutdown()
    api.request_shutdown()
    with pytest.raises(StopIteration) as completed:
        coroutine.send(None)
    assert completed.value.value == "aborted"
    assert not api._executions
    assert "shutdown" in api._abort.call_args.args[2].lower()
    assert "stop unconfirmed" in api._abort.call_args.args[2].lower()
    api._abort.assert_called_once()


def test_retraction_wait_exits_without_command_feedback():
    coordinator = Mock()
    coordinator.prepare_motion.return_value = SimpleNamespace(request_id="motion")
    entered = Event()
    coordinator.submit_motion.side_effect = lambda op: entered.set()
    runner = ProbeRefinementFinalizationCoordinator(coordinator)
    with ThreadPoolExecutor(max_workers=1) as worker:
        future = worker.submit(
            runner._execute_retraction, "context", "client", "finalization",
            ProbeMotionKind.MOVE_SAFE_APPROACH, lambda: False, 0.01, 0.1,
        )
        try:
            assert entered.wait(1)
            runner.request_shutdown()
            with pytest.raises(RuntimeError, match="shutdown"):
                future.result(timeout=1)
            assert not runner._waiters
        finally:
            runner.request_shutdown()
    # Work already accepted but not started must also stop at admission.
    with pytest.raises(RuntimeError, match="shutting down"):
        runner._execute_retraction(
            "context", "client", "finalization",
            ProbeMotionKind.MOVE_SAFE_APPROACH, lambda: False, 0.01, 0.1,
        )
    assert coordinator.submit_motion.call_count == 1


def test_capture_signal_preserves_resources_until_callback_drain(monkeypatch):
    listener = Mock(spec=["unregister"])
    monkeypatch.setattr(capture_module.tf2_ros, "Buffer", Mock())
    monkeypatch.setattr(
        capture_module.tf2_ros, "TransformListener", Mock(return_value=listener)
    )
    node = Mock(spec=["create_subscription", "destroy_subscription"])
    capture = ProbeReferenceCaptureCoordinator(node, Mock(), Mock(), Mock())
    capture._create_synchronizers(((0, "hand"),))
    capture.request_shutdown()
    with pytest.raises(ReferenceCaptureCancelled, match="shutdown"):
        capture._check_cancel(lambda: False)
    listener.unregister.assert_not_called()
    node.destroy_subscription.assert_not_called()
    capture.close()
    listener.unregister.assert_called_once()
    assert node.destroy_subscription.call_count == 4


def test_main_uses_ordered_shutdown_before_ros_teardown(monkeypatch):
    events = []
    node = Mock()
    def shutdown(executor):
        events.extend(["ordered_shutdown", "ros"])
        return SimpleNamespace(cleanup_errors={})
    node.shutdown.side_effect = shutdown
    executor = Mock()
    executor.spin_once.side_effect = KeyboardInterrupt
    monkeypatch.setattr(application_api_node, "ApplicationApiNode", lambda: node)
    monkeypatch.setattr(application_api_node, "MultiThreadedExecutor", lambda **kw: executor)
    monkeypatch.setattr(application_api_node.rclpy, "init", Mock())
    monkeypatch.setattr(application_api_node.rclpy, "ok", lambda: True)
    monkeypatch.setattr(application_api_node.rclpy, "try_shutdown", lambda: events.append("ros"))
    application_api_node.main()
    assert events == ["ordered_shutdown", "ros"]
    node.shutdown.assert_called_once_with(executor)
    assert application_api_node.rclpy.init.call_args.kwargs["signal_handler_options"] == (
        application_api_node.SignalHandlerOptions.NO
    )


def test_node_requests_stop_idempotently_through_the_command_controller():
    node = SimpleNamespace(_shutdown_stop_request="", _shutdown_lock=RLock(), application_controller=Mock())
    node.application_controller.emergency_stop.return_value = "shutdown-stop"
    application_api_node.ApplicationApiNode.request_shutdown(node)
    application_api_node.ApplicationApiNode.request_shutdown(node)
    node.application_controller.emergency_stop.assert_called_once_with("application_shutdown")


def test_capture_cleanup_continues_after_subscription_error_and_retries_only_failure(monkeypatch):
    listener = Mock(spec=['unregister'])
    monkeypatch.setattr(capture_module.tf2_ros, 'Buffer', Mock())
    monkeypatch.setattr(capture_module.tf2_ros, 'TransformListener', Mock(return_value=listener))
    node = Mock(spec=['create_subscription', 'destroy_subscription'])
    subscriptions = [object() for _ in range(4)]
    node.create_subscription.side_effect = subscriptions
    node.destroy_subscription.side_effect = [RuntimeError('subscription busy'), True, True, True, True]
    capture = ProbeReferenceCaptureCoordinator(node, Mock(), Mock(), Mock())
    capture._create_synchronizers(((0, 'hand'),))
    with pytest.raises(RuntimeError, match='subscription busy'):
        capture.close()
    assert node.destroy_subscription.call_count == 4
    listener.unregister.assert_called_once()
    capture.close()
    assert node.destroy_subscription.call_count == 5
    assert node.destroy_subscription.call_args.args[0] is subscriptions[0]
    listener.unregister.assert_called_once()
