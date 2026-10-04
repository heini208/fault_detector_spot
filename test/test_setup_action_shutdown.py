"""Setup waits must finish without ROS feedback during executor shutdown."""

from concurrent.futures import ThreadPoolExecutor
from threading import Event, RLock
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_spot.application.api import application_api_node
from fault_detector_spot.application.api.navigation_setup_api import NavigationSetupApi
from fault_detector_spot.application.api.probe_setup_motion_api import ProbeSetupMotionApi
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
    api._executions = {}
    api._early_states = {}
    api.coordinator = Mock()
    api._abort = Mock(return_value="aborted")
    api._motion_request = Mock()
    entered = Event()
    operation = SimpleNamespace(request_id="request")
    goal = Mock(is_cancel_requested=False)
    goal.request.intent.operation = 1

    if api_type is NavigationSetupApi:
        def submit(**kwargs):
            entered.set()
            return operation
        api.coordinator.submit_runtime_operation.side_effect = submit
        # START_MAPPING is a runtime operation supported by this API.
        from fault_detector_msgs.msg import NavigationSetupIntent
        goal.request.intent.operation = NavigationSetupIntent.OPERATION_START_MAPPING
        call = lambda: api._execute_runtime(goal, object(), goal.request.intent)
    else:
        api.coordinator.prepare_motion.return_value = operation
        api.coordinator.submit_motion.side_effect = lambda op: entered.set()
        call = lambda: api._execute(goal)

    with ThreadPoolExecutor(max_workers=1) as worker:
        future = worker.submit(call)
        try:
            assert entered.wait(1)
            api.request_shutdown()
            assert future.result(timeout=1) == "aborted"
            assert not api._executions
            assert "shutdown" in api._abort.call_args.args[2].lower()
        finally:
            api.request_shutdown()


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


def test_capture_signal_preserves_resources_until_callback_drain():
    capture = ProbeReferenceCaptureCoordinator.__new__(ProbeReferenceCaptureCoordinator)
    capture._shutdown = Event()
    capture.tf_listener = Mock()
    capture.request_shutdown()
    with pytest.raises(ReferenceCaptureCancelled, match="shutdown"):
        capture._check_cancel(lambda: False)
    capture.tf_listener.unregister.assert_not_called()
    capture.close()
    capture.tf_listener.unregister.assert_called_once()


def test_main_signals_before_drain_and_destroys_afterward(monkeypatch):
    events = []
    node = Mock()
    node.request_shutdown.side_effect = lambda: events.append("signal")
    node.destroy_node.side_effect = lambda: events.append("destroy")
    executor = Mock()
    executor.spin.side_effect = KeyboardInterrupt
    executor.shutdown.side_effect = lambda: events.append("drain")
    monkeypatch.setattr(application_api_node, "ApplicationApiNode", lambda: node)
    monkeypatch.setattr(application_api_node, "MultiThreadedExecutor", lambda **kw: executor)
    monkeypatch.setattr(application_api_node.rclpy, "init", Mock())
    monkeypatch.setattr(application_api_node.rclpy, "try_shutdown", lambda: events.append("ros"))
    application_api_node.main()
    assert events == ["signal", "drain", "destroy", "ros"]


def test_node_signals_every_blocking_setup_owner():
    owners = {
        name: Mock() for name in (
            "navigation_setup_api", "probe_setup_motion_api",
            "probe_reference_capture_api", "probe_refinement_finalization_api",
        )
    }
    application_api_node.ApplicationApiNode.request_shutdown(SimpleNamespace(**owners))
    for owner in owners.values():
        owner.request_shutdown.assert_called_once_with()
        owner.close.assert_not_called()
