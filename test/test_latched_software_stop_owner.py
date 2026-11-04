"""Software stop is a persistent admission latch, never a recovery sequence."""

from concurrent.futures import Future
from dataclasses import replace
from threading import Event, Thread
from types import SimpleNamespace
from unittest.mock import Mock, call

import pytest
from fault_detector_msgs.msg import CommandStatus
from py_trees.common import Status
from geometry_msgs.msg import PoseStamped, Twist

from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import (
    RobotCommandResources, _StopAwareActionClient,
)
from fault_detector_spot.application.behaviour_tree.behaviours.reset_software_stop import ResetSoftwareStop
from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.inspection.model.sensor_models import BARE_HAND_MOTION_ID
from test_command_owner_reconciliation import consumer, read_status, stop_publisher, stop_request, wire
from test_command_request_correlation import semantic_request
from test_execution_stop_evidence import resource_harness


def test_confirmed_stop_remains_latched_rejects_motion_and_repeated_stop_never_replays():
    owner = consumer()
    stop = stop_request()
    message = wire(stop, owner)
    owner.append_request_to_buffer(message)
    stop_publisher(owner, stop).update()
    for _ in range(3):
        owner.append_request_to_buffer(message)
    assert owner.status_ledger.stop_latched and owner.blackboard.estop_flag
    assert not owner.pending_msgs and not owner.blackboard.command_buffer
    status = read_status(owner, stop.request_id)
    assert status.software_stop_latched and status.software_stop_confirmed
    motion = semantic_request(CommandID.STAND_UP, "operator")
    owner.append_request_to_buffer(wire(motion, owner))
    assert read_status(owner, motion.request_id).state == CommandStatus.STATE_FAILED
    assert not owner.pending_msgs


def test_reset_requires_confirmed_ownership_and_preserves_old_request_fences():
    owner = consumer()
    delayed = semantic_request(CommandID.STAND_UP, "operator")
    stop = stop_request(delayed.request_id)
    owner.append_request_to_buffer(wire(stop, owner))
    reset = semantic_request(CommandID.RESET_SOFTWARE_STOP, "operator")
    reset = replace(reset, execution_instance_id=owner.status_ledger.execution_instance_id)
    owner.status_ledger.receive(reset)
    clear = Mock(side_effect=lambda clear_latch: (clear_latch() or True, "reset"))
    assert not owner.status_ledger.reset_stop(reset.request_id, clear)[0]
    clear.assert_not_called()
    stop_publisher(owner, stop).update()
    assert owner.status_ledger.reset_stop(reset.request_id, clear)[0]
    owner.append_request_to_buffer(wire(delayed, owner))
    assert not owner.pending_msgs
    assert read_status(owner, delayed.request_id).state == CommandStatus.STATE_STOP_CONFIRMED


def test_reset_rechecks_physical_stop_clears_buffer_and_performs_no_movement():
    ready = [False]
    resources = SimpleNamespace(
        begin_reset_confirmation=Mock(return_value=True),
        poll_stop_confirmation=lambda: (ready[0], "fresh physical stop"),
        reset_software_stop=Mock(side_effect=lambda clear: (clear() or True, "reset without replay")),
    )
    ledger = SimpleNamespace(reset_stop=lambda request_id, reset, clear_admission: reset(clear_admission))
    behavior = ResetSoftwareStop(resources, ledger)
    command = SimpleNamespace(request_id="reset")
    behavior.blackboard = SimpleNamespace(last_command=command, command_buffer=[object()], estop_flag=True)
    behavior.initialise()
    assert behavior.update() is Status.RUNNING
    assert behavior.blackboard.estop_flag
    resources.reset_software_stop.assert_not_called()
    ready[0] = True
    assert behavior.update() is Status.SUCCESS
    assert not behavior.blackboard.estop_flag and not behavior.blackboard.command_buffer
    resources.begin_reset_confirmation.assert_called_once()


def test_new_stop_after_atomic_reset_is_not_erased_by_old_reset_completion():
    owner = consumer()
    first_stop = stop_request()
    owner.append_request_to_buffer(wire(first_stop, owner))
    stop_publisher(owner, first_stop).update()
    reset_request = semantic_request(CommandID.RESET_SOFTWARE_STOP, "operator")
    owner.append_request_to_buffer(wire(reset_request, owner))
    owner.update()
    owner.blackboard.last_command = owner.blackboard.command_buffer.pop()
    newer_stop = stop_request(reset_request.request_id)

    def clear_then_receive_new_stop(clear):
        clear()
        # A new callback is delivered after the atomic clear but before the old
        # reset behavior has returned. It must own both latch and command buffer.
        owner.append_request_to_buffer(wire(newer_stop, owner))
        return True, "Previous software stop reset"

    resources = SimpleNamespace(
        begin_reset_confirmation=lambda: True,
        poll_stop_confirmation=lambda: (True, "fresh stop evidence"),
        reset_software_stop=clear_then_receive_new_stop,
    )
    behavior = ResetSoftwareStop(resources, owner.status_ledger)
    behavior.blackboard = owner.blackboard
    behavior.initialise()
    assert behavior.update() is Status.SUCCESS
    assert owner.status_ledger.stop_latched
    assert owner.blackboard.estop_flag
    assert [command.request_id for command in owner.blackboard.command_buffer] == [newer_stop.request_id]


def test_stop_callback_cancels_each_owner_before_a_tree_tick_or_resource_workflow_lock():
    resources = RobotCommandResources()
    arm, base, navigation = Mock(), Mock(), Mock()
    resources._arm_movement_executors[""] = arm
    resources._base_movement_executors[""] = base
    resources._waypoint_executor = navigation
    resources._velocity_publisher = Mock()
    velocity = Twist()
    velocity.linear.x = 0.2
    resources._receive_velocity(velocity)
    owner = consumer()
    owner.robot_command_resources = resources
    held, release = Event(), Event()

    def ordinary_workflow():
        with resources._lock:
            held.set()
            release.wait(2.0)

    worker = Thread(target=ordinary_workflow)
    worker.start()
    assert held.wait(1.0)
    try:
        stop = stop_request()
        owner.append_request_to_buffer(wire(stop, owner))
        assert resources.stop_latched and owner.blackboard.estop_flag
        arm.cancel.assert_called_once()
        base.begin_stop_confirmation.assert_called_once()
        navigation.cancel.assert_called_once()
        assert resources._velocity_publisher.publish.call_count == 1
        owner.append_request_to_buffer(wire(stop, owner))
        assert resources._velocity_publisher.publish.call_count == 1
        assert arm.cancel.call_count == base.begin_stop_confirmation.call_count == 1
    finally:
        release.set()
        worker.join(2.0)


def test_final_action_boundary_rejects_latched_motion_and_cancels_racing_late_acceptance():
    stopped = Event()
    future = Future()
    underlying = Mock()
    underlying.send_goal_async.return_value = future
    started = Mock()
    client = _StopAwareActionClient(underlying, stopped.is_set, started)
    assert client.send_goal_async(object()) is future
    stopped.set()
    handle = Mock(accepted=True)
    future.set_result(handle)
    handle.cancel_goal_async.assert_called_once()
    assert not client.send_goal_async(object()).result().accepted
    assert underlying.send_goal_async.call_count == 1
    started.assert_called_once()


@pytest.mark.parametrize("operator_stop_before_shutdown", [False, True])
def test_shutdown_rejects_new_goals_without_cancelling_late_acceptance(monkeypatch, operator_stop_before_shutdown):
    future = Future()
    underlying = Mock()
    underlying.send_goal_async.return_value = future
    monkeypatch.setattr(
        "fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources.ActionClientWrapper",
        lambda *_args, **_kwargs: underlying,
    )
    resources = RobotCommandResources()
    client = resources.get_action_client(object())
    assert client.send_goal_async(object()) is future
    if operator_stop_before_shutdown:
        resources.latch_software_stop()
    resources.begin_shutdown()
    handle = Mock(accepted=True)
    future.set_result(handle)
    handle.cancel_goal_async.assert_not_called()
    assert not client.send_goal_async(object()).result().accepted
    assert underlying.send_goal_async.call_count == 1


def test_shutdown_fences_inflight_action_submission_before_its_single_stop(monkeypatch):
    from test_arm_movement_executor import executor_with_client, FakeTransformer

    resources = RobotCommandResources()
    underlying = Mock()
    accepted = Future()
    entered, release, closed = Event(), Event(), Event()
    events = []
    def send(_goal):
        entered.set()
        release.wait(2.0)
        events.append("goal submitted")
        return accepted
    underlying.send_goal_async.side_effect = send
    monkeypatch.setattr(
        "fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources.ActionClientWrapper",
        lambda *_args, **_kwargs: underlying,
    )
    service = Mock()
    service.call_async.side_effect = lambda _request: (events.append("shutdown Stop"), Future())[1]
    arm, _ = executor_with_client(
        FakeTransformer(), arm_stop_service_client=service, submission_lock=resources._submission_lock,
    )
    arm._active = True
    resources._arm_movement_executors[""] = arm
    client = resources.get_action_client(object())
    worker = Thread(target=client.send_goal_async, args=(object(),))
    worker.start()
    assert entered.wait(1.0)
    closer = Thread(target=lambda: (resources.begin_shutdown(), closed.set()))
    closer.start()
    try:
        assert not closed.wait(0.05)
        service.call_async.assert_not_called()
    finally:
        release.set()
        worker.join(2.0)
        closer.join(2.0)
    assert closed.is_set()
    assert events == ["goal submitted", "shutdown Stop"]
    assert not client.send_goal_async(object()).result().accepted
    handle = Mock(accepted=True)
    accepted.set_result(handle)
    handle.cancel_goal_async.assert_not_called()
    underlying.send_goal_async.assert_called_once()
    resources.begin_shutdown()
    service.call_async.assert_called_once()


def test_velocity_stream_requires_quiet_period_even_when_zero_and_no_nav2_goal():
    now = [0.0]
    resources = RobotCommandResources(monotonic_clock=lambda: now[0])
    resources.request_software_stop()
    assert not resources.stop_sources_quiet()
    now[0] = 0.6
    assert resources.stop_sources_quiet()
    resources._receive_velocity(Twist())
    assert not resources.stop_sources_quiet()
    now[0] = 1.2
    assert resources.stop_sources_quiet()


def navigation_velocity_resources(monkeypatch):
    resources = RobotCommandResources()
    navigation = SimpleNamespace(active=False, close_admission=Mock(), request_cancel=Mock(), cancel=Mock())
    resources._waypoint_executor = navigation
    monkeypatch.setattr(resources, "get_arm_movement_executor", Mock())
    monkeypatch.setattr(resources, "get_base_movement_executor", Mock())
    monkeypatch.setattr(resources, "get_waypoint_navigation_executor", lambda _node: navigation)
    node = Mock()
    callbacks = {}
    node.create_subscription.side_effect = lambda _type, topic, callback, *_args, **_kwargs: callbacks.setdefault(topic, callback)
    resources.prepare_stop(node)
    return resources, navigation, node.create_publisher.return_value, callbacks[resources.NAVIGATION_VELOCITY_TOPIC]


def test_nav2_lifecycle_output_is_silent_but_owned_velocity_and_final_zero_are_forwarded(monkeypatch):
    resources, navigation, publisher, receive = navigation_velocity_resources(monkeypatch)
    velocity = Twist()
    velocity.linear.x = 0.2
    receive(Twist())
    receive(velocity)
    publisher.publish.assert_not_called()

    # Navigation retains active ownership through velocity release and fresh
    # stationary-base confirmation, so its final zero still reaches the driver.
    navigation.active = True
    receive(velocity)
    receive(Twist())
    assert publisher.publish.call_args_list == [call(velocity), call(Twist())]
    assert resources._last_velocity_command[1] is False
    navigation.active = False
    receive(velocity)
    assert publisher.publish.call_count == 2


def test_nav2_queued_output_is_dropped_on_shutdown_while_operator_stop_still_publishes_zero(monkeypatch):
    resources, navigation, publisher, receive = navigation_velocity_resources(monkeypatch)
    navigation.active = True
    velocity = Twist()
    velocity.linear.x = 0.2
    receive(velocity)
    resources.request_software_stop()
    assert publisher.publish.call_args_list == [call(velocity), call(Twist())]
    receive(velocity)
    receive(Twist())
    assert publisher.publish.call_count == 2
    resources.begin_shutdown()
    receive(velocity)
    receive(Twist())
    assert publisher.publish.call_count == 2


def test_nav2_shutdown_waits_only_for_inflight_publish_and_then_drops_queued_callbacks(monkeypatch):
    resources, navigation, publisher, receive = navigation_velocity_resources(monkeypatch)
    navigation.active = True
    publishing, release, closed = Event(), Event(), Event()
    publisher.publish.side_effect = lambda _message: (publishing.set(), release.wait(2.0))
    sender = Thread(target=receive, args=(Twist(),))
    sender.start()
    assert publishing.wait(1.0)
    closer = Thread(target=lambda: (resources.close_admission(), closed.set()))
    closer.start()
    try:
        assert not closed.wait(0.05)
    finally:
        release.set()
        sender.join(2.0)
        closer.join(2.0)
    assert closed.is_set()
    receive(Twist())
    resources.request_software_stop()
    publisher.publish.assert_called_once()


@pytest.mark.parametrize("stop_method", ["begin_shutdown", "request_software_stop"])
@pytest.mark.parametrize("stream", ["absent", "zero", "expired", "moving", "moving_navigation", "expired_navigation"])
def test_only_operator_stop_publishes_velocity_to_stop_a_live_motion_stream(stop_method, stream):
    now = [0.0]
    resources = RobotCommandResources(monotonic_clock=lambda: now[0])
    resources._velocity_publisher = Mock()
    if stream != "absent":
        velocity = Twist()
        velocity.linear.x = 0.0 if stream == "zero" else 0.2
        resources._receive_velocity(velocity)
    if stream in {"expired", "expired_navigation"}:
        now[0] = 1.0
    navigation_owned = stream in {"moving_navigation", "expired_navigation"}
    if navigation_owned:
        resources._waypoint_executor = SimpleNamespace(
            active=True, request_cancel=Mock(), cancel=Mock(), close_admission=Mock(),
            shutdown_stop_required=lambda: True,
        )

    getattr(resources, stop_method)()

    assert resources.stop_latched
    needs_source_stop = stop_method == "request_software_stop"
    if needs_source_stop:
        assert not resources.stop_sources_quiet()
    else:
        assert resources.poll_shutdown()[0] is not navigation_owned
    if needs_source_stop and stream in {"moving", "moving_navigation", "expired_navigation"}:
        resources._velocity_publisher.publish.assert_called_once_with(Twist())
    else:
        resources._velocity_publisher.publish.assert_not_called()
    now[0] += 0.6
    if needs_source_stop:
        assert resources.stop_sources_quiet()
    else:
        assert resources.poll_shutdown()[0] is not navigation_owned


@pytest.mark.parametrize("moving", [False, True])
def test_unowned_velocity_traffic_does_not_create_idle_shutdown_obligations(moving):
    from test_execution_stop_evidence import base_executor

    base, _, clock, source = base_executor()
    source.sample = Mock(side_effect=AssertionError("Idle shutdown must not read robot feedback"))
    resources = RobotCommandResources(monotonic_clock=clock)
    resources._base_movement_executors[""] = base
    resources._velocity_publisher = Mock()
    velocity = Twist()
    velocity.linear.x = 0.2 if moving else 0.0
    resources._receive_velocity(velocity)
    resources.begin_shutdown()
    base.begin_shutdown()
    for value in (0.1, 0.2, 0.3):
        clock.now = value
        resources._receive_velocity(velocity)
        assert resources.poll_shutdown()[0]
        assert base.poll_shutdown()[0]
    assert base.execution_idle
    resources._velocity_publisher.publish.assert_not_called()
    source.sample.assert_not_called()


@pytest.mark.parametrize("ownership", ["idle", "terminal", "arm", "base", "navigation", "uncertain"])
def test_shutdown_full_body_stop_depends_on_owned_motion_and_is_never_retried(ownership):
    from action_msgs.msg import GoalStatus
    from test_arm_movement_executor import executor_with_client, FakeTransformer
    from test_execution_stop_evidence import base_executor

    service = Mock()
    service.wait_for_service.return_value = True
    service.call_async.return_value = Future()
    arm, _ = executor_with_client(FakeTransformer(), arm_stop_service_client=service)
    base, _, _, _ = base_executor()
    resources = RobotCommandResources()
    resources._arm_movement_executors[""] = arm
    resources._base_movement_executors[""] = base
    resources._velocity_publisher = Mock()
    if ownership == "arm":
        arm._active = True
    elif ownership == "base":
        base._active = True
    elif ownership == "uncertain":
        arm._stop_unconfirmed = True
    elif ownership == "navigation":
        resources._waypoint_executor = SimpleNamespace(
            close_admission=Mock(), shutdown_stop_required=lambda: True,
        )
    elif ownership == "terminal":
        result, accepted = Future(), Future()
        result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
        accepted.set_result(SimpleNamespace(accepted=True, get_result_async=lambda: result))
        arm._submitted_goals = [[accepted, result]]
    required = ownership not in {"idle", "terminal"}
    for _ in range(3):
        resources.begin_shutdown()
        assert resources.poll_shutdown()[0] is not required
    assert service.call_async.call_count == (1 if required else 0)
    assert service.wait_for_service.call_count == (1 if required else 0)
    resources._velocity_publisher.publish.assert_not_called()
    if required:
        # Even a successful reply is diagnostic and never clears local evidence.
        service.call_async.return_value.set_result(SimpleNamespace(success=True, message="accepted"))
        assert not resources.poll_shutdown()[0]
        resources.begin_shutdown()
        service.call_async.assert_called_once()


def test_shutdown_submits_only_one_full_body_stop_for_each_robot_namespace():
    from test_arm_movement_executor import executor_with_client, FakeTransformer

    resources = RobotCommandResources()
    services = []
    for name in ("", "another_robot"):
        service = Mock()
        service.call_async.return_value = Future()
        arm, _ = executor_with_client(FakeTransformer(), arm_stop_service_client=service, robot_name=name)
        arm._stop_unconfirmed = True
        resources._arm_movement_executors[name] = arm
        resources._base_movement_executors[name] = SimpleNamespace(
            close_admission=Mock(), shutdown_stop_required=lambda: True,
        )
        services.append(service)
    resources.begin_shutdown()
    resources.begin_shutdown()
    for service in services:
        service.call_async.assert_called_once()
    assert not resources.poll_shutdown()[0]


@pytest.mark.parametrize("base_starts_first", [False, True])
def test_owned_navigation_velocity_does_not_trigger_shutdown_commands_or_base_proof(base_starts_first):
    from test_execution_stop_evidence import base_executor

    base, _, clock, source = base_executor()
    resources = RobotCommandResources(monotonic_clock=clock)
    resources._base_movement_executors[""] = base
    resources._waypoint_executor = SimpleNamespace(
        active=True, close_admission=Mock(), request_cancel=Mock(), cancel=Mock(),
        shutdown_stop_required=lambda: True,
    )
    resources._velocity_publisher = Mock()
    velocity = Twist()
    velocity.linear.x = 0.2
    resources._receive_velocity(velocity)
    if base_starts_first:
        base.begin_shutdown()
        assert base.poll_shutdown()[0]
    resources.begin_shutdown()
    base.begin_shutdown()
    resources._velocity_publisher.publish.assert_not_called()
    resources._waypoint_executor.request_cancel.assert_not_called()
    resources._waypoint_executor.cancel.assert_not_called()
    assert not resources.poll_shutdown()[0]
    assert base.poll_shutdown()[0]
    assert base.execution_idle
    resources._waypoint_executor.active = False
    clock.now = 0.6
    resources._receive_velocity(Twist())
    assert not resources.poll_shutdown()[0]
    assert base.poll_shutdown()[0]
    clock.now = 1.2
    assert not resources.poll_shutdown()[0]
    assert base.poll_shutdown()[0]


def recovery_node(arm):
    pose = PoseStamped()
    pose.header.stamp.sec = 1
    pose.pose.orientation.w = 1.0
    arm.probe_motion_planner.current_hand_pose.return_value = pose
    return SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)))


@pytest.mark.parametrize("failure", ["latched", "unconfirmed", "moving", "attachment", "collision", "pose", "stale_pose"])
def test_explicit_recovery_rejects_missing_current_prerequisites(monkeypatch, failure):
    resources, arm, base, _clock = resource_harness(monkeypatch)
    resources._stop_confirmed = True
    resources._stop_bases = (base,)
    node = recovery_node(arm)
    attachment = SimpleNamespace(active_attachment=lambda: (BARE_HAND_MOTION_ID, 4))
    collision = SimpleNamespace(state=lambda: SimpleNamespace(enabled=True))
    monkeypatch.setattr(resources, "get_probe_surface_source", lambda node: attachment)
    monkeypatch.setattr(resources, "get_arm_collision_control", lambda node: collision)
    if failure == "latched":
        resources._software_stop.set()
    elif failure == "unconfirmed":
        resources._stop_confirmed = False
    elif failure == "moving":
        base.poll_stop_confirmation.return_value = (False, "base moved")
    elif failure == "attachment":
        attachment.active_attachment = lambda: ("sensor-with-unproven-stow-clearance", 5)
    elif failure == "collision":
        collision.state = lambda: SimpleNamespace(enabled=False)
    elif failure == "pose":
        arm.probe_motion_planner.current_hand_pose.side_effect = ValueError("stale pose")
    else:
        arm.probe_motion_planner.current_hand_pose.return_value.header.stamp.sec = 0
    allowed, detail = resources.validate_recovery(node)
    assert not allowed and detail
    arm.stow.assert_not_called()


def test_recovery_validation_requires_bare_hand_current_pose_and_collision_policy(monkeypatch):
    resources, arm, base, _clock = resource_harness(monkeypatch)
    resources._stop_confirmed = True
    resources._stop_bases = (base,)
    node = recovery_node(arm)
    source = SimpleNamespace(active_attachment=lambda: (BARE_HAND_MOTION_ID, 4))
    collision = SimpleNamespace(state=lambda: SimpleNamespace(enabled=True))
    monkeypatch.setattr(resources, "get_probe_surface_source", lambda node: source)
    monkeypatch.setattr(resources, "get_arm_collision_control", lambda node: collision)
    assert resources.validate_recovery(node)[0]
    arm.probe_motion_planner.current_hand_pose.assert_called_once()
    arm.stow.assert_not_called()


def test_new_stop_during_reset_vetoes_final_clear(monkeypatch):
    resources, arm, base, _clock = resource_harness(monkeypatch)
    resources.request_software_stop()
    assert resources.poll_stop_confirmation()[0]
    assert resources.begin_reset_confirmation()
    assert resources.poll_stop_confirmation()[0]
    resources.latch_software_stop()
    clear = Mock()
    reset, detail = resources.reset_software_stop(clear)
    assert not reset and "newer" in detail
    clear.assert_not_called()
    assert resources.stop_latched


def test_rejected_late_plan_does_not_create_uncertain_goal_delivery():
    from test_execution_stop_evidence import base_executor
    owner, _sent, _clock, _source = base_executor()
    underlying = owner.action_client
    owner.action_client = _StopAwareActionClient(underlying, lambda: True, lambda: None)
    owner.sit()
    owner.poll()
    assert owner.goals_settled()
    assert underlying.send_calls == 0


def test_new_stop_latches_and_cancels_while_ledger_reset_lock_is_held():
    resources = RobotCommandResources()
    arm, base = Mock(), Mock()
    resources._arm_movement_executors[""] = arm
    resources._base_movement_executors[""] = base
    owner = consumer()
    owner.robot_command_resources = resources
    entered, cancelled = Event(), Event()
    arm.cancel.side_effect = cancelled.set
    stop = stop_request()
    message = wire(stop, owner)

    def deliver():
        entered.set()
        owner.append_request_to_buffer(message)

    with owner.status_ledger._lock:
        worker = Thread(target=deliver)
        worker.start()
        assert entered.wait(1.0)
        assert cancelled.wait(1.0)
        assert resources.stop_latched and owner.blackboard.estop_flag
        assert worker.is_alive()  # Evidence publication still waits for the ledger.
    worker.join(2.0)
    assert not worker.is_alive()
    generation = resources._stop_generation
    owner.append_request_to_buffer(message)
    assert resources._stop_generation == generation
    arm.cancel.assert_called_once()
    base.begin_stop_confirmation.assert_called_once()


def test_every_owner_gets_fast_cancellation_before_any_preparation_lock_wait():
    resources = RobotCommandResources()
    arm, base, navigation = Mock(), Mock(), Mock()
    resources._arm_movement_executors[""] = arm
    resources._base_movement_executors[""] = base
    resources._waypoint_executor = navigation
    blocked, release = Event(), Event()

    def busy_navigation_preparation():
        blocked.set()
        release.wait(2.0)

    navigation.cancel.side_effect = busy_navigation_preparation
    worker = Thread(target=resources.request_software_stop)
    worker.start()
    try:
        assert blocked.wait(1.0)
        assert resources.stop_latched
        for owner in (navigation, arm, base):
            owner.request_cancel.assert_called_once()
    finally:
        release.set()
        worker.join(2.0)


@pytest.mark.parametrize("owner_kind", ["arm", "navigation"])
def test_fast_owner_cancellation_does_not_wait_for_execution_lock_or_claim_idle(owner_kind):
    if owner_kind == "arm":
        from fault_detector_spot.manipulation.arm_movement_executor import ArmMovementExecutor
        owner = ArmMovementExecutor(object())
        owner._active = True
        lock = owner._execution_lock
    else:
        from test_waypoint_navigation_executor import rig
        owner, *_ = rig()
        lock = owner._lock
    handle = Mock(accepted=True)
    future = Future()
    future.set_result(handle)
    owner._send_goal_future = future
    owner._send_future = future
    requested = Event()
    handle.cancel_goal_async.side_effect = requested.set
    with lock:
        worker = Thread(target=owner.request_cancel)
        worker.start()
        assert requested.wait(1.0)
        worker.join(1.0)
        assert not worker.is_alive()
    handle.cancel_goal_async.assert_called_once()
    if owner_kind == "arm":
        assert owner.active and not owner.execution_idle


@pytest.mark.parametrize("physical_stop_confirmed", [False, True])
def test_request_error_is_reconciled_only_by_later_action_and_physical_proof(monkeypatch, physical_stop_confirmed):
    resources, arm, base, _clock = resource_harness(monkeypatch)
    arm.request_cancel.side_effect = RuntimeError("initial cancel request failed")
    base.poll_stop_confirmation.return_value = (physical_stop_confirmed, "base stop evidence")
    resources.request_software_stop()
    arm.cancel.assert_called_once()  # The ordinary owner still handles stopping.
    assert resources._stop_request_error
    assert resources.poll_stop_confirmation()[0] is physical_stop_confirmed
    assert bool(resources._stop_request_error) is not physical_stop_confirmed
    assert resources.stop_latched
