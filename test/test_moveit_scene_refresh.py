"""Idle scene refresh never requests motion or steals a command's polling."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.clock import ClockType

from fault_detector_spot.manipulation.moveit_arm_planner import (
    MoveItPlanOutcome, MoveItPlanUpdate,
)
from fault_detector_spot.manipulation.moveit_scene_refresh import MoveItSceneRefresh

from test_moveit_collision_planning import CARTESIAN, PLAN
from test_moveit_sensor_head import attachment_response, mount, robot_description
from test_moveit_sensor_head_planning import head_rig


@pytest.fixture
def rig():
    node = Mock()
    planner = Mock(active=False, scene_sync_active=False)
    planner.start_scene_sync.return_value = MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, "Started")
    planner.poll.return_value = MoveItPlanUpdate(MoveItPlanOutcome.RUNNING, "Waiting")
    state = SimpleNamespace(now=0., idle=True, needs_scene_sync=True)
    refresh = MoveItSceneRefresh(node, planner, state, lambda: state.idle, lambda: state.now)
    yield SimpleNamespace(node=node, planner=planner, state=state, refresh=refresh)
    refresh.destroy()


def test_timer_starts_scene_only_work_on_a_steady_clock(rig):
    args, kwargs = rig.node.create_timer.call_args
    assert args == (0.2, rig.refresh._tick)
    assert kwargs["clock"].clock_type == ClockType.STEADY_TIME
    assert "callback_group" not in kwargs  # Same exclusive group as tree.tick.
    rig.refresh._tick()
    rig.planner.start_scene_sync.assert_called_once_with()
    rig.planner.poll.assert_not_called()
    rig.planner.start.assert_not_called()
    rig.planner.start_cartesian.assert_not_called()


@pytest.mark.parametrize("reason", ["arm_busy", "planning", "synchronized"])
def test_no_refresh_when_an_arm_command_is_active_or_head_is_current(rig, reason):
    if reason == "arm_busy":
        rig.state.idle = False
    elif reason == "planning":
        rig.planner.active = True
    else:
        rig.state.needs_scene_sync = False
    rig.refresh._tick()
    rig.planner.start_scene_sync.assert_not_called()
    rig.planner.poll.assert_not_called()


def test_timer_polls_its_sync_but_stops_immediately_when_motion_takes_over(rig):
    rig.planner.active = True
    rig.planner.scene_sync_active = True
    rig.refresh._tick()
    rig.planner.poll.assert_called_once_with()
    rig.planner.scene_sync_active = False
    rig.refresh._tick()
    rig.planner.poll.assert_called_once_with()
    rig.planner.start_scene_sync.assert_not_called()


def test_busy_arm_pauses_idle_refresh_polling(rig):
    rig.planner.scene_sync_active = True
    rig.planner.active = True
    rig.state.idle = False
    rig.refresh._tick()
    rig.planner.poll.assert_not_called()
    rig.state.idle = True
    rig.refresh._tick()
    rig.planner.poll.assert_called_once_with()


def test_failed_service_retries_with_backoff_and_reports_unchanged_error_once(rig):
    rig.planner.start_scene_sync.return_value = MoveItPlanUpdate(
        MoveItPlanOutcome.SERVICE_UNAVAILABLE, "MoveIt is unavailable",
    )
    rig.refresh._tick()
    rig.refresh._tick()
    assert rig.planner.start_scene_sync.call_count == 1
    rig.state.now = 2.
    rig.refresh._tick()
    assert rig.planner.start_scene_sync.call_count == 2
    rig.node.get_logger.return_value.warning.assert_called_once()


def test_verified_sync_finishes_once_and_a_new_confirmation_can_refresh(rig):
    rig.planner.active = True
    rig.planner.scene_sync_active = True
    rig.planner.poll.return_value = MoveItPlanUpdate(MoveItPlanOutcome.SUCCESS, "Verified")
    rig.refresh._tick()
    rig.state.needs_scene_sync = False
    rig.planner.active = False
    rig.planner.scene_sync_active = False
    rig.refresh._tick()
    rig.planner.start_scene_sync.assert_not_called()
    rig.state.needs_scene_sync = True
    rig.refresh._tick()
    rig.planner.start_scene_sync.assert_called_once_with()


def test_destroyed_timer_callback_cannot_start_work(rig):
    rig.refresh.destroy()
    rig.refresh.destroy()
    rig.refresh._tick()
    rig.node.destroy_timer.assert_called_once_with(rig.node.create_timer.return_value)
    rig.planner.start_scene_sync.assert_not_called()
    rig.planner.poll.assert_not_called()


def test_confirmation_refreshes_real_scene_then_restores_stock_without_any_motion(head_rig):
    rig = head_rig
    rig.node.create_timer = Mock()
    rig.node.destroy_timer = Mock()
    refresh = MoveItSceneRefresh(rig.node, rig.planner, rig.head, lambda: True)

    def reply_on_timer(service, response):
        rig.node.clients[service].future.set_result(response)
        refresh._tick()

    rig.reply = reply_on_timer
    try:
        for registry, mesh_count in (
            (rig.registry, 1), (attachment_response(None, "", revision=4), 3),
        ):
            rig.registry = registry
            rig.head._observe_state(registry.state)
            refresh._tick()
            assert rig.planner.scene_sync_active
            while rig.planner.active:
                rig.step()
            assert not rig.head.needs_scene_sync
            assert len(rig.server_scene.robot_state.attached_collision_objects[-1].object.meshes) == mesh_count
            events = list(rig.node.events)
            refresh._tick()
            assert rig.node.events == events
        assert not rig.node.clients[PLAN].requests
        assert not rig.node.clients[CARTESIAN].requests
    finally:
        refresh.destroy()
