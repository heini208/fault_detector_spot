"""Map conversion must not starve the existing force listener or watchdog."""

from array import array
from threading import Event, Thread
from types import SimpleNamespace

from octomap_msgs.msg import Octomap
import pytest

import fault_detector_spot.manipulation.rtabmap_collision_scene as scene_module
from fault_detector_spot.manipulation.arm_movement_result import (
    ArmMovementOutcome, ArmMovementUpdate,
)
from test_guarded_probe_monitor import monitored, confirm_physical_stop
from test_map_collision_planning import Rig


@pytest.fixture
def converting_map(monkeypatch, monitored):
    m = monitored
    rig = Rig()
    entered, release, converted, tick_finished = Event(), Event(), Event(), Event()
    errors = []
    convert = scene_module.snapshot_scene_diff

    def gated_conversion(snapshot, transform):
        entered.set()
        try:
            if not release.wait(5.0):
                raise RuntimeError("Test did not release map conversion")
            return convert(snapshot, transform)
        finally:
            converted.set()

    monkeypatch.setattr(scene_module, "snapshot_scene_diff", gated_conversion)
    rig.planner.start(rig.target)
    snapshot = Octomap()
    snapshot.header.frame_id = "map"
    snapshot.id = "ColorOcTree"
    snapshot.binary = True
    snapshot.resolution = 0.05
    snapshot.data = array("b", [2, 0])
    rig.node.clients["/rtabmap/octomap_binary"].future.set_result(
        SimpleNamespace(map=snapshot),
    )

    def poll_planner():
        update = rig.planner.poll()
        return ArmMovementUpdate(ArmMovementOutcome(update.outcome.value), update.detail)

    def cancel_planner():
        m.driver.cancel()
        rig.planner.cancel()

    # Use the real monitor -> guard -> planning path while its safety lock is held.
    m.guard._poll_goal = poll_planner
    m.guard._cancel_goal = cancel_planner

    def tick():
        try:
            m.node.timer.callback()
        except Exception as exception:
            errors.append(exception)
        finally:
            tick_finished.set()

    timer_thread = Thread(target=tick, daemon=True)
    timer_thread.start()
    try:
        assert entered.wait(2.0), "Planner did not begin map conversion"
        assert tick_finished.wait(1.0), "Map conversion blocked the guard timer"
        assert not errors
        assert rig.planner._planning_mode == "prepare_map"
        yield SimpleNamespace(monitor=m, rig=rig, release=release)
    finally:
        release.set()
        timer_thread.join(2.0)
        if entered.is_set():
            assert converted.wait(2.0)
        rig.planner.destroy()


def test_fresh_force_is_processed_while_map_conversion_is_pending(converting_map):
    m, rig = converting_map.monitor, converting_map.rig
    for received_at in (0.1, 0.2, 0.3, 0.4):
        m.emit(received_at, force=(1.0, 2.0, 3.0))
        m.node.timer.callback()
        assert m.guard._force_last_received_at == pytest.approx(received_at)
    assert m.clock.now > m.guard.force_stale_timeout_sec
    assert not converting_map.release.is_set()
    assert not rig.planner._future.done()
    assert m.driver.stop_count == m.driver.cancel_count == 0
    assert m.monitor.poll().outcome is ArmMovementOutcome.RUNNING


def test_missing_force_still_stops_during_pending_map_conversion(converting_map):
    m = converting_map.monitor
    m.clock.now = m.guard.force_stale_timeout_sec
    m.node.timer.callback()
    assert m.driver.cancel_count == m.driver.stop_count == 1
    assert not converting_map.release.is_set()

    m.driver.stop_updates.append(ArmMovementUpdate(ArmMovementOutcome.SUCCESS, "accepted"))
    m.node.timer.callback()
    confirm_physical_stop(m, 0.3)
    assert m.monitor.poll().outcome is ArmMovementOutcome.FORCE_STALE
    assert not m.guard.active
