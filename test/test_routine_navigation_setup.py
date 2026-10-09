"""Routine navigation assignments cross setup, persistence, and ROS state."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from fault_detector_msgs.msg import ProbeSetupIntent, ProbeSetupState
from fault_detector_msgs.srv import ExecuteProbeSetup
from fault_detector_spot.application.api.probe_setup_api import ProbeSetupApi
from fault_detector_spot.inspection.setup.probe_setup_state_adapter import ProbeSetupStateAdapter
from fault_detector_spot.mapping.model.models import Waypoint
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.shared.geometry.models import PoseData
from test_command_request_correlation import FakeClock
from test_probe_setup_coordinator import coordinator, create_selected_routine


@pytest.fixture
def navigation_setup(tmp_path):
    probe, commands = coordinator(tmp_path)
    probe.map_repository = MapRepository(tmp_path / "maps")
    for map_id, waypoint_id in (("factory", "inspection"), ("workshop", "bench")):
        probe.map_repository.create_empty(map_id)
        probe.map_repository.add_waypoint(
            map_id, Waypoint(waypoint_id, waypoint_id, PoseData.identity()),
        )
    state = create_selected_routine(probe, probe.open_context("probe-ui").context)
    api = ProbeSetupApi.__new__(ProbeSetupApi)
    api.coordinator = probe
    api.state_adapter = ProbeSetupStateAdapter(FakeClock())
    api.state_publisher = Mock()
    api._handlers = api._transaction_handlers()
    return probe, commands, api, state.context


def execute(api, context, operation, **fields):
    intent = ProbeSetupIntent(
        operation=operation, object_id="motor", routine_id="magnetic_scan",
    )
    for name, value in fields.items():
        setattr(intent, name, value)
    request = ExecuteProbeSetup.Request(
        client_id=context.client_id, context_id=context.context_id, intent=intent,
    )
    return api._execute(request, ExecuteProbeSetup.Response()).state


def assign_map(api, context, map_name="factory", **fields):
    return execute(
        api, context, ProbeSetupIntent.OPERATION_SAVE_ROUTINE_MAP,
        map_name=map_name, **fields,
    )


def assign_waypoint(api, context, map_name="factory", waypoint_name="inspection"):
    return execute(
        api, context, ProbeSetupIntent.OPERATION_SAVE_ROUTINE_WAYPOINT,
        map_name=map_name, waypoint_name=waypoint_name,
    )


def test_assignment_persists_and_publishes_map_specific_choices_without_movement(navigation_setup):
    probe, commands, api, context = navigation_setup
    commands.active = "another-command"
    commands.queued = ("queued-command",)

    mapped = assign_map(api, context)
    assigned = assign_waypoint(api, context)
    reopened = probe.select_routine(
        probe.open_context("reopened-ui").context, "motor", "magnetic_scan",
    )

    assert mapped.state == assigned.state == ProbeSetupState.STATE_SUCCEEDED
    assert list(mapped.navigation_map_names) == ["factory", "workshop"]
    assert list(mapped.routine_waypoint_names) == ["inspection"]
    assert mapped.routine_map_name == "factory"
    assert assigned.routine_waypoint_name == "inspection"
    assert (reopened.routine_map_name, reopened.routine_waypoint_name) == ("factory", "inspection")
    assert api.state_publisher.publish.call_count == 2
    assert commands.submitted == []


def test_changing_map_clears_waypoint_and_rejects_stale_waypoint_save(navigation_setup):
    probe, _, api, context = navigation_setup
    assign_map(api, context)
    assign_waypoint(api, context)

    changed = assign_map(api, context, "workshop")
    stale = assign_waypoint(api, context)

    assert changed.state == ProbeSetupState.STATE_SUCCEEDED
    assert changed.routine_map_name == "workshop"
    assert changed.routine_waypoint_name == ""
    assert list(changed.routine_waypoint_names) == ["bench"]
    assert stale.state == ProbeSetupState.STATE_FAILED
    assert "routine map changed" in stale.detail
    assert probe.object_repository.load("motor").get_routine("magnetic_scan").waypoint_id == ""


def test_same_map_preserves_waypoint_and_clear_operations_persist(navigation_setup):
    probe, _, api, context = navigation_setup
    assign_map(api, context)
    assign_waypoint(api, context)
    assert assign_map(api, context).routine_waypoint_name == "inspection"

    cleared_waypoint = assign_waypoint(api, context, waypoint_name="")
    assert cleared_waypoint.state == ProbeSetupState.STATE_SUCCEEDED
    assert cleared_waypoint.routine_map_name == "factory"
    assert cleared_waypoint.routine_waypoint_name == ""
    assign_waypoint(api, context)
    cleared_map = assign_map(api, context, "")
    assert cleared_map.state == ProbeSetupState.STATE_SUCCEEDED
    assert cleared_map.routine_map_name == cleared_map.routine_waypoint_name == ""
    assert list(cleared_map.routine_waypoint_names) == []
    routine = probe.object_repository.load("motor").get_routine("magnetic_scan")
    assert routine.map_id == routine.waypoint_id == ""


@pytest.mark.parametrize("operation, fields, detail", [
    (ProbeSetupIntent.OPERATION_SAVE_ROUTINE_MAP, {"map_name": "missing"}, "does not exist"),
    (ProbeSetupIntent.OPERATION_SAVE_ROUTINE_MAP, {"map_name": "workshop", "routine_id": "other"}, "selected routine changed"),
    (ProbeSetupIntent.OPERATION_SAVE_ROUTINE_WAYPOINT, {"map_name": "factory", "waypoint_name": "bench"}, "does not exist in map"),
])
def test_invalid_assignment_leaves_persisted_routine_unchanged(navigation_setup, operation, fields, detail):
    probe, _, api, context = navigation_setup
    assign_map(api, context)
    assign_waypoint(api, context)
    path = probe.object_repository.get_object_path("motor")
    before = path.read_text()

    result = execute(api, context, operation, **fields)

    assert result.state == ProbeSetupState.STATE_FAILED
    assert detail in result.detail
    assert path.read_text() == before
    assert result.routine_map_name == "factory"
    assert result.routine_waypoint_name == "inspection"


def test_waypoint_requires_saved_map(navigation_setup):
    _, _, api, context = navigation_setup

    result = assign_waypoint(api, context, map_name="")

    assert result.state == ProbeSetupState.STATE_FAILED
    assert "Set a routine map" in result.detail
    assert result.routine_waypoint_name == ""


def test_navigation_assignment_rejects_open_refinement(navigation_setup):
    probe, _, _, context = navigation_setup
    draft = probe._selected_draft(context)
    draft.refinement = SimpleNamespace(recovery_required=False)

    with pytest.raises(RuntimeError, match="Close probe refinement"):
        probe.save_routine_map(context, "motor", "magnetic_scan", "factory")

    assert probe.object_repository.load("motor").get_routine("magnetic_scan").map_id == ""


@pytest.mark.parametrize("damage", [None, "{", "{}"])
def test_unavailable_map_association_can_be_seen_and_cleared(navigation_setup, damage):
    probe, _, api, context = navigation_setup
    assign_map(api, context)
    assign_waypoint(api, context)
    path = probe.map_repository.get_map_path("factory")
    if damage is None:
        path.unlink()
    else:
        path.write_text(damage)

    state = probe.snapshot(probe.context(context.context_id, context.client_id))

    assert state.routine_map_name == "factory"
    assert state.routine_waypoint_name == "inspection"
    assert state.navigation_map_names == (("workshop",) if damage is None else ("factory", "workshop"))
    assert state.routine_waypoint_names == ()
    assert assign_map(api, context, "").state == ProbeSetupState.STATE_SUCCEEDED
