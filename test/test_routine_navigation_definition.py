"""Routine navigation associations persist and cross the setup state boundary."""

from dataclasses import replace

import pytest
from fault_detector_msgs.msg import ProbeSetupState

from fault_detector_spot.application.commanding.command_request import CommandOrigin
from fault_detector_spot.application.setup.setup_context import SetupContextSnapshot
from fault_detector_spot.inspection.model.models import (
    InspectionObject,
    InspectionRoutine,
    ReferenceTag,
)
from fault_detector_spot.inspection.repository.object_repository import ObjectRepository
from fault_detector_spot.inspection.setup.probe_definition_service import ProbeDefinitionService
from fault_detector_spot.inspection.setup.probe_setup_context import (
    ProbeSetupDraft,
    ProbeSetupSnapshot,
)
from fault_detector_spot.inspection.setup.probe_setup_state_adapter import ProbeSetupStateAdapter
from fault_detector_spot.shared.geometry.models import PoseData
from test_command_request_correlation import FakeClock


def routine():
    return InspectionRoutine(
        routine_id="scan",
        display_name="Scan",
        reference_tag=ReferenceTag(tag_id=7, tag_family="36h11"),
        base_position=PoseData.identity(),
        base_body_height_m=-0.1,
    )


def test_legacy_routine_defaults_to_no_navigation_association():
    serialized = routine().to_dict()
    assert "map_id" not in serialized
    assert "waypoint_id" not in serialized

    restored = InspectionRoutine.from_dict(serialized)
    restored.validate()

    assert restored.map_id == ""
    assert restored.waypoint_id == ""
    assert restored.base_body_height_m == -0.1


@pytest.mark.parametrize("waypoint_id", ["", "motor_front", "motor/front"])
def test_routine_navigation_association_round_trips(waypoint_id):
    original = replace(routine(), map_id="laboratory", waypoint_id=waypoint_id)
    restored = InspectionRoutine.from_dict(original.to_dict())
    restored.validate()

    assert restored == original
    assert restored.to_dict()["map_id"] == "laboratory"
    if waypoint_id:
        assert restored.to_dict()["waypoint_id"] == waypoint_id


@pytest.mark.parametrize("field_name", ["map_id", "waypoint_id"])
@pytest.mark.parametrize("value", [None, 7, " ", " padded "])
def test_routine_navigation_rejects_invalid_identifiers(field_name, value):
    serialized = replace(routine(), map_id="laboratory").to_dict()
    serialized[field_name] = value
    restored = InspectionRoutine.from_dict(serialized)

    with pytest.raises((TypeError, ValueError), match="Routine (map|waypoint) ID"):
        restored.validate()


@pytest.mark.parametrize("map_id", ["../outside", "..", "map/submap"])
def test_routine_map_rejects_invalid_storage_ids(map_id):
    original = replace(routine(), map_id=map_id)

    with pytest.raises(ValueError, match="Routine map ID"):
        original.validate()


def test_routine_waypoint_requires_a_map():
    original = replace(routine(), waypoint_id="motor_front")

    with pytest.raises(ValueError, match="waypoint requires a map"):
        original.validate()


@pytest.fixture
def repository(tmp_path):
    objects = ObjectRepository(tmp_path / "objects")
    objects.save(InspectionObject(
        object_id="motor",
        display_name="Motor",
        routines=[routine(), replace(routine(), routine_id="other_scan")],
    ))
    return objects


def test_definition_service_saves_navigation_and_only_map_changes_clear_waypoint(repository):
    definitions = ProbeDefinitionService(repository)
    assert definitions.set_routine_map("motor", "scan", "laboratory") == (
        "motor", "scan",
    )
    assert definitions.set_routine_waypoint("motor", "scan", "motor_front") == (
        "motor", "scan",
    )
    restored = repository.load("motor")
    selected = restored.get_routine("scan")
    assert selected.map_id == "laboratory"
    assert selected.waypoint_id == "motor_front"
    assert selected.base_position == routine().base_position
    assert selected.base_body_height_m == -0.1
    assert restored.get_routine("other_scan").map_id == ""

    repository.set_routine_map("motor", "scan", "laboratory")
    assert repository.load("motor").get_routine("scan").waypoint_id == "motor_front"

    repository.set_routine_map("motor", "scan", "workshop")
    selected = repository.load("motor").get_routine("scan")
    assert selected.map_id == "workshop"
    assert selected.waypoint_id == ""

    repository.set_routine_waypoint("motor", "scan", "motor_back")
    repository.set_routine_waypoint("motor", "scan", "")
    selected = repository.load("motor").get_routine("scan")
    assert selected.map_id == "workshop"
    assert selected.waypoint_id == ""

    repository.set_routine_waypoint("motor", "scan", "motor_back")
    repository.set_routine_map("motor", "scan", "")
    selected = repository.load("motor").get_routine("scan")
    assert selected.map_id == ""
    assert selected.waypoint_id == ""


@pytest.mark.parametrize(
    "setter,value,error",
    [
        ("set_routine_map", "../outside", "Routine map ID"),
        ("set_routine_waypoint", "motor_front", "waypoint requires a map"),
        ("set_routine_waypoint", None, "Routine waypoint ID"),
    ],
)
def test_invalid_navigation_update_preserves_repository(repository, setter, value, error):
    path = repository.get_object_path("motor")
    original = path.read_text()

    with pytest.raises((TypeError, ValueError), match=error):
        getattr(repository, setter)("motor", "scan", value)

    assert path.read_text() == original


@pytest.mark.parametrize("setter", ["set_routine_map", "set_routine_waypoint"])
def test_navigation_update_rejects_unknown_routine(repository, setter):
    with pytest.raises(KeyError, match="routine does not exist"):
        getattr(repository, setter)("motor", "missing", "laboratory")


def snapshot(**navigation):
    context = SetupContextSnapshot(
        context_id="11111111-1111-4111-8111-111111111111",
        client_id="probe-ui",
        origin=CommandOrigin.PROBE_SETUP,
        revision=0,
    )
    return ProbeSetupSnapshot.from_draft(
        draft=ProbeSetupDraft(context),
        object_ids=(),
        routine_ids=(),
        reference_view_ids=(),
        reference_camera_ids=(),
        selected_reference_tag_id=-1,
        selected_reference_tag_family="",
        probe_point_ids=(),
        **navigation,
    )


def test_setup_snapshot_defaults_to_empty_navigation_fields():
    state = snapshot()
    message = ProbeSetupStateAdapter(FakeClock()).message(
        state, 0, ProbeSetupState.STATE_READY, "Ready",
    )

    assert message.routine_map_name == ""
    assert message.routine_waypoint_name == ""
    assert message.navigation_map_names == []
    assert message.routine_waypoint_names == []


def test_setup_state_preserves_navigation_selection_and_isolated_lists():
    maps = ["laboratory", "workshop"]
    waypoints = ["motor_front", "motor_back"]
    state = snapshot(
        routine_map_name="laboratory",
        routine_waypoint_name="motor_front",
        navigation_map_names=maps,
        routine_waypoint_names=waypoints,
    )
    maps.clear()
    waypoints.clear()
    message = ProbeSetupStateAdapter(FakeClock()).message(
        state, 0, ProbeSetupState.STATE_READY, "Ready",
    )

    assert message.routine_map_name == "laboratory"
    assert message.routine_waypoint_name == "motor_front"
    assert message.navigation_map_names == ["laboratory", "workshop"]
    assert message.routine_waypoint_names == ["motor_front", "motor_back"]
    assert state.navigation_map_names == ("laboratory", "workshop")
    assert state.routine_waypoint_names == ("motor_front", "motor_back")
