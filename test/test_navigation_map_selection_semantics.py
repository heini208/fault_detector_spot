"""Tests for navigation map selection semantics."""

from pathlib import Path
from types import SimpleNamespace

from fault_detector_msgs.msg import NavigationSetupIntent
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.coordinators.navigation_setup_coordinator import (
    MODE_MAPPING,
    NavigationSetupCoordinator,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupCoordinator,
)
from fault_detector_spot.mapping.repository.map_artifact_store import (
    MapArtifactStore,
)
from fault_detector_spot.mapping.repository.map_repository import MapRepository


class FakeCommandController:
    def __init__(self):
        self.listeners = []
        self.submitted = []

    def add_status_listener(self, listener):
        self.listeners.append(listener)

    def remove_status_listener(self, listener):
        self.listeners.remove(listener)

    def submit(self, request):
        self.submitted.append(request)
        return request.request_id

    def cancel(self, request_id):
        return request_id


def _pose():
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.orientation.w = 1.0
    return pose


def _coordinator(tmp_path: Path):
    command_controller = FakeCommandController()
    setup = SetupCoordinator(command_controller)
    coordinator = NavigationSetupCoordinator(
        setup_coordinator=setup,
        map_repository=MapRepository(tmp_path),
        map_artifacts=MapArtifactStore(tmp_path),
        current_pose=_pose,
        visible_tag_pose=lambda _tag_id: _pose(),
    )
    return coordinator, command_controller


def test_select_map_while_runtime_is_stopped_is_not_a_robot_command(tmp_path):
    navigation, command_controller = _coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    created = navigation.create_map_definition(context, "plant")

    selected = navigation.select_map(created.context, "plant")

    assert selected.active_map == "plant"
    assert selected.map_names == ("plant",)
    assert command_controller.submitted == []


def test_create_and_select_map_makes_new_map_current(tmp_path):
    navigation, command_controller = _coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context

    state = navigation.create_and_select_map(context, "new_map")

    assert state.active_map == "new_map"
    assert state.map_names == ("new_map",)
    assert command_controller.submitted == []


def test_select_map_requires_runtime_to_be_stopped(tmp_path):
    navigation, _ = _coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    created = navigation.create_map_definition(context, "plant")
    navigation._mode = MODE_MAPPING

    try:
        navigation.select_map(created.context, "plant")
    except RuntimeError as exception:
        assert "Stop mapping or localization" in str(exception)
    else:
        raise AssertionError("Expected selection to reject active runtime")


def test_select_map_is_not_a_runtime_navigation_operation():
    from fault_detector_spot.application.api import navigation_setup_api

    assert (
        NavigationSetupIntent.OPERATION_SELECT_MAP
        not in navigation_setup_api._RUNTIME_OPERATIONS
    )
