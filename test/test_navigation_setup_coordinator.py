"""Tests for navigation setup ownership and persistence."""

from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
    RecordingPolicy,
)
from fault_detector_spot.application.commanding.semantic_command import (
    SemanticCommand,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
    CommandControllerStatus,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupCoordinator,
)
from fault_detector_spot.mapping.repository.map_artifact_store import (
    MapArtifactStore,
)
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.application.coordinators.navigation_setup_coordinator import (
    MODE_MAPPING,
    NavigationSetupCoordinator,
)


class FakeCommandController:
    def __init__(self):
        self.listeners = []
        self.submitted = []
        self.cancelled = []

    def add_status_listener(self, listener):
        self.listeners.append(listener)

    def remove_status_listener(self, listener):
        self.listeners.remove(listener)

    def submit(self, request):
        self.submitted.append(request)
        return request.request_id

    def cancel(self, request_id):
        self.cancelled.append(request_id)
        return request_id

    def emit(self, request, state, detail=""):
        status = CommandControllerStatus(
            request=request,
            state=state,
            detail=detail,
        )
        for listener in tuple(self.listeners):
            listener(status)


def map_pose(x=1.0):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.orientation.w = 1.0
    return pose


def coordinator(tmp_path: Path):
    command_controller = FakeCommandController()
    shared = SetupCoordinator(command_controller)
    navigation = NavigationSetupCoordinator(
        setup_coordinator=shared,
        map_repository=MapRepository(tmp_path),
        map_artifacts=MapArtifactStore(tmp_path),
        current_pose=lambda: map_pose(),
        visible_tag_pose={7: map_pose(7.0)}.get,
    )
    return navigation, command_controller


def activate_mapping(navigation, boundary, context, map_name):
    statuses = []
    navigation.add_status_listener(statuses.append)
    operation = navigation.submit_runtime_operation(
        context,
        operation_code=5,
        command_id=CommandID.START_SLAM,
        map_name=map_name,
    )
    boundary.emit(
        operation.request,
        CommandControllerState.SUCCEEDED,
    )
    return statuses[-1].snapshot.context


def test_repository_transactions_stay_outside_command_lane(tmp_path):
    navigation, boundary = coordinator(tmp_path)
    state = navigation.open_context("navigation-ui")
    state = navigation.create_map_definition(
        state.context,
        "plant",
    )
    navigation.observe_active_map("plant")
    context = activate_mapping(
        navigation,
        boundary,
        state.context,
        "plant",
    )

    waypoint = navigation.add_current_waypoint(
        context,
        "plant",
        "motor_front",
    )
    landmark = navigation.add_visible_tag_landmark(
        waypoint.context,
        "plant",
        7,
    )

    definition = navigation.map_repository.load("plant")
    assert [item.waypoint_id for item in definition.waypoints] == [
        "motor_front"
    ]
    assert [item.landmark_id for item in definition.localization_landmarks] == [
        "Tag_7"
    ]
    assert landmark.waypoint_names == ("motor_front",)


def test_runtime_operation_uses_semantic_nonrecordable_command(tmp_path):
    navigation, boundary = coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    context = navigation.create_map_definition(
        context,
        "plant",
    ).context
    navigation.observe_active_map("plant")
    statuses = []
    navigation.add_status_listener(statuses.append)

    operation = navigation.submit_runtime_operation(
        context,
        operation_code=5,
        command_id=CommandID.START_SLAM,
        map_name="plant",
    )

    assert boundary.submitted == [operation.request]
    assert operation.request.origin is CommandOrigin.NAVIGATION_SETUP
    assert operation.request.recording_policy is RecordingPolicy.EXCLUDE
    assert isinstance(operation.request.command, SemanticCommand)
    assert operation.request.command.command_id is CommandID.START_SLAM
    assert operation.request.command.map_name == "plant"

    boundary.emit(
        operation.request,
        CommandControllerState.SUCCEEDED,
        "Mapping started",
    )

    assert statuses[-1].snapshot.mode == MODE_MAPPING
    assert statuses[-1].detail == "Mapping started"


def test_pose_authoring_requires_active_runtime_mode(tmp_path):
    navigation, _ = coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    context = navigation.create_map_definition(
        context,
        "plant",
    ).context
    navigation.observe_active_map("plant")

    with pytest.raises(RuntimeError, match="Mapping or localization"):
        navigation.add_current_waypoint(
            context,
            "plant",
            "motor_front",
        )


def test_duplicate_waypoint_and_missing_tag_fail(tmp_path):
    navigation, boundary = coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    context = navigation.create_map_definition(
        context,
        "plant",
    ).context
    navigation.observe_active_map("plant")
    context = activate_mapping(
        navigation,
        boundary,
        context,
        "plant",
    )
    context = navigation.add_current_waypoint(
        context,
        "plant",
        "motor_front",
    ).context

    with pytest.raises(FileExistsError, match="Waypoint already exists"):
        navigation.add_current_waypoint(
            context,
            "plant",
            "motor_front",
        )
    with pytest.raises(ValueError, match="No visible tag 8"):
        navigation.add_visible_tag_landmark(
            context,
            "plant",
            8,
        )


def test_active_map_cannot_be_deleted(tmp_path):
    navigation, _ = coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    context = navigation.create_map_definition(
        context,
        "plant",
    ).context
    navigation.observe_active_map("plant")

    with pytest.raises(ValueError, match="active map"):
        navigation.delete_map(context, "plant")


def test_delete_map_removes_definition_and_database(tmp_path):
    navigation, _ = coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    context = navigation.create_map_definition(
        context,
        "plant",
    ).context
    (tmp_path / "plant.db").write_bytes(b"database")

    state = navigation.delete_map(context, "plant")

    assert state.map_names == ()
    assert not (tmp_path / "plant.json").exists()
    assert not (tmp_path / "plant.db").exists()


def test_context_allows_only_one_runtime_operation(tmp_path):
    navigation, _ = coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    context = navigation.create_map_definition(
        context,
        "plant",
    ).context
    navigation.observe_active_map("plant")
    navigation.submit_runtime_operation(
        context,
        operation_code=5,
        command_id=CommandID.START_SLAM,
        map_name="plant",
    )

    with pytest.raises(RuntimeError, match="active operation"):
        navigation.submit_runtime_operation(
            context,
            operation_code=6,
            command_id=CommandID.START_LOCALIZATION,
            map_name="plant",
        )


def test_closing_context_cancels_inflight_operation(tmp_path):
    navigation, boundary = coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    context = navigation.create_map_definition(
        context,
        "plant",
    ).context
    navigation.observe_active_map("plant")
    operation = navigation.submit_runtime_operation(
        context,
        operation_code=5,
        command_id=CommandID.START_SLAM,
        map_name="plant",
    )

    navigation.close_context(context)

    assert boundary.cancelled == [operation.request_id]


def test_client_cannot_use_another_clients_context(tmp_path):
    navigation, _ = coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context

    with pytest.raises(ValueError, match="does not own"):
        navigation.context(context.context_id, "other-ui")


def test_close_survives_synchronous_queued_cancellation_status(tmp_path):
    navigation, boundary = coordinator(tmp_path)
    context = navigation.open_context("navigation-ui").context
    context = navigation.create_map_definition(
        context,
        "plant",
    ).context
    navigation.observe_active_map("plant")
    operation = navigation.submit_runtime_operation(
        context,
        operation_code=5,
        command_id=CommandID.START_SLAM,
        map_name="plant",
    )

    def cancel(request_id):
        boundary.cancelled.append(request_id)
        boundary.emit(
            operation.request,
            CommandControllerState.CANCELLED,
        )
        return request_id

    boundary.cancel = cancel

    navigation.close_context(context)

    assert navigation.setup_coordinator.contexts == ()
