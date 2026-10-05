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


def pending_swap(tmp_path):
    navigation, boundary = coordinator(tmp_path)
    owner = navigation.open_context("owner").context
    owner = navigation.create_and_select_map(owner, "plant").context
    owner = navigation.create_map_definition(owner, "other").context
    other = navigation.open_context("other-client").context
    operation = navigation.submit_runtime_operation(
        owner, operation_code=1, command_id=CommandID.SWAP_MAP,
        map_name="other",
    )
    return navigation, boundary, owner, other, operation


@pytest.mark.parametrize("action", ["select", "delete", "create_select", "runtime"])
def test_pending_runtime_blocks_other_contexts(tmp_path, action):
    navigation, boundary, owner, other, operation = pending_swap(tmp_path)
    actions = {
        "select": lambda: navigation.select_map(other, "other"),
        "delete": lambda: navigation.delete_map(other, "other"),
        "create_select": lambda: navigation.create_and_select_map(other, "new"),
        "runtime": lambda: navigation.submit_runtime_operation(
            other, 2, CommandID.START_SLAM, "plant",
        ),
    }
    with pytest.raises(RuntimeError, match="active operation"):
        actions[action]()
    assert navigation.snapshot(other).active_map == "plant"
    assert navigation.map_repository.exists("other")
    assert not navigation.map_repository.exists("new")
    assert boundary.submitted == [operation.request]


@pytest.mark.parametrize("state", [
    CommandControllerState.SUCCEEDED,
    CommandControllerState.FAILED,
    CommandControllerState.CANCELLED,
])
def test_shared_runtime_released_only_at_terminal_status(tmp_path, state):
    navigation, boundary, owner, other, operation = pending_swap(tmp_path)
    navigation.cancel(owner, operation.request_id)
    with pytest.raises(RuntimeError, match="active operation"):
        navigation.select_map(other, "plant")
    boundary.emit(operation.request, state)
    result = navigation.select_map(other, "plant")
    assert result.active_map == "plant"


def test_closing_owner_keeps_shared_runtime_reserved_until_cancel_finishes(tmp_path):
    navigation, boundary, owner, other, operation = pending_swap(tmp_path)
    navigation.close_context(owner)
    with pytest.raises(RuntimeError, match="active operation"):
        navigation.delete_map(other, "other")
    boundary.emit(operation.request, CommandControllerState.CANCELLED)
    with pytest.raises(LookupError):
        navigation.context(owner.context_id, owner.client_id)
    result = navigation.delete_map(other, "other")
    assert result.map_names == ("plant",)


def test_dispatch_failure_releases_shared_runtime(tmp_path, monkeypatch):
    navigation, boundary = coordinator(tmp_path)
    owner = navigation.open_context("owner").context
    owner = navigation.create_and_select_map(owner, "plant").context
    other = navigation.open_context("other").context

    def fail_submit(_request):
        raise RuntimeError("dispatch failed")

    monkeypatch.setattr(boundary, "submit", fail_submit)
    with pytest.raises(RuntimeError, match="dispatch failed"):
        navigation.submit_runtime_operation(owner, 1, CommandID.START_SLAM, "plant")
    assert navigation.select_map(other, "plant").active_map == "plant"


def test_concurrent_request_cannot_enter_between_validation_and_registration(tmp_path, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    from threading import Event

    navigation, boundary = coordinator(tmp_path)
    owner = navigation.open_context("owner").context
    owner = navigation.create_and_select_map(owner, "plant").context
    other = navigation.open_context("other").context
    entered = Event()
    release = Event()
    original = navigation.command_factory.create

    def create(*args):
        entered.set()
        assert release.wait(5)
        return original(*args)

    monkeypatch.setattr(navigation.command_factory, "create", create)
    with ThreadPoolExecutor(max_workers=2) as pool:
        first = pool.submit(navigation.submit_runtime_operation,
                            owner, 1, CommandID.START_SLAM, "plant")
        assert entered.wait(5)
        second = pool.submit(navigation.select_map, other, "plant")
        release.set()
        operation = first.result(timeout=5)
        with pytest.raises(RuntimeError, match="active operation"):
            second.result(timeout=5)
    assert boundary.submitted == [operation.request]


def test_close_during_dispatch_keeps_reservation_and_cancels_after_submit(tmp_path, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    from threading import Event

    navigation, boundary = coordinator(tmp_path)
    owner = navigation.open_context("owner").context
    owner = navigation.create_and_select_map(owner, "plant").context
    other = navigation.open_context("other").context
    entered = Event()
    release = Event()
    original = navigation.setup_coordinator.submit

    def submit(operation):
        entered.set()
        assert release.wait(5)
        return original(operation)

    monkeypatch.setattr(navigation.setup_coordinator, "submit", submit)
    with ThreadPoolExecutor(max_workers=1) as pool:
        pending = pool.submit(navigation.submit_runtime_operation,
                              owner, 1, CommandID.START_SLAM, "plant")
        assert entered.wait(5)
        try:
            navigation.close_context(owner)
            with pytest.raises(RuntimeError, match="active operation"):
                navigation.select_map(other, "plant")
        finally:
            release.set()
        operation = pending.result(timeout=5)
    assert boundary.cancelled == [operation.request_id]
    boundary.emit(operation.request, CommandControllerState.CANCELLED)
    assert navigation.select_map(other, "plant").active_map == "plant"


def test_coordinator_shutdown_releases_deferred_contexts(tmp_path):
    navigation, boundary, owner, other, operation = pending_swap(tmp_path)
    navigation.close()
    assert boundary.cancelled == [operation.request_id]
    assert navigation.setup_coordinator.contexts == ()
