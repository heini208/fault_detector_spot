"""Coordinate navigation authoring and runtime setup operations."""

from copy import deepcopy
from dataclasses import dataclass
from threading import RLock
from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.commanding.command_ids import CommandID
from fault_detector_spot.application.commanding.command_request import (
    CommandOrigin,
)
from fault_detector_spot.application.controllers.command_controller import (
    CommandControllerState,
)
from fault_detector_spot.application.coordinators.setup_coordinator import (
    SetupCoordinator,
    SetupOperation,
    SetupOperationStatus,
)
from fault_detector_spot.application.setup.setup_context import (
    SetupContextSnapshot,
)
from fault_detector_spot.application.setup.setup_operation_registry import (
    SetupOperationRegistry,
)
from fault_detector_spot.inspection.model.models import ReferenceTag
from fault_detector_spot.mapping.model.models import (
    LocalizationLandmark,
    Waypoint,
)
from fault_detector_spot.mapping.repository.map_artifact_store import (
    MapArtifactStore,
)
from fault_detector_spot.mapping.repository.map_repository import MapRepository
from fault_detector_spot.navigation.setup.navigation_setup_command_factory import (
    NavigationSetupCommandFactory,
)
from fault_detector_spot.shared.geometry.transforms import pose_to_pose_data
from fault_detector_spot.shared.persistence.file_storage import (
    validate_storage_name,
)


MODE_NONE = "none"
MODE_MAPPING = "mapping"
MODE_LOCALIZATION = "localization"


@dataclass(frozen=True)
class NavigationSetupSnapshot:
    """Expose immutable navigation setup state."""

    context: SetupContextSnapshot
    active_map: str
    mode: str
    map_names: tuple
    waypoint_names: tuple
    landmark_names: tuple


@dataclass(frozen=True)
class NavigationSetupStatus:
    """Describe one runtime setup operation transition."""

    operation_code: int
    request_id: str
    state: CommandControllerState
    detail: str
    snapshot: NavigationSetupSnapshot


class NavigationSetupCoordinator:
    """Own navigation setup validation, persistence, and delegation."""

    def __init__(
        self,
        setup_coordinator: SetupCoordinator,
        map_repository: MapRepository,
        map_artifacts: MapArtifactStore,
        current_pose: Callable[[], Optional[PoseStamped]],
        visible_tag_pose: Callable[[int], Optional[PoseStamped]],
        command_factory=None,
    ):
        self.setup_coordinator = setup_coordinator
        self.map_repository = map_repository
        self.map_artifacts = map_artifacts
        self.current_pose = current_pose
        self.visible_tag_pose = visible_tag_pose
        self.command_factory = (
            command_factory or NavigationSetupCommandFactory()
        )
        self._lock = RLock()
        self._operations = SetupOperationRegistry()
        self._active_map = ""
        self._mode = MODE_NONE

    def open_context(self, client_id: str) -> NavigationSetupSnapshot:
        """Open one navigation setup context for a remote client."""
        context = self.setup_coordinator.open_context(
            CommandOrigin.NAVIGATION_SETUP,
            client_id,
        )
        self.setup_coordinator.add_operation_listener(
            context,
            self._handle_operation_status,
        )
        return self.snapshot(context)

    def context(
        self,
        context_id: str,
        client_id: str,
    ) -> SetupContextSnapshot:
        """Resolve a current context and verify client ownership."""
        try:
            return self.setup_coordinator.resolve_context(
                context_id,
                client_id,
                CommandOrigin.NAVIGATION_SETUP,
            )
        except LookupError as exception:
            raise LookupError(
                f"Unknown navigation context: {context_id}"
            ) from exception

    def close_context(self, context: SetupContextSnapshot) -> None:
        """Close one navigation setup context."""
        self.setup_coordinator.require_current(context)
        request_ids = self._operations.request_ids_for(context)
        for request_id in request_ids:
            try:
                self.setup_coordinator.command_controller.cancel(request_id)
            except LookupError:
                pass
        try:
            current = self.setup_coordinator.resolve_context(
                context.context_id,
                context.client_id,
                CommandOrigin.NAVIGATION_SETUP,
            )
        except LookupError:
            self._operations.discard_context(context)
            return
        self._operations.discard_context(current)
        if self.setup_coordinator.is_current(current):
            self.setup_coordinator.close_context(current)

    def observe_active_map(self, map_name: str) -> None:
        """Update the runtime active-map observation."""
        with self._lock:
            self._active_map = map_name.strip()

    def snapshot(
        self,
        context: SetupContextSnapshot,
    ) -> NavigationSetupSnapshot:
        """Build one immutable state snapshot from repository data."""
        self.setup_coordinator.require_current(context)
        with self._lock:
            active_map = self._active_map
            mode = self._mode
        map_names = tuple(self.map_repository.list_map_ids())
        waypoint_names = ()
        landmark_names = ()
        if active_map and self.map_repository.exists(active_map):
            definition = self.map_repository.load(active_map)
            waypoint_names = tuple(
                waypoint.waypoint_id for waypoint in definition.waypoints
            )
            landmark_names = tuple(
                landmark.landmark_id
                for landmark in definition.localization_landmarks
            )
        return NavigationSetupSnapshot(
            context=context,
            active_map=active_map,
            mode=mode,
            map_names=map_names,
            waypoint_names=waypoint_names,
            landmark_names=landmark_names,
        )

    def create_map_definition(
        self,
        context: SetupContextSnapshot,
        map_name: str,
    ) -> NavigationSetupSnapshot:
        """Create one empty map metadata definition."""
        self.setup_coordinator.require_current(context)
        self._require_idle(context)
        map_id = self._map_id(map_name)
        self.map_repository.create_empty(map_id)
        return self._advance(context)

    def create_and_select_map(
        self,
        context: SetupContextSnapshot,
        map_name: str,
    ) -> NavigationSetupSnapshot:
        """Create one map definition and select it for later runtime use."""
        self._require_runtime_stopped("creating a map")
        created = self.create_map_definition(context, map_name)
        return self.select_map(created.context, map_name)

    def select_map(
        self,
        context: SetupContextSnapshot,
        map_name: str,
    ) -> NavigationSetupSnapshot:
        """Select persisted map metadata while runtime navigation is stopped."""
        self.setup_coordinator.require_current(context)
        self._require_idle(context)
        self._require_runtime_stopped("selecting a map")
        map_id = self._map_id(map_name)
        if not self.map_repository.exists(map_id):
            raise FileNotFoundError(f"Unknown map: {map_id}")
        with self._lock:
            self._active_map = map_id
        return self._advance(context)

    def delete_map(
        self,
        context: SetupContextSnapshot,
        map_name: str,
    ) -> NavigationSetupSnapshot:
        """Delete an inactive map's metadata and database artifacts."""
        self.setup_coordinator.require_current(context)
        self._require_idle(context)
        map_id = self._map_id(map_name)
        with self._lock:
            if map_id == self._active_map:
                raise ValueError("The active map cannot be deleted")
        self.map_artifacts.delete(map_id)
        return self._advance(context)

    def add_current_waypoint(
        self,
        context: SetupContextSnapshot,
        map_name: str,
        waypoint_name: str,
    ) -> NavigationSetupSnapshot:
        """Persist the current localized robot pose as a waypoint."""
        self.setup_coordinator.require_current(context)
        self._require_idle(context)
        map_id = self._require_active_map(map_name)
        self._require_authoring_mode()
        waypoint_id = self._name(waypoint_name, "waypoint ID")
        pose = self._map_pose(self.current_pose(), "localization pose")
        self.map_repository.add_waypoint(
            map_id,
            Waypoint(
                waypoint_id=waypoint_id,
                display_name=waypoint_id,
                pose_map=pose_to_pose_data(pose),
            ),
        )
        return self._advance(context)

    def add_visible_tag_landmark(
        self,
        context: SetupContextSnapshot,
        map_name: str,
        tag_id: int,
    ) -> NavigationSetupSnapshot:
        """Persist one currently visible AprilTag pose as a landmark."""
        self.setup_coordinator.require_current(context)
        self._require_idle(context)
        map_id = self._require_active_map(map_name)
        self._require_authoring_mode()
        if isinstance(tag_id, bool) or not isinstance(tag_id, int):
            raise TypeError("Tag ID must be an integer")
        if tag_id < 0:
            raise ValueError("Tag ID must not be negative")
        pose = self._map_pose(
            self.visible_tag_pose(tag_id),
            f"visible tag {tag_id}",
        )
        landmark_id = f"Tag_{tag_id}"
        self.map_repository.add_landmark(
            map_id,
            LocalizationLandmark(
                landmark_id=landmark_id,
                display_name=landmark_id,
                reference_tag=ReferenceTag(
                    tag_id=tag_id,
                    tag_family="36h11",
                ),
                pose_map=pose_to_pose_data(pose),
            ),
        )
        return self._advance(context)

    def delete_waypoint(
        self,
        context: SetupContextSnapshot,
        map_name: str,
        waypoint_name: str,
    ) -> NavigationSetupSnapshot:
        """Delete one waypoint through the map repository."""
        self.setup_coordinator.require_current(context)
        self._require_idle(context)
        map_id = self._map_id(map_name)
        waypoint_id = self._name(waypoint_name, "waypoint ID")
        self.map_repository.delete_waypoint(map_id, waypoint_id)
        return self._advance(context)

    def delete_landmark(
        self,
        context: SetupContextSnapshot,
        map_name: str,
        landmark_name: str,
    ) -> NavigationSetupSnapshot:
        """Delete one landmark through the map repository."""
        self.setup_coordinator.require_current(context)
        self._require_idle(context)
        map_id = self._map_id(map_name)
        landmark_id = self._name(landmark_name, "landmark ID")
        self.map_repository.delete_landmark(map_id, landmark_id)
        return self._advance(context)

    def submit_runtime_operation(
        self,
        context: SetupContextSnapshot,
        operation_code: int,
        command_id: CommandID,
        map_name: str = "",
    ) -> SetupOperation:
        """Delegate one asynchronous runtime operation to the shared lane."""
        self.setup_coordinator.require_current(context)
        self._require_idle(context)
        command_id, normalized_map = self._runtime_command(
            command_id,
            map_name,
        )
        command = self.command_factory.create(command_id, normalized_map)
        operation = self.setup_coordinator.prepare_command(context, command)
        self._register_pending_operation(
            operation,
            operation_code,
            context,
            command_id,
            normalized_map,
        )
        return operation

    def _runtime_command(self, command_id, map_name):
        command_id = CommandID(command_id)
        normalized_map = map_name.strip()
        if command_id in {
            CommandID.START_SLAM,
            CommandID.START_LOCALIZATION,
        }:
            normalized_map = self._require_active_map(normalized_map)
        if command_id == CommandID.SWAP_MAP:
            map_id = self._map_id(normalized_map)
            if not self.map_repository.exists(map_id):
                raise FileNotFoundError(f"Unknown map: {map_id}")
            normalized_map = map_id
        return command_id, normalized_map

    def _register_pending_operation(
        self,
        operation,
        operation_code,
        context,
        command_id,
        map_name,
    ):
        self._operations.register(
            operation.request_id,
            context,
            (
                int(operation_code),
                command_id,
                map_name,
            ),
        )
        try:
            self.setup_coordinator.submit(operation)
        except Exception:
            self._operations.pop(operation.request_id)
            raise

    def add_status_listener(self, listener) -> None:
        """Register one navigation setup status listener."""
        self._operations.add_listener(listener)

    def cancel(
        self,
        context: SetupContextSnapshot,
        request_id: str,
    ) -> str:
        """Cancel one pending runtime operation owned by a context."""
        self.setup_coordinator.require_current(context)
        normalized = request_id.strip()
        if self._operations.owned(normalized, context) is None:
            raise LookupError(
                f"Unknown navigation setup request: {request_id}"
            )
        return self.setup_coordinator.command_controller.cancel(normalized)

    def remove_status_listener(self, listener) -> None:
        """Remove one navigation setup status listener."""
        self._operations.remove_listener(listener)

    def close(self) -> None:
        """Close all owned contexts and discard specialized state."""
        contexts = self.setup_coordinator.contexts_for(
            CommandOrigin.NAVIGATION_SETUP
        )
        for context in contexts:
            if self.setup_coordinator.is_current(context):
                self.close_context(context)
        self._operations.clear()

    def _handle_operation_status(self, status: SetupOperationStatus) -> None:
        tracked = self._operations.get(status.operation.request_id)
        if tracked is None:
            return
        context = tracked.context
        operation_code, command_id, map_name = tracked.payload
        terminal = status.state in {
            CommandControllerState.SUCCEEDED,
            CommandControllerState.FAILED,
            CommandControllerState.CANCELLED,
        }
        if terminal:
            self._operations.pop(status.operation.request_id)
            if status.state == CommandControllerState.SUCCEEDED:
                self._apply_runtime_success(command_id, map_name)
            current = self._advance(context)
        else:
            current = self.snapshot(context)
        self._operations.emit(
            NavigationSetupStatus(
                operation_code=operation_code,
                request_id=status.operation.request_id,
                state=status.state,
                detail=status.detail,
                snapshot=current,
            )
        )

    def _apply_runtime_success(
        self,
        command_id: CommandID,
        map_name: str,
    ) -> None:
        with self._lock:
            if command_id == CommandID.SWAP_MAP:
                self._active_map = map_name
            elif command_id == CommandID.START_SLAM:
                self._mode = MODE_MAPPING
            elif command_id == CommandID.START_LOCALIZATION:
                self._mode = MODE_LOCALIZATION
            elif command_id == CommandID.STOP_MAPPING:
                self._mode = MODE_NONE

    def _advance(
        self,
        context: SetupContextSnapshot,
    ) -> NavigationSetupSnapshot:
        updated = self.setup_coordinator.advance_context(context)
        return self.snapshot(updated)

    def _require_active_map(self, map_name: str) -> str:
        map_id = self._map_id(map_name)
        with self._lock:
            active_map = self._active_map
        if not active_map:
            raise ValueError("No active map is selected")
        if map_id != active_map:
            raise ValueError(
                f"Map is not active: {map_id}"
            )
        if not self.map_repository.exists(map_id):
            raise FileNotFoundError(f"Unknown map: {map_id}")
        return map_id

    def _require_idle(self, context: SetupContextSnapshot) -> None:
        if self._operations.has_context(context):
            raise RuntimeError(
                "Navigation setup context already has an active operation"
            )

    def _require_authoring_mode(self) -> None:
        with self._lock:
            mode = self._mode
        if mode not in {MODE_MAPPING, MODE_LOCALIZATION}:
            raise RuntimeError(
                "Mapping or localization must be active before saving poses"
            )

    def _require_runtime_stopped(self, operation: str) -> None:
        with self._lock:
            mode = self._mode
        if mode != MODE_NONE:
            raise RuntimeError(
                f"Stop mapping or localization before {operation}"
            )

    @staticmethod
    def _map_id(value: str) -> str:
        return validate_storage_name(value.strip(), "map ID")

    @staticmethod
    def _name(value: str, label: str) -> str:
        return validate_storage_name(value.strip(), label)

    @staticmethod
    def _map_pose(
        pose: Optional[PoseStamped],
        label: str,
    ) -> PoseStamped:
        if pose is None:
            raise ValueError(f"No {label} is available")
        if not isinstance(pose, PoseStamped):
            raise TypeError(f"{label.title()} must be a PoseStamped")
        if pose.header.frame_id != "map":
            raise ValueError(f"{label.title()} must use the map frame")
        return deepcopy(pose)


__all__ = [
    "MODE_LOCALIZATION",
    "MODE_MAPPING",
    "MODE_NONE",
    "NavigationSetupCoordinator",
    "NavigationSetupSnapshot",
    "NavigationSetupStatus",
]
