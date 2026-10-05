"""Coordinate navigation authoring and runtime setup operations."""

from copy import deepcopy
from dataclasses import dataclass
from threading import RLock
from typing import Callable, Optional

from fault_detector_spot.shared.geometry.models import PoseData

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
    runtime_error: str = ""


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
        current_pose: Callable[[], Optional[PoseData]],
        visible_tag_pose: Callable[[int], Optional[PoseData]],
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
        self._closing_contexts = set()
        self._active_map = ""
        self._mode = MODE_NONE
        self._runtime_observation = None
        self._runtime_error = ""

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

    def is_current(self, context: SetupContextSnapshot) -> bool:
        """Return whether a navigation context snapshot is still current."""
        return self.setup_coordinator.is_current(context)

    def uses_setup_coordinator(self, coordinator: SetupCoordinator) -> bool:
        """Return whether this facade uses the supplied shared coordinator."""
        return self.setup_coordinator is coordinator

    def close_context(self, context: SetupContextSnapshot) -> None:
        """Close one navigation setup context."""
        with self._lock:
            self.setup_coordinator.require_current(context)
            self._closing_contexts.add(context.context_id)
            request_ids = self._operations.request_ids_for(context)
        for request_id in request_ids:
            try:
                self.setup_coordinator.cancel_operation(context, request_id)
            except LookupError:
                pass
        with self._lock:
            if not self._operations.has_context(context):
                self._finish_context_close(context)

    def _finish_context_close(self, context):
        self._closing_contexts.discard(context.context_id)
        try:
            current = self.setup_coordinator.resolve_context(
                context.context_id,
                context.client_id,
                CommandOrigin.NAVIGATION_SETUP,
            )
        except LookupError:
            return
        self.setup_coordinator.close_context(current)

    def observe_active_map(self, map_name: str) -> None:
        """Accept the legacy map observation until runtime status is available."""
        with self._lock:
            if self._runtime_observation is None:
                self._active_map = map_name.strip()

    def observe_runtime(self, mode, map_name: str, error: str = "") -> None:
        """Reconcile process observations without treating commands as state."""
        if mode not in {None, MODE_NONE, MODE_MAPPING, MODE_LOCALIZATION}:
            raise ValueError(f"Unknown runtime mode: {mode}")
        updates = []
        with self._lock:
            observation = (mode, map_name, error)
            if observation == self._runtime_observation:
                return
            self._runtime_observation = observation
            self._runtime_error = error
            if mode is not None:
                self._mode = mode
            # A stopped runtime retains its old map; preserve the user's selection.
            if mode in {MODE_MAPPING, MODE_LOCALIZATION}:
                self._active_map = map_name
            for context in self.setup_coordinator.contexts_for(CommandOrigin.NAVIGATION_SETUP):
                if self._operations.has_context(context):
                    continue
                updates.append(NavigationSetupStatus(
                    operation_code=0, request_id="",
                    state=(CommandControllerState.FAILED if error
                           else CommandControllerState.SUCCEEDED),
                    detail=error or f"Runtime mode: {self._mode}",
                    snapshot=self._advance(context),
                ))
        for update in updates:
            self._operations.emit(update)

    def snapshot(
        self,
        context: SetupContextSnapshot,
    ) -> NavigationSetupSnapshot:
        """Build one immutable state snapshot from repository data."""
        self.setup_coordinator.require_current(context)
        with self._lock:
            active_map = self._active_map
            mode = self._mode
            runtime_error = self._runtime_error
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
            runtime_error=runtime_error,
        )

    def create_map_definition(
        self,
        context: SetupContextSnapshot,
        map_name: str,
    ) -> NavigationSetupSnapshot:
        """Create one empty map metadata definition."""
        with self._lock:
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
        with self._lock:
            self._require_runtime_stopped("creating a map")
            created = self.create_map_definition(context, map_name)
            return self.select_map(created.context, map_name)

    def select_map(
        self,
        context: SetupContextSnapshot,
        map_name: str,
    ) -> NavigationSetupSnapshot:
        """Select persisted map metadata while runtime navigation is stopped."""
        with self._lock:
            self.setup_coordinator.require_current(context)
            self._require_idle(context)
            self._require_runtime_stopped("selecting a map")
            map_id = self._map_id(map_name)
            if not self.map_repository.exists(map_id):
                raise FileNotFoundError(f"Unknown map: {map_id}")
            self._active_map = map_id
            return self._advance(context)

    def delete_map(
        self,
        context: SetupContextSnapshot,
        map_name: str,
    ) -> NavigationSetupSnapshot:
        """Delete an inactive map's metadata and database artifacts."""
        with self._lock:
            self.setup_coordinator.require_current(context)
            self._require_idle(context)
            map_id = self._map_id(map_name)
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
        with self._lock:
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
                    pose_map=pose,
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
        with self._lock:
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
                    pose_map=pose,
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
        with self._lock:
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
        with self._lock:
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
        with self._lock:
            self.setup_coordinator.require_current(context)
            self._require_idle(context)
            command_id, normalized_map = self._runtime_command(
                command_id, map_name,
            )
            command = self.command_factory.create(command_id, normalized_map)
            operation = self.setup_coordinator.prepare_command(context, command)
            self._operations.register(
                operation.request_id,
                context,
                (int(operation_code), command_id, normalized_map),
            )
        # Command status callbacks can enter this coordinator synchronously.
        # Reserve the shared runtime first, but do not hold its lock at dispatch.
        try:
            self.setup_coordinator.submit(operation)
        except Exception:
            with self._lock:
                self._operations.pop(operation.request_id)
                if context.context_id in self._closing_contexts:
                    self._finish_context_close(context)
            raise
        with self._lock:
            closing = (
                context.context_id in self._closing_contexts
                and self._operations.get(operation.request_id) is not None
            )
        if closing:
            try:
                self.setup_coordinator.cancel_operation(context, operation.request_id)
            except LookupError:
                pass
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
        return self.setup_coordinator.cancel_operation(
            context,
            normalized,
        )

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
        with self._lock:
            # Full coordinator shutdown releases contexts after requesting stops.
            for context in self.setup_coordinator.contexts_for(
                CommandOrigin.NAVIGATION_SETUP
            ):
                self._finish_context_close(context)
            self._operations.clear()

    def _handle_operation_status(self, status: SetupOperationStatus) -> None:
        with self._lock:
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
            if terminal and context.context_id in self._closing_contexts:
                self._finish_context_close(current.context)
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
        if self._operations.has_operations():
            raise RuntimeError(
                "Navigation setup already has an active operation"
            )

    def _require_authoring_mode(self) -> None:
        with self._lock:
            mode = self._mode
            if self._runtime_error:
                raise RuntimeError(self._runtime_error)
        if mode not in {MODE_MAPPING, MODE_LOCALIZATION}:
            raise RuntimeError(
                "Mapping or localization must be active before saving poses"
            )

    def _require_runtime_stopped(self, operation: str) -> None:
        with self._lock:
            mode = self._mode
            if self._runtime_error:
                raise RuntimeError(self._runtime_error)
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
        pose: Optional[PoseData],
        label: str,
    ) -> PoseData:
        if pose is None:
            raise ValueError(f"No {label} is available")
        if not isinstance(pose, PoseData):
            raise TypeError(f"{label.title()} must be map-frame PoseData")
        pose.validate()
        return deepcopy(pose)


__all__ = [
    "MODE_LOCALIZATION",
    "MODE_MAPPING",
    "MODE_NONE",
    "NavigationSetupCoordinator",
    "NavigationSetupSnapshot",
    "NavigationSetupStatus",
]
