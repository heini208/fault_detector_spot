import os
import time
from dataclasses import dataclass
from pathlib import Path

import py_trees
from std_msgs.msg import String
from std_srvs.srv import SetBool
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.clock import Clock, ClockType

from fault_detector_spot.navigation.runtime.nav2_runtime_manager import Nav2RuntimeManager
from fault_detector_spot.shared.persistence.runtime_paths import (
    default_map_root,
)
from fault_detector_spot.shared.ros.runtime_manager import RuntimeManager
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS


@dataclass(frozen=True)
class CollisionMapSession:
    """Identity of one owned mapping interval, including its ROS-clock start."""

    process_id: int
    map_name: str
    generation: int
    started_at_ns: int


@dataclass(frozen=True)
class CollisionCheckingState:
    """Authoritative map availability and the user's current session policy."""

    session: CollisionMapSession | None
    enabled: bool
    revision: int

    @property
    def available(self):
        return self.session is not None


class RtabmapRuntimeManager(RuntimeManager):
    """Manage RTAB-Map and Nav2 runtime processes."""

    PROCESS_KEY = "slam_launch_process"
    RUNTIME_NAME = "RTAB-Map"

    MODE_NONE = "none"
    MODE_MAPPING = "mapping"
    MODE_LOCALIZATION = "localization"

    def __init__(
        self,
        node,
        blackboard,
        nav2_launch_file="nav2_lidar_launch.py",
        nav2_params_file="nav2_lidar_params.yaml",
        launch_file="lidar_rtab_mapping_launch.py",
        maps_dir=None,
    ):
        super().__init__(node, blackboard)
        self._collision_map_session = None
        self._collision_map_generation = 0
        self._collision_checking_enabled = False
        self._collision_checking_revision = 0
        self.launch_file = launch_file
        configured_maps_dir = maps_dir or default_map_root()
        self.maps_dir = os.fspath(
            Path(configured_maps_dir).expanduser()
        )
        os.makedirs(self.maps_dir, exist_ok=True)
        self._init_blackboard_keys()
        self._init_ros_publishers()

        self.nav2_runtime = Nav2RuntimeManager(
            node=self.node,
            blackboard=self.bb,
            launch_file=nav2_launch_file,
            params_file=nav2_params_file,
        )
        self._status_pub = node.create_publisher(
            DiagnosticStatus, "fault_detector/navigation_runtime", LATCHED_QOS,
        )
        self._collision_checking_service = node.create_service(
            SetBool,
            "fault_detector/set_arm_collision_checking",
            self._set_collision_checking_service,
        )
        self._status_timer = node.create_timer(
            0.5, self._publish_runtime_status,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._publish_runtime_status()

    def _publish_runtime_status(self):
        errors = []
        try:
            mode = self.get_running_mode()
        except RuntimeError as exception:
            mode = self.MODE_NONE
            errors.append(str(exception))
        nav2_running = self.nav2_runtime.is_running()
        if self.process is not None and mode == self.MODE_NONE:
            errors.append("RTAB-Map process exited unexpectedly")
        if mode == self.MODE_LOCALIZATION and not nav2_running:
            errors.append("Nav2 is not running")
        if mode == self.MODE_NONE and nav2_running:
            errors.append("Nav2 is running without RTAB-Map")
        collision_state = self.collision_checking_state()
        status = DiagnosticStatus(
            name="navigation_runtime",
            level=DiagnosticStatus.ERROR if errors else DiagnosticStatus.OK,
            message="; ".join(errors) or f"Runtime mode: {mode}",
            values=[
                KeyValue(key="mode", value=mode),
                KeyValue(key="active_map", value=self.bb.active_map_name or ""),
                KeyValue(
                    key="arm_collision_available",
                    value=str(collision_state.available).lower(),
                ),
                KeyValue(
                    key="arm_collision_enabled",
                    value=str(collision_state.enabled).lower(),
                ),
            ],
        )
        self._status_pub.publish(status)

    def close(self):
        result = super().close()
        if self._status_timer is not None:
            self._publish_runtime_status()
            self.node.destroy_timer(self._status_timer)
            self._status_timer = None
        if self._collision_checking_service is not None:
            self.node.destroy_service(self._collision_checking_service)
            self._collision_checking_service = None
        return result

    def _init_blackboard_keys(self):
        self.bb.register_key(
            "active_map_name",
            access=py_trees.common.Access.WRITE,
        )
        self.bb.register_key(
            "slam_runtime_mode",
            access=py_trees.common.Access.WRITE,
        )

        if not self.bb.exists("active_map_name"):
            self.bb.active_map_name = None
        if not self.bb.exists("slam_runtime_mode"):
            self.bb.slam_runtime_mode = self.MODE_NONE

    def _init_ros_publishers(self):
        self.active_map_pub = self.node.create_publisher(
            String,
            "active_map",
            LATCHED_QOS,
        )

    def _db_path(self, map_name: str):
        return os.path.join(self.maps_dir, f"{map_name}.db")

    def _publish_active_map(self):
        if self.bb.active_map_name:
            msg = String()
            msg.data = self.bb.active_map_name
            self.active_map_pub.publish(msg)

    def _stop_rtabmap(self, save):
        """Stop only this manager's process; the caller handles Nav2 separately."""
        self._invalidate_collision_map_session()
        if save and self.is_running() and self.get_running_mode() == self.MODE_MAPPING:
            for service in ("pause", "save_db", "publish_map"):
                self._call_service(f"/rtabmap/{service}")
        stopped = self._stop_process(
            interrupt_timeout_sec=3.0 if save else 2.0,
            terminate_timeout_sec=2.0 if save else 1.0,
        )
        if stopped:
            self.bb.slam_runtime_mode = self.MODE_NONE
        return stopped

    def stop(self, save=True):
        """Stop RTAB-Map and Nav2, optionally saving an active mapping session."""
        with self._process_lock:
            try:
                stopped = self._stop_rtabmap(save)
            finally:
                nav2_stopped = self.nav2_runtime.stop()
            return stopped and nav2_stopped

    def _stop_for_close(self):
        with self._process_lock:
            try:
                return self._stop_rtabmap(save=False)
            finally:
                # Close drains Nav2's worker and stops its process once. Calling
                # stop() first would repeat the entire timeout on failure.
                self.nav2_runtime.close()

    def _call_service(
        self,
        service_name: str,
        srv_type=None,
        request=None,
        timeout_sec=2.0,
    ):
        if srv_type is None:
            from std_srvs.srv import Empty
            srv_type = Empty

        client = self.node.create_client(srv_type, service_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.node.get_logger().warning(
                    f"Service {service_name} not available"
                )
                return False

            if request is None:
                request = srv_type.Request()

            future = client.call_async(request)
            if not future:
                return False

            start_time = time.monotonic()
            while not future.done():
                time.sleep(0.05)
                if time.monotonic() - start_time > timeout_sec:
                    self.node.get_logger().warning(
                        f"Service {service_name} timed out "
                        f"after {timeout_sec}s"
                    )
                    return False

            try:
                future.result()
            except Exception as exception:
                self.node.get_logger().warning(
                    f"Service {service_name} failed: {exception}"
                )
                return False
            return True
        finally:
            try:
                self.node.destroy_client(client)
            except Exception:
                pass

    def start_mapping(self, map_name=None, delete_db=False, rviz=True):
        if (not self.is_running() or delete_db
                or (map_name is not None and map_name != self.bb.active_map_name)):
            return self._start_rtabmap(
                map_name, extend_map=True, delete_db=delete_db, rviz=rviz,
            )
        if not self.nav2_runtime.stop():
            raise RuntimeError("Could not stop Nav2 before mapping")
        self.set_mode_mapping()
        return self.process

    def _start_rtabmap(
        self,
        map_name: str = None,
        extend_map: bool = True,
        delete_db: bool = False,
        rviz: bool = True,
    ):
        if map_name is None:
            map_name = self.bb.active_map_name
            if map_name is None:
                raise RuntimeError(
                    "No active map set to start mapping/localization"
                )

        db_path = self._db_path(map_name)
        if not extend_map and not os.path.exists(db_path):
            raise FileNotFoundError(f"Database file not found for map: {db_path}")
        if not self.stop():
            raise RuntimeError("Could not stop runtime before switching maps")
        os.makedirs(os.path.dirname(db_path), exist_ok=True)

        extend_map_str = "true" if extend_map else "false"
        delete_db_str = "true" if delete_db else "false"
        rviz_str = "true" if rviz else "false"

        args = [
            f"db_path:={db_path}",
            f"delete_db:={delete_db_str}",
            f"extend_map:={extend_map_str}",
            f"rviz:={rviz_str}",
        ]

        started_at_ns = self.node.get_clock().now().nanoseconds
        proc = self._launch(self.launch_file, args)
        self.bb.slam_runtime_mode = (
            self.MODE_MAPPING if extend_map else self.MODE_LOCALIZATION
        )
        self.bb.active_map_name = map_name
        if extend_map:
            self._begin_collision_map_session(started_at_ns)
        self._publish_active_map()

        self.node.get_logger().info(
            "Started RTAB-Map in "
            f"{'mapping' if extend_map else 'localization'} mode "
            f"with DB: {db_path}"
        )

        return proc

    def start_localization(
        self,
        map_name: str = None,
        rviz: bool = True,
    ):
        if (not self.is_running()
                or (map_name is not None and map_name != self.bb.active_map_name)):
            self._start_rtabmap(map_name, extend_map=False, rviz=rviz)
        else:
            self.set_mode_localization()

        if not self.nav2_runtime.is_running():
            self.nav2_runtime.start()

        return self.bb.slam_launch_process

    def set_mode_localization(self):
        self._invalidate_collision_map_session()
        if not self._call_service("/rtabmap/set_mode_localization"):
            raise RuntimeError(
                "Could not switch RTAB-Map to localization mode"
            )
        self.bb.slam_runtime_mode = self.MODE_LOCALIZATION

    def set_mode_mapping(self):
        # Repeated requests to an already active session retain its UI policy.
        if self.current_collision_map_session() is not None:
            return
        self._invalidate_collision_map_session()
        started_at_ns = self.node.get_clock().now().nanoseconds
        if not self._call_service("/rtabmap/set_mode_mapping"):
            raise RuntimeError(
                "Could not switch RTAB-Map to mapping mode"
            )
        self.bb.slam_runtime_mode = self.MODE_MAPPING
        self._begin_collision_map_session(started_at_ns)

    def _begin_collision_map_session(self, started_at_ns):
        with self._runtime_lock:
            self._collision_map_generation += 1
            self._collision_map_session = CollisionMapSession(
                process_id=id(self.process),
                map_name=self.bb.active_map_name,
                generation=self._collision_map_generation,
                started_at_ns=started_at_ns,
            )
            self._collision_checking_enabled = True
        self._publish_runtime_status()

    def _invalidate_collision_map_session(self):
        with self._runtime_lock:
            self._collision_map_session = None
            self._collision_checking_enabled = False

    def current_collision_map_session(self) -> CollisionMapSession | None:
        """Return the active mapping interval without waiting for runtime work.

        A selected map or a localization process is insufficient. Invalidating
        before lifecycle changes also rejects snapshots taken during shutdown
        or an unconfirmed mode switch, without blocking the planning tick.
        """
        with self._runtime_lock:
            session = self._collision_map_session
            if (
                session is None
                or self._closing
                or self.bb.slam_runtime_mode != self.MODE_MAPPING
                or not self.is_running()
                or session.process_id != id(self.process)
                or session.map_name != self.bb.active_map_name
            ):
                return None
            return session

    def collision_checking_state(self) -> CollisionCheckingState:
        with self._runtime_lock:
            session = self.current_collision_map_session()
            return CollisionCheckingState(
                session=session,
                enabled=session is not None and self._collision_checking_enabled,
                revision=self._collision_checking_revision,
            )

    def set_collision_checking_enabled(self, enabled: bool) -> CollisionCheckingState:
        """Change planning policy only; never mutate MoveIt or command motion."""
        if type(enabled) is not bool:
            raise TypeError("Arm collision checking must be a boolean")
        with self._runtime_lock:
            state = self.collision_checking_state()
            if enabled and not state.available:
                raise ValueError("Arm collision checking requires an active mapping session")
            if self._collision_checking_enabled != enabled:
                self._collision_checking_enabled = enabled
                self._collision_checking_revision += 1
            state = self.collision_checking_state()
        self._publish_runtime_status()
        return state

    def _set_collision_checking_service(self, request, response):
        try:
            state = self.set_collision_checking_enabled(request.data)
        except (TypeError, ValueError) as exception:
            response.success = False
            response.message = str(exception)
        else:
            response.success = True
            response.message = (
                "Arm collision checking enabled" if state.enabled
                else "Arm collision checking disabled"
            )
        return response

    def get_running_mode(self) -> str:
        if not self.is_running():
            return self.MODE_NONE
        mode = getattr(
            self.bb,
            "slam_runtime_mode",
            self.MODE_NONE,
        )
        if mode not in {
            self.MODE_MAPPING,
            self.MODE_LOCALIZATION,
        }:
            raise RuntimeError(
                f"Unknown RTAB-Map runtime mode: {mode}"
            )
        return mode

    def is_mapping_running(self) -> bool:
        return (
            self.is_running()
            and self.get_running_mode() == self.MODE_MAPPING
        )

    def is_localization_running(self) -> bool:
        return (
            self.is_running()
            and self.get_running_mode() == self.MODE_LOCALIZATION
            and self.nav2_runtime.is_running()
        )

    def change_map(self, map_name: str):
        if not map_name:
            raise RuntimeError("No map specified to switch to")
        if map_name == self.bb.active_map_name:
            self.node.get_logger().info(
                f"Map '{map_name}' already active"
            )
            return True

        current_mode = self.get_running_mode()
        if current_mode == self.MODE_NONE:
            self.bb.active_map_name = map_name
            self._publish_active_map()
            self.node.get_logger().info(
                f"Set active map to '{map_name}' "
                "(no running process)."
            )
            return True

        self.node.get_logger().info(
            f"Switching to map '{map_name}' "
            f"in {current_mode} mode..."
        )

        if current_mode == self.MODE_MAPPING:
            self.start_mapping(map_name)
        else:
            self.start_localization(map_name)

        message = (
            f"Changed to map '{map_name}' "
            f"in {current_mode} mode"
        )
        self.node.get_logger().info(message)
        return True
