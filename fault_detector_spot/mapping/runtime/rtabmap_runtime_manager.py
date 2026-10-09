import os
import time
from pathlib import Path

import py_trees
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from rclpy.clock import Clock, ClockType

from fault_detector_spot.navigation.runtime.nav2_runtime_manager import Nav2RuntimeManager
from fault_detector_spot.shared.persistence.runtime_paths import (
    default_map_root,
)
from fault_detector_spot.shared.ros.runtime_manager import RuntimeManager
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS


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
        raw_lidar_topic=None,
    ):
        super().__init__(node, blackboard)
        self.launch_file = launch_file
        self.raw_lidar_topic = raw_lidar_topic
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
        status = DiagnosticStatus(
            name="navigation_runtime",
            level=DiagnosticStatus.ERROR if errors else DiagnosticStatus.OK,
            message="; ".join(errors) or f"Runtime mode: {mode}",
            values=[
                KeyValue(key="mode", value=mode),
                KeyValue(key="active_map", value=self.bb.active_map_name or ""),
            ],
        )
        self._status_pub.publish(status)

    def close(self):
        result = super().close()
        if self._status_timer is not None:
            self._publish_runtime_status()
            self.node.destroy_timer(self._status_timer)
            self._status_timer = None
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
        if self.raw_lidar_topic is not None:
            args.append(f"raw_lidar_topic:={self.raw_lidar_topic}")

        proc = self._launch(self.launch_file, args)
        self.bb.slam_runtime_mode = (
            self.MODE_MAPPING if extend_map else self.MODE_LOCALIZATION
        )
        self.bb.active_map_name = map_name
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
        """Reuse matching localization, otherwise stop and launch its database."""
        if (self.get_running_mode() != self.MODE_LOCALIZATION
                or (map_name is not None and map_name != self.bb.active_map_name)):
            self._start_rtabmap(map_name, extend_map=False, rviz=rviz)

        if not self.nav2_runtime.is_running():
            self.nav2_runtime.start()

        return self.bb.slam_launch_process

    def set_mode_localization(self):
        if not self._call_service("/rtabmap/set_mode_localization"):
            raise RuntimeError(
                "Could not switch RTAB-Map to localization mode"
            )
        self.bb.slam_runtime_mode = self.MODE_LOCALIZATION

    def set_mode_mapping(self):
        if not self._call_service("/rtabmap/set_mode_mapping"):
            raise RuntimeError(
                "Could not switch RTAB-Map to mapping mode"
            )
        self.bb.slam_runtime_mode = self.MODE_MAPPING

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
