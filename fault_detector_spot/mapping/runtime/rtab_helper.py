import os
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from threading import RLock

import py_trees
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from fault_detector_spot.navigation.runtime.nav2_helper import Nav2Helper
from fault_detector_spot.shared.ros.process_lifecycle import (
    terminate_process_group,
)
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS


class RTABHelper:
    """Manage RTAB-Map and Nav2 runtime processes."""

    MODE_NONE = "none"
    MODE_MAPPING = "mapping"
    MODE_LOCALIZATION = "localization"

    def __init__(
        self,
        node,
        blackboard,
        nav2_launch_file="nav2_sim_launch.py",
        nav2_params_file=None,
        launch_file="rtab_mapping_launch.py",
    ):
        self.node = node
        self.slam_launch_file = launch_file
        self.bb = blackboard
        self.maps_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"),
            "maps",
        )
        os.makedirs(self.maps_dir, exist_ok=True)
        self._runtime_executor = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="mapping-runtime",
        )
        self._runtime_lock = RLock()
        self._runtime_future = None
        self._runtime_operation = ""
        self.init_blackboard_keys()
        self.init_ros_publishers()

        self.nav2_helper = Nav2Helper(
            node=self.node,
            blackboard=self.bb,
            launch_file=nav2_launch_file,
            params_file=nav2_params_file,
        )

    def init_blackboard_keys(self):
        self.bb.register_key(
            "active_map_name",
            access=py_trees.common.Access.WRITE,
        )
        self.bb.register_key(
            "slam_launch_process",
            access=py_trees.common.Access.WRITE,
        )
        self.bb.register_key(
            "slam_runtime_mode",
            access=py_trees.common.Access.WRITE,
        )
        self.bb.register_key(
            "nav2_launch_process",
            access=py_trees.common.Access.WRITE,
        )
        self.bb.register_key(
            "last_pose_estimation",
            access=py_trees.common.Access.READ,
        )

        if not self.bb.exists("active_map_name"):
            self.bb.active_map_name = None
        if not self.bb.exists("slam_launch_process"):
            self.bb.slam_launch_process = None
        if not self.bb.exists("slam_runtime_mode"):
            self.bb.slam_runtime_mode = self.MODE_NONE
        if not self.bb.exists("nav2_launch_process"):
            self.bb.nav2_launch_process = None

    def init_ros_publishers(self):
        self.active_map_pub = self.node.create_publisher(
            String,
            "active_map",
            LATCHED_QOS,
        )

    def _db_path(self, map_name: str):
        return os.path.join(self.maps_dir, f"{map_name}.db")

    def get_json_path(self, map_name: str):
        return os.path.join(self.maps_dir, f"{map_name}.json")

    def _publish_active_map(self):
        if self.bb.active_map_name:
            msg = String()
            msg.data = self.bb.active_map_name
            self.active_map_pub.publish(msg)

    def _use_sim_time(self) -> bool:
        try:
            return bool(
                self.node.get_parameter("use_sim_time").value
            )
        except Exception:
            return False

    def _use_sim_time_launch_arg(self) -> str:
        return "true" if self._use_sim_time() else "false"

    def begin_runtime_operation(
        self,
        operation_name: str,
        callback,
        *args,
        **kwargs,
    ) -> bool:
        with self._runtime_lock:
            self._reap_completed_runtime_operation()
            if self._runtime_future is not None:
                return False
            self._runtime_operation = operation_name
            self._runtime_future = self._runtime_executor.submit(
                callback,
                *args,
                **kwargs,
            )
            return True

    def poll_runtime_operation(self, operation_name: str):
        with self._runtime_lock:
            future = self._runtime_future
            if future is None:
                raise RuntimeError(
                    f"Runtime operation '{operation_name}' is not active"
                )
            if self._runtime_operation != operation_name:
                raise RuntimeError(
                    "Another mapping runtime operation is active: "
                    f"{self._runtime_operation}"
                )
            if not future.done():
                return None

            self._runtime_future = None
            self._runtime_operation = ""

        return future.result()

    def _reap_completed_runtime_operation(self):
        future = self._runtime_future
        if future is None or not future.done():
            return

        operation_name = self._runtime_operation
        self._runtime_future = None
        self._runtime_operation = ""
        try:
            future.result()
        except Exception as exception:
            self.node.get_logger().warning(
                "Discarded completed mapping runtime operation "
                f"'{operation_name}' after preemption: {exception}"
            )

    def save_static_map(self, path: str) -> bool:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        return self._call_service("/rtabmap/save_db")

    def stop_current_process(self):
        proc = getattr(self.bb, "slam_launch_process", None)

        if proc is not None:
            running_mode = self.get_running_mode()
            if running_mode == self.MODE_MAPPING:
                self.node.get_logger().info(
                    "Pausing RTAB-Map before shutdown..."
                )
                self._call_service("/rtabmap/pause")

                self.node.get_logger().info(
                    "Saving the RTAB-Map database to disk..."
                )
                self._call_service("/rtabmap/save_db")

                self.node.get_logger().info(
                    "Publishing latest map to topics..."
                )
                self._call_service("/rtabmap/publish_map")

            self.node.get_logger().info(
                "Stopping RTAB-Map process..."
            )
            if not terminate_process_group(
                proc,
                interrupt_timeout_sec=3.0,
                terminate_timeout_sec=2.0,
                kill_timeout_sec=1.0,
            ):
                raise RuntimeError(
                    "RTAB-Map launch process did not terminate"
                )
            self.bb.slam_launch_process = None
            self.bb.slam_runtime_mode = self.MODE_NONE

        self.stop_nav2()
        return True

    def stop_without_save(self):
        proc = getattr(self.bb, "slam_launch_process", None)
        if proc is not None:
            if not terminate_process_group(
                proc,
                interrupt_timeout_sec=2.0,
                terminate_timeout_sec=1.0,
                kill_timeout_sec=1.0,
            ):
                raise RuntimeError(
                    "RTAB-Map launch process did not terminate"
                )
            self.bb.slam_launch_process = None
            self.bb.slam_runtime_mode = self.MODE_NONE

        self.stop_nav2()
        return True

    def stop_nav2(self):
        if not self.nav2_helper.stop():
            raise RuntimeError("Nav2 launch process did not terminate")
        return True

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

    def get_last_localization_pose(
        self,
        tolerance_sec: float = 0.1,
    ) -> PoseStamped | None:
        pose = PoseStamped()
        last_pose = self.bb.last_pose_estimation
        pose.header = last_pose.header
        pose.pose = last_pose.pose.pose
        return pose

    def start_mapping_from_existing(
        self,
        map_name: str = None,
        extend_map: bool = True,
        delete_db: bool = False,
        rviz: bool = True,
    ):
        if not self.is_rtabmap_running():
            return self.initialize_mapping_from_existing(
                map_name,
                extend_map,
                delete_db,
                rviz,
            )

        if self.nav2_helper.is_running() and not self.nav2_helper.stop():
            raise RuntimeError("Could not stop Nav2 before mapping")
        if extend_map:
            self.set_mode_mapping()
        else:
            self.set_mode_localization()
        return self.bb.slam_launch_process

    def initialize_mapping_from_existing(
        self,
        map_name: str = None,
        extend_map: bool = True,
        delete_db: bool = False,
        rviz: bool = True,
    ):
        self.stop_current_process()

        if map_name is None:
            map_name = self.bb.active_map_name
            if map_name is None:
                raise RuntimeError(
                    "No active map set to start mapping/localization"
                )

        db_path = self._db_path(map_name)
        os.makedirs(os.path.dirname(db_path), exist_ok=True)

        extend_map_str = "true" if extend_map else "false"
        delete_db_str = "true" if delete_db else "false"
        rviz_str = "true" if rviz else "false"

        args = [
            "ros2",
            "launch",
            "fault_detector_spot",
            self.slam_launch_file,
            f"db_path:={db_path}",
            f"delete_db:={delete_db_str}",
            f"extend_map:={extend_map_str}",
            f"rviz:={rviz_str}",
            f"use_sim_time:={self._use_sim_time_launch_arg()}",
        ]

        proc = subprocess.Popen(args, preexec_fn=os.setsid)

        self.bb.slam_launch_process = proc
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
        if not self.is_rtabmap_running():
            self.init_localization(map_name, rviz)
        else:
            self.set_mode_localization()

        if not self.nav2_helper.is_running():
            self.nav2_helper.start()

        return self.bb.slam_launch_process

    def init_localization(
        self,
        map_name: str = None,
        rviz: bool = True,
    ):
        self.stop_current_process()

        if map_name is None:
            map_name = self.bb.active_map_name
            if map_name is None:
                raise RuntimeError(
                    "No active map specified to start localization"
                )

        db_path = self._db_path(map_name)
        if not os.path.exists(db_path):
            raise FileNotFoundError(
                f"Database file not found for map: {db_path}"
            )

        os.makedirs(os.path.dirname(db_path), exist_ok=True)

        rviz_str = "true" if rviz else "false"

        args = [
            "ros2",
            "launch",
            "fault_detector_spot",
            self.slam_launch_file,
            f"db_path:={db_path}",
            "delete_db:=false",
            "extend_map:=false",
            f"rviz:={rviz_str}",
            f"use_sim_time:={self._use_sim_time_launch_arg()}",
        ]

        proc = subprocess.Popen(args, preexec_fn=os.setsid)

        self.bb.slam_launch_process = proc
        self.bb.slam_runtime_mode = self.MODE_LOCALIZATION
        self.bb.active_map_name = map_name
        self._publish_active_map()

        self.node.get_logger().info(
            "Started RTAB-Map localization with database: "
            f"{db_path}"
        )
        return proc

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

    def is_rtabmap_running(self) -> bool:
        proc = getattr(self.bb, "slam_launch_process", None)
        if proc is None:
            return False
        return proc.poll() is None

    def get_running_mode(self) -> str:
        if not self.is_rtabmap_running():
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

    def _get_running_mode(self) -> str:
        return self.get_running_mode()

    def is_slam_running(self) -> bool:
        return self.is_mapping_running()

    def is_mapping_running(self) -> bool:
        return (
            self.is_rtabmap_running()
            and self.get_running_mode() == self.MODE_MAPPING
        )

    def is_localization_running(self) -> bool:
        return (
            self.is_rtabmap_running()
            and self.get_running_mode() == self.MODE_LOCALIZATION
            and self.nav2_helper.is_running()
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

        self.stop_current_process()
        self.node.get_logger().info(
            f"Switching to map '{map_name}' "
            f"in {current_mode} mode..."
        )

        if current_mode == self.MODE_MAPPING:
            self.initialize_mapping_from_existing(map_name)
        else:
            self.start_localization(map_name)

        self.feedback_message = (
            f"Changed to map '{map_name}' "
            f"in {current_mode} mode"
        )
        self.node.get_logger().info(self.feedback_message)
        return True
