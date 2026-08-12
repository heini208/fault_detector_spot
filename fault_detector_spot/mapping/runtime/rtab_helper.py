import os
import signal
import subprocess
import time

import py_trees
from ament_index_python.packages import get_package_share_directory
from fault_detector_spot.navigation.runtime.nav2_helper import Nav2Helper
from fault_detector_spot.shared.ros.qos_profiles import LATCHED_QOS
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class RTABHelper:
    """
    Helper class for managing RTAB-Map SLAM and Localization processes.
    Integrates with py_trees Blackboard and ROS2 Node for publishing map state.
    """

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

    def save_static_map(self, path: str) -> bool:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._call_service("/rtabmap/save_db")

    def stop_current_process(self):
        proc = getattr(self.bb, "slam_launch_process", None)

        if proc:
            running_mode = self._get_running_mode()
            if running_mode == "mapping":
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
                time.sleep(5)

            self.node.get_logger().info(
                "Stopping RTAB-Map process..."
            )
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=3)
            self.bb.slam_launch_process = None

        self.stop_nav2()

    def stop_without_save(self):
        proc = getattr(self.bb, "slam_launch_process", None)
        if proc:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=2)
            except Exception as exception:
                self.node.get_logger().warn(
                    f"Failed to stop Slam Toolbox: {exception}"
                )
            finally:
                self.bb.slam_launch_process = None

        self.stop_nav2()

    def stop_nav2(self):
        if self.nav2_helper.is_running():
            self.node.get_logger().info("Stopping Nav2...")
            self.nav2_helper.stop()

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
        if not client.wait_for_service(timeout_sec=timeout_sec):
            print(f"Service {service_name} not available")
            return False

        if request is None:
            request = srv_type.Request()

        future = client.call_async(request)
        if not future:
            return False

        start_time = time.time()
        while not future.done():
            time.sleep(0.05)
            if time.time() - start_time > timeout_sec:
                self.node.get_logger().warn(
                    f"Service {service_name} timed out "
                    f"after {timeout_sec}s"
                )
                return False

        return True

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

        if self.nav2_helper.is_running():
            self.nav2_helper.stop()
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
        self.bb.active_map_name = map_name
        self._publish_active_map()

        self.node.get_logger().info(
            "Started RTAB-Map localization with database: "
            f"{db_path}"
        )
        return proc

    def set_mode_localization(self):
        self._call_service("/rtabmap/set_mode_localization")

    def set_mode_mapping(self):
        self._call_service("/rtabmap/set_mode_mapping")

    def is_rtabmap_running(self) -> bool:
        proc = self.bb.slam_launch_process
        if proc is None:
            return False
        return proc.poll() is None

    def _get_running_mode(self) -> str:
        if not self.is_rtabmap_running():
            return "none"
        result = subprocess.run(
            [
                "ros2",
                "param",
                "get",
                "/rtabmap",
                "localization",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        if "True" in result.stdout:
            return "localization"
        return "mapping"

    def is_slam_running(self) -> bool:
        return self._get_running_mode() == "mapping"

    def change_map(self, map_name: str):
        if not map_name:
            raise RuntimeError("No map specified to switch to")
        if map_name == self.bb.active_map_name:
            self.node.get_logger().info(
                f"Map '{map_name}' already active"
            )
            return True

        current_mode = self._get_running_mode()
        if current_mode == "none":
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

        if current_mode == "mapping":
            self.initialize_mapping_from_existing(map_name)
        else:
            self.start_localization(map_name)

        self.feedback_message = (
            f"Changed to map '{map_name}' "
            f"in {current_mode} mode"
        )
        self.node.get_logger().info(self.feedback_message)
        return True
