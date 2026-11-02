import py_trees

from fault_detector_spot.mapping.runtime.rtabmap_runtime_manager import RtabmapRuntimeManager
from fault_detector_spot.shared.persistence.runtime_paths import (
    default_map_root,
)
from fault_detector_spot.application.behaviour_tree.behaviours.robot_command_resources import (
    RobotCommandResources,
)
from fault_detector_spot.sensing.tag_state_source import TagStateSource
from fault_detector_spot.sensing.lidar_adapter_runtime import LidarAdapterRuntime


class HelperInitializer(py_trees.behaviour.Behaviour):
    """Initialize resources shared by the behavior-tree runtime."""

    def __init__(self, name: str, node):
        super().__init__(name)
        self.node = node
        self.rtabmap_runtime = None
        self.nav2_runtime = None
        self.lidar_adapter_runtime = None
        self.tag_state_source = None
        self.robot_command_resources = RobotCommandResources()

    def setup(self, timeout):
        self.bb_client = self.attach_blackboard_client()
        self.robot_command_resources.get_arm_state_source(self.node)

        if not self.node.has_parameter(
            "tag_sensing.state_timeout_sec"
        ):
            self.node.declare_parameter(
                "tag_sensing.state_timeout_sec",
                1.5,
            )
        tag_state_timeout_sec = float(
            self.node.get_parameter(
                "tag_sensing.state_timeout_sec"
            ).value
        )
        if self.tag_state_source is None:
            self.tag_state_source = TagStateSource(
                self.node,
                stale_after_sec=tag_state_timeout_sec,
            )

        if not self.node.has_parameter("navigation.map_root"):
            self.node.declare_parameter(
                "navigation.map_root",
                str(default_map_root()),
            )
        configured_map_root = str(
            self.node.get_parameter("navigation.map_root").value
        ).strip()
        map_root = configured_map_root or str(default_map_root())

        self.rtabmap_runtime = RtabmapRuntimeManager(
            node=self.node,
            blackboard=self.bb_client,
            maps_dir=map_root,
            launch_file="lidar_rtab_mapping_launch.py",
            nav2_launch_file="nav2_lidar_launch.py",
            nav2_params_file="nav2_lidar_params.yaml",
            raw_lidar_topic=LidarAdapterRuntime.OUTPUT_TOPIC,
        )

        self.nav2_runtime = self.rtabmap_runtime.nav2_runtime
        collision_control = self.robot_command_resources.get_arm_collision_control(self.node)
        self.robot_command_resources.get_moveit_arm_planner(self.node)
        self.lidar_adapter_runtime = LidarAdapterRuntime(
            self.node, self.bb_client, self.rtabmap_runtime, collision_control,
        )
        return True

    def initialise(self):
        pass

    def update(self):
        return py_trees.common.Status.SUCCESS

    def cleanup_operations(self):
        """Expose child runtimes separately for bounded ordered teardown."""
        operations = {}
        for name, method in (("rtabmap_runtime", "close"),
                             ("lidar_adapter_runtime", "close"),
                             ("tag_state_source", "destroy")):
            resource = getattr(self, name)
            if resource is None:
                continue

            def release(name=name, resource=resource, method=method):
                result = getattr(resource, method)()
                if result is False:
                    raise RuntimeError(f"{name} cleanup reported incomplete")
                if getattr(self, name) is resource:
                    setattr(self, name, None)
            operations[name] = release
        return operations
