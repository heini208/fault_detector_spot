import py_trees

from fault_detector_spot.behaviour_tree.nodes.mapping.slam_toolbox_helper import SlamToolboxHelper


class EnableSLAM(py_trees.behaviour.Behaviour):
    def __init__(self,slam_helper: SlamToolboxHelper ,name: str = "EnableSLAM"):
        super().__init__(name)
        self.slam_helper = slam_helper
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key("active_map_name", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        if not self.blackboard.active_map_name:
            self.feedback_message = "No active map set, cannot enable SLAM"
            return py_trees.common.Status.FAILURE

        self.feedback_message = f"Launching Mapping"
        self.slam_helper.start_mapping_from_existing()
        if self.slam_helper.is_slam_running():
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
