import py_trees

from fault_detector_spot.behaviour_tree.nodes.mapping.slam_toolbox_helper import SlamToolboxHelper


class EnableLocalization(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "EnableLocalization", launch_file: str = "slam_sim_merged_launch.py"):
        super().__init__(name)
        self.launch_file = launch_file
        self.blackboard = self.attach_blackboard_client(name=name)

    def setup(self, **kwargs):
        self.slam_helper = SlamToolboxHelper(kwargs.get("node"), self.blackboard, launch_file=self.launch_file)
        pass

    def update(self) -> py_trees.common.Status:
        if not self.blackboard.active_map_name:
            self.feedback_message = "No active map set, cannot enable Localization"
            return py_trees.common.Status.FAILURE

        self.feedback_message = f"Launching {self.launch_file} for Localization"
        self.slam_helper.start_localization()
        if self.slam_helper.is_slam_running():
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

