import py_trees

from fault_detector_spot.behaviour_tree.nodes.mapping.rtab_helper import RTABHelper


class EnableLocalization(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "EnableLocalization", launch_file: str = "localization_merged_launch.py"):
        super().__init__(name)
        self.launch_file = launch_file
        self.blackboard = self.attach_blackboard_client(name=name)

    def setup(self, **kwargs):
        self.rtab_helper = RTABHelper(kwargs.get("node"), self.blackboard)
        pass

    def update(self) -> py_trees.common.Status:
        if not self.blackboard.active_map_name:
            self.feedback_message = "No active map set, cannot enable Localization"
            return py_trees.common.Status.FAILURE

        if self.rtab_helper.is_rtabmap_running():
            self.feedback_message = "RTAB-Map detected, switching to Localization mode"
            self.rtab_helper.set_mode_localization()
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Launching {self.launch_file} for Localization"
            proc = self.rtab_helper.start_localization(launch_file=self.launch_file)
            if proc:
                self.blackboard.mapping_launch_process = proc
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

