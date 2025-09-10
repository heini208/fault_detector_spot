import py_trees

from fault_detector_spot.behaviour_tree.nodes.mapping.slam_toolbox_helper import SlamToolboxHelper


class EnableLocalization(py_trees.behaviour.Behaviour):
    def __init__(self, slam_helper: SlamToolboxHelper, name: str = "EnableLocalization"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.slam_helper = slam_helper
        self.blackboard.register_key("active_map_name", access=py_trees.common.Access.READ)
        self.launched_initialized = False

    def update(self) -> py_trees.common.Status:
        if not self.blackboard.active_map_name:
            self.feedback_message = "No active map set, cannot enable Localization"
            return py_trees.common.Status.FAILURE

        self.feedback_message = f"Launching Localization"
        if self.launched_initialized is False:
            self.launched_initialized = True
            self.slam_helper.start_localization()

        if self.slam_helper.nav2_helper.is_running():
            self.launched_initialized = False
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
