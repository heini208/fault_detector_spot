import py_trees
from fault_detector_spot.behaviour_tree.nodes.mapping.rtab_helper import RTABHelper


class SwapMap(py_trees.behaviour.Behaviour):
    """
    Swap/load a map in RTAB-Map by restarting SLAM or localization with the new database.
    If SLAM_TOOLBOX-Map is not running, just set the active map and publish it.
    Updates blackboard variable `active_map_name`.
    """
    #TODO this can be replaced with a combination behaviour from get_map_path and EnableSLAM/EnableLocalization

    def __init__(self, slam_helper: RTABHelper, name="SwapMap"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.slam_helper = slam_helper

    def setup(self, **kwargs):
        self.blackboard.register_key("last_command", access=py_trees.common.Access.READ)
        return True

    def _validate_last_command(self):
        last_command = getattr(self.blackboard, "last_command", None)
        if not last_command or not getattr(last_command, "map_name", None):
            self.feedback_message = "No map_name in last_command"
            return None
        return last_command.map_name

    def update(self):
        requested_map = self._validate_last_command()
        if not requested_map:
            return py_trees.common.Status.FAILURE

        current_map = getattr(self.blackboard, "active_map_name", None)
        if requested_map == current_map:
            self.feedback_message = f"Map '{requested_map}' already active"
            return py_trees.common.Status.SUCCESS

        self.slam_helper.change_map(requested_map)
        return py_trees.common.Status.SUCCESS