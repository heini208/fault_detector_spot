import py_trees
from .rtab_helper import RTABHelper  # adjust import path


class DeleteWaypoint(py_trees.behaviour.Behaviour):
    """
    Deletes a waypoint from the active map using RTABHelper.delete_waypoint().
    Expects last_command.waypoint_name and last_command.map_name on the blackboard.
    """

    def __init__(self, name: str = "DeleteWaypoint"):
        super(DeleteWaypoint, self).__init__(name)
        self.bb = self.attach_blackboard_client()

    def setup(self, **kwargs):
        """Register required blackboard keys."""
        self.bb.register_key("last_command", access=py_trees.common.Access.READ)
        self.helper = RTABHelper(kwargs.get("node"), self.bb)

    def update(self) -> py_trees.common.Status:
        """Try to delete the waypoint specified in last_command."""
        if not self.bb.exists("last_command") or self.bb.last_command is None:
            self.feedback_message = "No last_command on blackboard"
            return py_trees.common.Status.FAILURE

        cmd = self.bb.last_command
        if not hasattr(cmd, "map_name") or cmd.map_name is None:
            self.feedback_message = "last_command has no map_name"
            return py_trees.common.Status.FAILURE

        if not hasattr(cmd, "waypoint_name") or cmd.waypoint_name is None:
            self.feedback_message = "last_command has no waypoint_name"
            return py_trees.common.Status.FAILURE

        try:
            success = self.helper.delete_waypoint(
                waypoint_name=cmd.waypoint_name,
                map_name=cmd.map_name,
            )
            if success:
                self.feedback_message = (
                    f"Deleted waypoint '{cmd.waypoint_name}' from map '{cmd.map_name}'"
                )
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = (
                    f"Waypoint '{cmd.waypoint_name}' not found in map '{cmd.map_name}'"
                )
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.feedback_message = f"Failed to delete waypoint: {e}"
            return py_trees.common.Status.FAILURE
