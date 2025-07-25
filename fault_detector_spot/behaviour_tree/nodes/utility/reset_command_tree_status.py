# In fault_detector_spot/behaviour_tree/nodes/reset_command_tree_status.py

import py_trees
from py_trees.common import Status

class ResetCommandTreeStatus(py_trees.behaviour.Behaviour):

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client()
        self.bb.register_key("command_tree_status", access=py_trees.common.Access.WRITE)
        return True

    def update(self):
        self.bb.command_tree_status = None
        return Status.SUCCESS
