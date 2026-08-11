import py_trees
from py_trees.common import Status


class ResetEstopFlag(py_trees.behaviour.Behaviour):

    def setup(self, **kwargs):
        self.bb = self.attach_blackboard_client()
        self.bb.register_key("estop_flag", access=py_trees.common.Access.WRITE)
        return True

    def update(self):
        self.bb.estop_flag = False
        return Status.SUCCESS
