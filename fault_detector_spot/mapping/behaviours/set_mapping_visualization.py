"""Change RViz visibility through the existing asynchronous runtime owner."""
import py_trees


class SetMappingVisualization(py_trees.behaviour.Behaviour):
    def __init__(self, runtime, name="SetMappingVisualization"):
        super().__init__(name)
        self.runtime = runtime
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key('last_command', access=py_trees.common.Access.READ)
        self._requested = False
        self._operation = f'mapping_visualization:{name}'

    def initialise(self):
        self._requested = False

    def update(self):
        try:
            if not self._requested:
                self._requested = self.runtime.begin_runtime_operation(
                    self._operation, self.runtime.set_visualization,
                    self.blackboard.last_command.mapping_rviz,
                )
                self.feedback_message = 'Updating mapping visualization'
                return py_trees.common.Status.RUNNING
            result = self.runtime.poll_runtime_operation(self._operation)
            if result is None:
                return py_trees.common.Status.RUNNING
            self.feedback_message = 'Mapping visualization updated'
            return py_trees.common.Status.SUCCESS if result else py_trees.common.Status.FAILURE
        except Exception as exception:
            self.feedback_message = f'Could not update mapping visualization: {exception}'
            return py_trees.common.Status.FAILURE
