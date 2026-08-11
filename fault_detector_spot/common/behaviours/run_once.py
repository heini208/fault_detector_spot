import py_trees

class RunOnce(py_trees.decorators.Decorator):
    """
    Decorator that runs a child only once, then always returns SUCCESS.
    """
    def __init__(self, child: py_trees.behaviour.Behaviour, name="RunOnce"):
        super().__init__(name=name, child=child)
        self._has_run = False

    def initialise(self):
        # Initialise the child if it hasn't run yet
        if not self._has_run and self.decorated.status == py_trees.common.Status.INVALID:
            self.decorated.initialise()

    def update(self):
        if not self._has_run:
            status = self.decorated.tick_once()  # tick_once may return None
            # fallback in case tick_once returns None
            if status is None:
                status = self.decorated.status
            if status == py_trees.common.Status.SUCCESS:
                self._has_run = True
            return status
        else:
            return py_trees.common.Status.SUCCESS
