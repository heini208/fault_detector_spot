"""Keep each public command bound to its observed execution instance."""


class CommandStateOrder:
    def __init__(self):
        self._requests = {}

    def accept(self, state):
        request_id = state.request_id
        if not request_id:
            return True
        instance = state.execution_instance_id
        sequence = int(state.status_sequence)
        if instance and sequence <= 0:
            return False
        terminal = state.state in {
            state.STATE_SUCCEEDED, state.STATE_FAILED,
            state.STATE_CANCELLED, state.STATE_STOP_CONFIRMED,
        }
        previous = self._requests.get(request_id)
        if previous is not None:
            owner, last_sequence, finished = previous
            if owner and instance != owner:
                return False
            if last_sequence and sequence <= last_sequence:
                return False
            if finished and not terminal:
                return False
        self._requests[request_id] = (instance, sequence, terminal)
        return True
