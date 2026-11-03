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
        terminal_states = {
            state.STATE_SUCCEEDED, state.STATE_FAILED,
            state.STATE_CANCELLED, state.STATE_STOP_CONFIRMED,
        }
        previous = self._requests.get(request_id)
        if previous is not None:
            owner, last_sequence, previous_state = previous
            if owner and instance != owner:
                return False
            if last_sequence and sequence <= last_sequence:
                return False
            if previous_state in terminal_states and state.state not in terminal_states:
                return False
        self._requests[request_id] = (instance, sequence, state.state)
        return True

    def is_duplicate(self, state):
        """Recognize an already validated status without accepting a new update."""
        return self._requests.get(state.request_id) == (
            state.execution_instance_id, int(state.status_sequence), state.state,
        )
