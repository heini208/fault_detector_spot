"""Reject stale snapshots shared by the probe and navigation setup clients."""


class SetupStateOrder:
    def __init__(self):
        self._revision = -1
        self._request_states = {}
        self._closed_contexts = set()

    def accept(self, state) -> bool:
        if state.context_id in self._closed_contexts:
            return False
        revision = int(state.revision)
        if revision < self._revision:
            return False
        if revision > self._revision:
            self._revision = revision
            self._request_states.clear()
        previous = self._request_states.get(state.request_id)
        terminal = {state.STATE_SUCCEEDED, state.STATE_FAILED, state.STATE_CANCELLED}
        if previous in terminal and state.state not in terminal:
            return False
        if previous == state.STATE_RUNNING and state.state == state.STATE_QUEUED:
            return False
        if state.request_id:
            self._request_states[state.request_id] = state.state
        return True

    def close_context(self, context_id):
        if context_id:
            self._closed_contexts.add(context_id)
        self._revision = -1
        self._request_states.clear()
