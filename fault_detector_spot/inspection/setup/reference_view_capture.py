"""Shared reference-view capture readiness error."""


class ReferenceViewCaptureNotReady(RuntimeError):
    """Indicate that the completed collection has no usable capture."""
