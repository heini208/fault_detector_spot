"""Validated timing policy for command receipt, liveness, and reconciliation."""

from dataclasses import dataclass, fields
from math import isfinite
from numbers import Real


@dataclass(frozen=True)
class CommandLifecycleConfig:
    ack_timeout_sec: float = 3.0
    heartbeat_timeout_sec: float = 5.0
    reconciliation_interval_sec: float = 1.0
    query_timeout_sec: float = 2.0
    poll_period_sec: float = 0.1

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, Real):
                raise TypeError(f"{field.name} must be a finite positive number")
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{field.name} must be a finite positive number")
            object.__setattr__(self, field.name, float(value))

    @classmethod
    def from_node(cls, node):
        """Read declared ROS parameters without importing ROS into the policy."""
        values = {}
        for field in fields(cls):
            name = f"command.{field.name}"
            if not node.has_parameter(name):
                node.declare_parameter(name, field.default)
            values[field.name] = node.get_parameter(name).value
        return cls(**values)
