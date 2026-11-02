"""Lifecycle deadlines cannot disable reconciliation through invalid values."""

from dataclasses import fields
from types import SimpleNamespace

import pytest

from fault_detector_spot.application.commanding.command_lifecycle import CommandLifecycleConfig


@pytest.mark.parametrize("name", [field.name for field in fields(CommandLifecycleConfig)])
@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf"), -float("inf")])
def test_deadlines_reject_nonpositive_or_nonfinite_values(name, value):
    with pytest.raises(ValueError, match=name):
        CommandLifecycleConfig(**{name: value})


@pytest.mark.parametrize("name", [field.name for field in fields(CommandLifecycleConfig)])
@pytest.mark.parametrize("value", [True, False, "1", None, object()])
def test_deadlines_require_numbers_not_flags_or_strings(name, value):
    with pytest.raises(TypeError, match=name):
        CommandLifecycleConfig(**{name: value})


def test_node_configuration_preserves_overrides_and_declares_missing_defaults():
    values = {"command.heartbeat_timeout_sec": 8.0, "command.poll_period_sec": .25}
    node = SimpleNamespace(
        has_parameter=lambda name: name in values,
        declare_parameter=lambda name, value: values.setdefault(name, value),
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
    )
    config = CommandLifecycleConfig.from_node(node)
    assert config.heartbeat_timeout_sec == 8.0
    assert config.poll_period_sec == .25
    assert config.ack_timeout_sec == 3.0
    assert config.query_timeout_sec == 2.0
    assert config.reconciliation_interval_sec == 1.0
