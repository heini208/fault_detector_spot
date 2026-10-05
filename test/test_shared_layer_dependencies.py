"""Shared code must not import the application or a robot domain."""

import ast
from pathlib import Path


def test_shared_layer_has_no_upward_imports():
    """Reject shared modules that import higher-level application domains."""
    shared = Path(__file__).parents[1] / 'fault_detector_spot/shared'
    forbidden = tuple(
        f'fault_detector_spot.{domain}'
        for domain in (
            'application', 'inspection', 'mapping', 'navigation',
            'manipulation', 'ui',
        )
    )
    violations = []
    for path in shared.rglob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.ImportFrom):
                names = [node.module or '']
            elif isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            else:
                continue
            violations.extend(
                f'{path.name}: {name}'
                for name in names if name.startswith(forbidden)
            )
    assert not violations
