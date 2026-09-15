"""Operational runtime state sources share one recognizable lifecycle type."""

from fault_detector_spot.inspection.sensing.probe_surface_source import (
    ProbeSurfaceSource,
)
from fault_detector_spot.manipulation.arm_joint_state_source import (
    ArmJointStateSource,
)
from fault_detector_spot.manipulation.arm_state_source import ArmStateSource
from fault_detector_spot.navigation.posture_state_source import (
    PostureStateSource,
)
from fault_detector_spot.sensing.tag_state_source import TagStateSource
from fault_detector_spot.shared.runtime_source import RuntimeSource


def test_operational_runtime_sources_share_runtime_source_parent():
    for source_type in (
        ArmStateSource,
        ArmJointStateSource,
        PostureStateSource,
        TagStateSource,
        ProbeSurfaceSource,
    ):
        assert issubclass(source_type, RuntimeSource)
        assert callable(getattr(source_type, "destroy", None))
