"""Expose base-camera tags as the authoritative tag set."""

from copy import deepcopy
from typing import Dict

import py_trees
from fault_detector_msgs.msg import TagElement


class MergeVisibleTagObservations(
    py_trees.behaviour.Behaviour
):
    """Keep hand detections diagnostic and motion-ineligible."""

    def __init__(
        self,
        name: str = "MergeVisibleTagObservations",
    ):
        """Create a base-only authoritative selector."""
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self._previous_sources: Dict[int, str] = {}

    def setup(self, **kwargs):
        """Register source inputs and authoritative outputs."""
        self.blackboard.register_key(
            key="base_tag_observations",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key="hand_tag_observations",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key="visible_tags",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="visible_tag_sources",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key="visible_tag_source_changed_ids",
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.visible_tags = {}
        self.blackboard.visible_tag_sources = {}
        self.blackboard.visible_tag_source_changed_ids = set()
        self._previous_sources = {}

    def update(self) -> py_trees.common.Status:
        """Publish base observations and never promote hand tags."""
        base_tags: Dict[int, TagElement] = getattr(
            self.blackboard,
            "base_tag_observations",
            {},
        ) or {}
        hand_tags: Dict[int, TagElement] = getattr(
            self.blackboard,
            "hand_tag_observations",
            {},
        ) or {}

        selected = deepcopy(base_tags)
        sources = {
            tag_id: "base"
            for tag_id in selected
        }
        changed_ids = {
            tag_id
            for tag_id in (
                set(self._previous_sources) | set(sources)
            )
            if self._previous_sources.get(tag_id)
            != sources.get(tag_id)
        }

        self._previous_sources = deepcopy(sources)
        self.blackboard.visible_tags = selected
        self.blackboard.visible_tag_sources = sources
        self.blackboard.visible_tag_source_changed_ids = (
            changed_ids
        )
        self.feedback_message = (
            f"Authoritative base tags: {sorted(selected)}; "
            f"diagnostic hand tags: {sorted(hand_tags)}"
        )
        return py_trees.common.Status.SUCCESS
