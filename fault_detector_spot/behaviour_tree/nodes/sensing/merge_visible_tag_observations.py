from copy import deepcopy
from typing import Dict

import py_trees
from fault_detector_msgs.msg import TagElement


class MergeVisibleTagObservations(
    py_trees.behaviour.Behaviour
):

    def __init__(
        self,
        name: str = "MergeVisibleTagObservations",
        preferred_source: str = "base",
    ):
        super().__init__(name)

        if preferred_source not in {"base", "hand"}:
            raise ValueError(
                "Preferred source must be base or hand"
            )

        self.preferred_source = preferred_source
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs):
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
        self.blackboard.visible_tags = {}

    def update(self) -> py_trees.common.Status:
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

        if self.preferred_source == "base":
            merged = deepcopy(hand_tags)
            merged.update(deepcopy(base_tags))
        else:
            merged = deepcopy(base_tags)
            merged.update(deepcopy(hand_tags))

        self.blackboard.visible_tags = merged
        self.feedback_message = (
            f"Merged {len(base_tags)} base and "
            f"{len(hand_tags)} hand observations into "
            f"{len(merged)} visible tags"
        )

        return py_trees.common.Status.SUCCESS