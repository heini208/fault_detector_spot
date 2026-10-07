"""Adapt the arm collision setting to MoveIt's occupancy collision rules."""

from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

from fault_detector_spot.manipulation.moveit_collision_policy import (
    occupancy_collision_matrix,
)


class MoveItCollisionScene:
    """Prepare occupancy policy requests; the planner owns their sequencing.

    This adapter neither supplies obstacle geometry nor changes it. Explicit
    objects, attached geometry, and self-collision rules remain in force.
    """

    def __init__(self, node, control):
        self.node = node
        self.control = control
        self.get_client = node.create_client(GetPlanningScene, "/get_planning_scene")
        self.apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._policy = None
        self._ignore_environment_collisions = True

    def prepare(self, ignore_environment_collisions):
        if type(ignore_environment_collisions) is not bool:
            raise TypeError("Ignore environmental collisions must be a boolean")
        self._policy = None if ignore_environment_collisions else self.control.state()
        self._ignore_environment_collisions = (
            ignore_environment_collisions or not self._policy.enabled
        )
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        return "read_scene", self.get_client, request

    def apply_request(self, response):
        self.validate()
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = occupancy_collision_matrix(
            response.scene.allowed_collision_matrix,
            ignore_environment_collisions=self._ignore_environment_collisions,
        )
        return "apply", self.apply_client, ApplyPlanningScene.Request(scene=scene)

    def validate(self):
        if self._policy is None:
            return
        current = self.control.state()
        if (current.enabled != self._policy.enabled
                or current.revision != self._policy.revision):
            raise RuntimeError(
                "Arm collision setting changed during planning; retry the movement"
            )

    def destroy(self):
        for client in (self.get_client, self.apply_client):
            self.node.destroy_client(client)
