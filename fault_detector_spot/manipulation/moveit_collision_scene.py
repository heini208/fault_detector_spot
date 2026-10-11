"""Prepare acknowledged head geometry and occupancy updates for MoveIt."""

from copy import deepcopy

from moveit_msgs.msg import ObjectColor, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from std_msgs.msg import ColorRGBA

from fault_detector_spot.manipulation.sensor_head_geometry import validate_attached_geometry
from fault_detector_spot.manipulation.moveit_collision_policy import (
    occupancy_collision_matrix,
    sensor_head_collision_matrix,
)


class MoveItCollisionScene:
    """Prepare scene requests; the planner remains the single scene writer."""

    def __init__(self, node, control, sensor_head=None):
        self.node = node
        self.control = control
        self.get_client = node.create_client(GetPlanningScene, "/get_planning_scene")
        self.apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._policy = None
        self._ignore_environment_collisions = True
        self.sensor_head = sensor_head

    def prepare(self, ignore_environment_collisions):
        if type(ignore_environment_collisions) is not bool:
            raise TypeError("Ignore environmental collisions must be a boolean")
        self._policy = None if ignore_environment_collisions else self.control.state()
        self._ignore_environment_collisions = (
            ignore_environment_collisions or not self._policy.enabled
        )
        if self.sensor_head is not None:
            return self.sensor_head.prepare()
        return self._read_request()

    def _read_request(self, phase="read_scene"):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        if self.sensor_head is not None:
            request.components.components |= (
                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                | PlanningSceneComponents.WORLD_OBJECT_NAMES
            )
        return phase, self.get_client, request

    def advance(self, phase, response):
        """Return the next request, or None after successful synchronization."""
        self.validate()
        if phase.startswith("head_"):
            request = self.sensor_head.advance(phase, response)
            if request is not None or phase == "head_verify":
                return request
            return self._read_request()
        if phase == "read_scene":
            return self.apply_request(response)
        if phase == "apply":
            if not response.success:
                raise RuntimeError("MoveIt rejected the collision scene update")
            return self.verification_request()
        if phase == "verify_scene":
            self._verify_geometry(response.scene)
            return self.sensor_head.verification_request()
        raise ValueError(f"Unknown collision scene phase: {phase}")

    def verification_request(self):
        if self.sensor_head is not None:
            return self._read_request("verify_scene")
        return None

    def apply_request(self, response):
        self.validate()
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = occupancy_collision_matrix(
            response.scene.allowed_collision_matrix,
            ignore_environment_collisions=self._ignore_environment_collisions,
        )
        if self.sensor_head is not None:
            head = self.sensor_head.expected
            if head is None:
                raise RuntimeError("Sensor collision geometry has not been prepared")
            scene.allowed_collision_matrix = sensor_head_collision_matrix(
                scene.allowed_collision_matrix, head.object.id,
                ignore_environment_collisions=self._ignore_environment_collisions,
            )
            # Attached ADD replaces this body and consumes any same-ID world
            # object. An additional world REMOVE would make MoveIt reject the diff.
            scene.robot_state.attached_collision_objects = [deepcopy(head)]
            scene.object_colors = [ObjectColor(
                id=head.object.id, color=ColorRGBA(r=0.1, g=0.8, b=0.4, a=0.8),
            )]
        return "apply", self.apply_client, ApplyPlanningScene.Request(scene=scene)

    def _verify_geometry(self, scene):
        expected = self.sensor_head.expected
        objects = [obj for obj in scene.robot_state.attached_collision_objects
                   if obj.object.id == expected.object.id]
        if len(objects) != 1 or any(
            obj.id == expected.object.id for obj in scene.world.collision_objects
        ):
            raise RuntimeError("MoveIt sensor head is missing, duplicated, or detached")
        validate_attached_geometry(objects[0], expected)
        # Verify occupancy for the newly attached body too. The default applies
        # when its name was absent from the original matrix.
        matrix = sensor_head_collision_matrix(
            scene.allowed_collision_matrix, expected.object.id,
            ignore_environment_collisions=self._ignore_environment_collisions,
        )
        if matrix != scene.allowed_collision_matrix:
            raise RuntimeError("MoveIt occupancy collision rules changed during planning")

    def validate(self):
        if self.sensor_head is not None:
            self.sensor_head.validate()
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
        if self.sensor_head is not None:
            self.sensor_head.destroy()
