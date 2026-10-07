"""Map service conversion and placement checks; the planner owns sequencing."""

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import math

from moveit_msgs.srv import ApplyPlanningScene
from octomap_msgs.srv import GetOctomap
from rclpy.time import Time
from std_srvs.srv import Empty

from fault_detector_spot.manipulation.rtabmap_octomap import snapshot_scene_diff
from fault_detector_spot.shared.geometry.rotation import rotation_distance_rad
from fault_detector_spot.shared.geometry.transforms import (
    inverse_pose, pose_data_to_pose, pose_to_pose_data,
)
from fault_detector_spot.shared.ros.tf_transforms import transform_to_pose_data


class RtabmapCollisionScene:
    """Import one active mapping session, or clear only MoveIt's occupancy.

    MoveIt's sensor updaters must be disabled and this application must be its
    only occupancy writer. No background updates or execution are performed here.
    Placement limits match the planning-only feasibility checks, allowing small
    body sway while rejecting base movement/localization jumps during planning.
    """

    MAX_TF_AGE_SEC = 1.5
    MAX_TRANSLATION_M = 0.02
    MAX_ROTATION_RAD = 0.03

    def __init__(self, node, tf_buffer, runtime):
        self.node = node
        self.tf_buffer = tf_buffer
        self.runtime = runtime
        self.map_client = node.create_client(GetOctomap, "/rtabmap/octomap_binary")
        self.apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.clear_client = node.create_client(Empty, "/clear_octomap")
        self._session = None
        self._reference = None
        self._policy = None
        self._conversion_worker = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="map_collision_conversion",
        )

    def prepare(self, ignore_environment_collisions):
        """Return the first service request; bypass needs neither mapping nor TF."""
        self._session = None
        self._reference = None
        self._policy = None
        if ignore_environment_collisions:
            return "clear", self.clear_client, Empty.Request()
        self._policy = self.runtime.collision_checking_state()
        if not self._policy.enabled:
            return "clear", self.clear_client, Empty.Request()
        self._session = self._policy.session
        if self._session is None:
            raise RuntimeError(
                "Map collision checking lost its active mapping session"
            )
        self._reference = self._placement()
        return "map", self.map_client, GetOctomap.Request()

    def start_import(self, response):
        """Convert occupancy outside the arm execution/force-monitor lock.

        The worker only builds a message; it cannot apply a scene or plan a
        motion. Cancellation can therefore discard its result safely.
        """
        self.validate()
        return self._conversion_worker.submit(
            snapshot_scene_diff, response.map, self._placement(),
        )

    def import_request(self, scene):
        """Recheck policy and placement before admitting the prepared scene."""
        self.validate()
        return "apply", self.apply_client, ApplyPlanningScene.Request(scene=scene)

    def validate(self):
        """Reject a checked plan if its session or body-relative placement changed."""
        if self._policy is not None:
            current_policy = self.runtime.collision_checking_state()
            if (current_policy.enabled != self._policy.enabled
                    or current_policy.revision != self._policy.revision):
                raise RuntimeError("Map collision setting changed during arm planning; retry the movement")
            if self._session is not None and current_policy.session != self._session:
                raise RuntimeError("Mapping stopped, switched, or restarted during arm planning")
        if self._session is None:
            return
        # Compare the body's position in map coordinates. Comparing body<-map
        # translations magnifies harmless rotation about a distant map origin.
        current = self._body_pose(self._placement())
        reference = self._body_pose(self._reference)
        displacement = math.dist(
            (current.position.x, current.position.y, current.position.z),
            (reference.position.x, reference.position.y, reference.position.z),
        )
        rotation = rotation_distance_rad(reference.orientation, current.orientation)
        if displacement > self.MAX_TRANSLATION_M or rotation > self.MAX_ROTATION_RAD:
            raise RuntimeError(
                "Map placement changed during arm planning "
                f"({displacement:.3f} m, {rotation:.3f} rad); keep the base stationary"
            )

    @staticmethod
    def _body_pose(transform):
        return pose_to_pose_data(inverse_pose(
            pose_data_to_pose(transform_to_pose_data(transform))
        ))

    def _placement(self):
        transform = self.tf_buffer.lookup_transform("body", "map", Time())
        stamp_ns = Time.from_msg(transform.header.stamp).nanoseconds
        age = (self.node.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if not -0.1 <= age <= self.MAX_TF_AGE_SEC:
            raise RuntimeError(f"Map placement TF is stale or future-dated ({age:.3f} s)")
        if stamp_ns < self._session.started_at_ns:
            raise RuntimeError("Map placement TF predates the current mapping session")
        transform_to_pose_data(transform)
        return deepcopy(transform)

    def destroy(self):
        self._conversion_worker.shutdown(wait=False, cancel_futures=True)
        for client in (self.map_client, self.apply_client, self.clear_client):
            self.node.destroy_client(client)
