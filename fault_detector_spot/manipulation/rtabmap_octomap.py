"""Validate RTAB-Map binary occupancy and place it in a MoveIt scene diff."""

from copy import deepcopy
import math

from moveit_msgs.msg import PlanningScene

from fault_detector_spot.shared.geometry.transforms import pose_data_to_pose
from fault_detector_spot.shared.ros.tf_transforms import transform_to_pose_data


def validate_binary_occupancy(data):
    """Check OcTree's depth-first, two-byte node records before native decoding.

    ColorOcTree and RTAB-Map's subtype share this occupancy-only binary format.
    Child pairs mean absent, free, occupied, or branch; maximum depth is 16.
    This checks framing, not voxel geometry, and never accepts full tree data.
    """
    def read_node(offset, depth):
        if depth >= 16 or offset + 2 > len(data):
            raise ValueError("Truncated or over-depth binary OctoMap")
        word = (int(data[offset]) & 255) | ((int(data[offset + 1]) & 255) << 8)
        if word == 0:
            raise ValueError("Binary OctoMap contains an empty node")
        offset += 2
        for child in range(8):
            if (word >> (2 * child)) & 3 == 3:
                offset = read_node(offset, depth + 1)
        return offset

    if not data:
        raise ValueError("RTAB-Map returned no occupancy data")
    if read_node(0, 0) != len(data):
        raise ValueError("Trailing bytes in binary OctoMap")


def snapshot_scene_diff(snapshot, body_from_map, *, map_frame="map"):
    """Keep voxel coordinates intact and position them using current body <- map.

    Only binary encoding permits ColorOcTree -> OcTree normalization. Explicit
    objects, attachments, robot state, and collision rules are left untouched.
    """
    if not snapshot.binary or snapshot.id not in ("ColorOcTree", "OcTree"):
        raise ValueError("Expected binary ColorOcTree or OcTree occupancy")
    if not math.isfinite(snapshot.resolution) or snapshot.resolution <= 0.0:
        raise ValueError("OctoMap resolution must be positive and finite")
    if not map_frame or snapshot.header.frame_id != map_frame:
        raise ValueError(f"Expected OctoMap frame {map_frame!r}")
    if (body_from_map.header.frame_id != "body"
            or body_from_map.child_frame_id != map_frame):
        raise ValueError("Expected body <- map placement transform")
    origin = pose_data_to_pose(transform_to_pose_data(body_from_map))
    validate_binary_occupancy(snapshot.data)

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.world.octomap.header = deepcopy(body_from_map.header)
    scene.world.octomap.origin = origin
    scene.world.octomap.octomap = deepcopy(snapshot)
    scene.world.octomap.octomap.id = "OcTree"
    return scene
