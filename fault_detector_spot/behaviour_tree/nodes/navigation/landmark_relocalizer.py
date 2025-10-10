import math
import typing

import numpy as np

import py_trees
import rclpy
import tf2_ros
from fault_detector_spot.behaviour_tree.nodes.mapping.rtab_helper import RTABHelper
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.time import Time


class LandmarkRelocalizer(py_trees.behaviour.Behaviour):
    """Behaviour that uses AprilTag landmarks (from a JSON map) to publish `/initialpose` for AMCL.

    Behaviour logic:
    - Load landmarks for the active map into a mapping: tag_id (int) -> geometry_msgs/Pose
    - Keep track of which tags have already triggered a relocalization (self.seen_tags).
    - On each update, determine currently visible tag ids from blackboard.visible_tags_map_frame.
      * For tags that are visible now but not in self.seen_tags: compute corrected robot pose and publish `/initialpose`.
      * For tags that were in self.seen_tags but are no longer visible: remove them so they can trigger again later.

    The computation uses TF to obtain the current (observed) pose of the tag in the map frame
    (available in `visible_tags_map_frame`) and the current robot pose in the map frame
    (via tf lookup of base_link in the map frame). It computes the corrective transform that moves
    the observed tag pose to the true tag pose and applies that same corrective transform to the
    robot's observed pose to get the corrected robot pose.
    """

    def __init__(self, slam_helper: RTABHelper, node=None, base_frame: str = "base_link", name="LandmarkRelocalizer"):
        super().__init__(name)
        self.slam_helper = slam_helper
        self.node = node
        self.blackboard = self.attach_blackboard_client()

        # landmark_map: int tag_id -> geometry_msgs.msg.Pose (pose in map frame from JSON)
        self.landmark_map: typing.Dict[int, PoseStamped] = {}
        self.seen_tags: typing.Set[int] = set()
        self.last_published_pose: typing.Optional[PoseStamped] = None

        # config
        self.base_frame = base_frame
        self.map_frame = "map"
        # how long to wait for TF lookups
        self.tf_timeout = Duration(seconds=0.25)

    def setup(self, **kwargs: typing.Any) -> None:
        self.node = kwargs.get("node", self.node)
        self.blackboard.register_key("visible_tags", access=py_trees.common.Access.READ)
        self.blackboard.register_key("visible_tags_map_frame", access=py_trees.common.Access.READ)
        # expose active_map_name from the slam helper via blackboard if available
        self.blackboard.register_key("active_map_name", access=py_trees.common.Access.READ)
        self.initialpose_pub = self.node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    # ---- Helper math (2D) ----
    def _quat_to_yaw(self, qx, qy, qz, qw):
        # Try to use a standard conversion; keep a direct formula fallback
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))  (for quaternion x,y,z,w)
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _pose_to_mat(self, pose: typing.Any) -> np.ndarray:
        """Convert a geometry_msgs/Pose-like object to a 3x3 2D homogeneous matrix (x,y,yaw)."""
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        c = math.cos(yaw)
        s = math.sin(yaw)
        mat = np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])
        return mat

    def _mat_to_pose(self, mat: np.ndarray, pose_out: typing.Any) -> None:
        """Fill pose_out (geometry_msgs/Pose) from 3x3 homogeneous matrix (2D)."""
        x = float(mat[0, 2])
        y = float(mat[1, 2])
        # extract yaw
        yaw = math.atan2(mat[1, 0], mat[0, 0])
        # convert yaw to quaternion (x,y,z,w)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose_out.position.x = x
        pose_out.position.y = y
        pose_out.position.z = 0.0
        pose_out.orientation.x = 0.0
        pose_out.orientation.y = 0.0
        pose_out.orientation.z = qz
        pose_out.orientation.w = qw

    # ---- Landmark loading/parsing ----
    def _load_landmarks_for_map(self, map_name: str) -> None:
        """Load landmarks from the map JSON into self.landmark_map.

        The helper `get_list_from_json_category` in your setup only returns *names*.
        Here we try several fallbacks to get the actual JSON file and parse full landmark
        entries (dicts with 'name' and 'pose').

        Strategy:
        - If slam_helper exposes a `_json_path(map_name)` method (used by your helper), call it.
        - If not, try `get_json_path` or an attribute like `json_path` / `maps_dir` heuristically.
        - If we obtain a valid json path, open it and parse the 'landmarks' list (full dicts).
        - Otherwise we fall back to trying `get_list_from_json_category` and bail with a helpful
          feedback message (since only names are available and no poses can be read).
        """
        self.landmark_map.clear()

        json_path = self.slam_helper.get_json_path(map_name)
        if json_path:
            try:
                import os, json

                if os.path.exists(json_path):
                    with open(json_path, "r") as f:
                        data = json.load(f)
                    raw_list = data.get("landmarks", [])
                else:
                    # file not present
                    self.feedback_message = f"Landmark JSON not found at {json_path}"
                    return
            except Exception as ex:
                self.feedback_message = f"Failed to read landmark JSON: {ex}"
                return

        # At this point raw_list should be a list of dicts (full entries) or a list of names.
        if not raw_list:
            self.feedback_message = "No landmarks found in JSON"
            return

        # If the entries are just strings (names) we cannot recover poses here
        first = raw_list[0]
        if isinstance(first, str):
            self.feedback_message = "Landmark helper only returned names (no poses). Cannot load poses."
            return

        # parse dict entries
        for entry in raw_list:
            try:
                name = entry.get("name")
                if not name:
                    continue
                # parse id from 'Tag_5' -> 5
                tag_id = int(str(name).split("_")[-1])
                # convert dict pose to a PoseStamped-like object
                pose_dict = entry.get("pose")
                if not pose_dict:
                    continue
                p = PoseStamped()
                p.header.frame_id = self.map_frame
                # position
                p.pose.position.x = float(pose_dict["position"]["x"])
                p.pose.position.y = float(pose_dict["position"]["y"])
                p.pose.position.z = float(pose_dict["position"].get("z", 0.0))
                # orientation
                q = pose_dict.get("orientation", {})
                p.pose.orientation.x = float(q.get("x", 0.0))
                p.pose.orientation.y = float(q.get("y", 0.0))
                p.pose.orientation.z = float(q.get("z", 0.0))
                p.pose.orientation.w = float(q.get("w", 1.0))
                self.landmark_map[tag_id] = p
            except Exception:
                # ignore malformed entries but continue parsing the rest
                continue

        if not self.landmark_map:
            self.feedback_message = "No valid landmark poses parsed from JSON"
            return
        # success
        self.feedback_message = f"Loaded {len(self.landmark_map)} landmarks from JSON"

    # ---- equality helper for published pose (2D) ----"}]}
    def poses_equal(self, p1, p2, tol=1e-3):
        dx = abs(p1.position.x - p2.position.x)
        dy = abs(p1.position.y - p2.position.y)
        # compare yaw
        yaw1 = self._quat_to_yaw(p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w)
        yaw2 = self._quat_to_yaw(p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w)
        dyaw = abs(math.atan2(math.sin(yaw1 - yaw2), math.cos(yaw1 - yaw2)))
        return dx < tol and dy < tol and dyaw < (tol * 10)

    def update(self):
        # basic preconditions
        if not self.slam_helper.nav2_helper.is_running():
            self.feedback_message = "Localization not running"
            self.seen_tags: typing.Set[int] = set()
            return py_trees.common.Status.SUCCESS

        map_name = getattr(self.blackboard, 'active_map_name', None) or getattr(getattr(self.slam_helper, 'bb', None),
                                                                                'active_map_name', None)
        if not map_name:
            self.feedback_message = "No active map"
            return py_trees.common.Status.SUCCESS

        # load landmarks if needed
        self._load_landmarks_for_map(map_name)
        if not self.landmark_map:
            self.feedback_message = "No landmarks for active map"
            return py_trees.common.Status.SUCCESS

        # check blackboard for visible tags (map-frame and body-frame)
        if not self.blackboard.exists("visible_tags_map_frame") or self.blackboard.visible_tags_map_frame is None:
            self.feedback_message = "No tags in map frame"
            self.seen_tags: typing.Set[int] = set()
            return py_trees.common.Status.SUCCESS

        visible_map = self.blackboard.visible_tags_map_frame  # expected: dict tag_id -> TagElement (with .pose as PoseStamped)
        if not isinstance(visible_map, dict):
            self.feedback_message = "visible_tags_map_frame not a dict"
            return py_trees.common.Status.SUCCESS

        visible_ids = set(int(k) for k in visible_map.keys())

        # remove tags from seen set that are no longer visible so they can retrigger later
        for seen in list(self.seen_tags):
            if seen not in visible_ids:
                self.seen_tags.remove(seen)

        # find tags that are new (visible now but not seen before)
        new_tags = [tid for tid in visible_ids if tid not in self.seen_tags and tid in self.landmark_map]

        if not new_tags:
            self.feedback_message = "No new visible landmark"
            return py_trees.common.Status.SUCCESS

        # process the first new tag (could be extended to choose best tag or fuse several)
        for tag_id in new_tags:
            try:
                tag_element = visible_map.get(tag_id)
                if not tag_element or not getattr(tag_element, "pose", None):
                    self.feedback_message = "tag has no pose"
                    continue

                # observed tag pose in map frame (PoseStamped)
                tag_obs = PoseStamped()
                tag_obs.header = tag_element.pose.header
                tag_obs.pose = tag_element.pose.pose

                # true tag pose from JSON (PoseStamped in map frame)
                tag_true = self.landmark_map.get(tag_id)
                if not tag_true:
                    self.feedback_message = "no true tag"
                    continue

                # lookup current robot pose (base_frame) in map
                try:
                    t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time(),
                                                        self.tf_timeout)
                except Exception:
                    # fallback: use 'now' if timestamped lookup failed
                    try:
                        t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time(seconds=0).to_msg())
                    except Exception as ex:
                        self.feedback_message = f"TF lookup for robot in map failed: {ex}"
                        continue

                # construct Pose for robot observed from transform
                robot_obs = PoseStamped()
                robot_obs.header = t.header
                tr = t.transform.translation
                rot = t.transform.rotation
                robot_obs.pose.position.x = tr.x
                robot_obs.pose.position.y = tr.y
                robot_obs.pose.position.z = tr.z
                robot_obs.pose.orientation.x = rot.x
                robot_obs.pose.orientation.y = rot.y
                robot_obs.pose.orientation.z = rot.z
                robot_obs.pose.orientation.w = rot.w

                # compute correction transform: T_delta = T_map_tag_true * inv(T_map_tag_obs)
                M_true = self._pose_to_mat(tag_true.pose)
                M_obs = self._pose_to_mat(tag_obs.pose)
                # use 3x3 matrices
                try:
                    M_delta = M_true.dot(np.linalg.inv(M_obs))
                except Exception as ex:
                    self.feedback_message = f"Matrix math error computing correction: {ex}"
                    continue

                # apply delta to current robot observed pose
                M_robot_obs = self._pose_to_mat(robot_obs.pose)
                M_robot_corrected = M_delta.dot(M_robot_obs)

                # write corrected pose into message
                corrected_pose = PoseWithCovarianceStamped()
                corrected_pose.header.frame_id = self.map_frame
                corrected_pose.header.stamp = self.node.get_clock().now().to_msg()
                # fill pose
                self._mat_to_pose(M_robot_corrected, corrected_pose.pose.pose)

                # set a sensible covariance (6x6 flattened row-major). We only set x,y,yaw entries.
                cov = [0.0] * 36
                cov[0] = 0.25  # variance x
                cov[7] = 0.25  # variance y
                cov[35] = 0.5  # variance yaw
                corrected_pose.pose.covariance = cov

                # avoid republishing the exact same pose repeatedly
                if self.last_published_pose and self.poses_equal(self.last_published_pose, corrected_pose.pose.pose):
                    self.feedback_message = f"Corrected pose equal to last published for tag {tag_id}"
                    # mark tag as seen so it won't retrigger immediately
                    self.seen_tags.add(tag_id)
                    return py_trees.common.Status.SUCCESS

                self.initialpose_pub.publish(corrected_pose)
                self.last_published_pose = corrected_pose.pose.pose
                self.seen_tags.add(tag_id)
                self.feedback_message = f"Published initialpose from tag {tag_id}"
                return py_trees.common.Status.SUCCESS

            except Exception as e:
                # continue with the next tag if anything fails
                self.feedback_message = f"Error processing tag {tag_id}: {e}"
                continue
        return py_trees.common.Status.SUCCESS