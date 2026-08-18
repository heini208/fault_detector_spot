"""Behavior classes composed by the main application tree."""

from fault_detector_spot.application.behaviour_tree.behaviours.command_manager import CommandManager
from fault_detector_spot.application.behaviour_tree.behaviours.helper_initializer import HelperInitializer
from fault_detector_spot.application.behaviour_tree.behaviours.new_command_guard import NewCommandGuard
from fault_detector_spot.application.behaviour_tree.behaviours.reset_estop_flag import ResetEstopFlag
from fault_detector_spot.application.behaviour_tree.behaviours.spot_action import SimpleSpotAction
from fault_detector_spot.application.behaviour_tree.behaviours.wait_for_duration import WaitForDuration
from fault_detector_spot.inspection.behaviours.publish_live_inspection_object import (
    PublishLiveInspectionObject,
)
from fault_detector_spot.inspection.behaviours.resolve_live_inspection_object import (
    ResolveLiveInspectionObject,
)
from fault_detector_spot.manipulation.behaviours.close_gripper_action import CloseGripperAction
from fault_detector_spot.manipulation.behaviours.manipulator_get_goal_tag import (
    ManipulatorGetGoalTag,
)
from fault_detector_spot.manipulation.behaviours.manipulator_move_arm_action import (
    ManipulatorMoveArmAction,
)
from fault_detector_spot.manipulation.behaviours.manipulator_move_close_to_surface_action import (
    ManipulatorMoveCloseToSurfaceAction,
)
from fault_detector_spot.manipulation.behaviours.manipulator_move_relative_action import (
    ManipulatorMoveRelativeAction,
)
from fault_detector_spot.manipulation.behaviours.ready_arm_action import ReadyArmActionSimple
from fault_detector_spot.manipulation.behaviours.stand_up_action import StandUpActionSimple
from fault_detector_spot.manipulation.behaviours.stow_arm_action import StowArmActionSimple
from fault_detector_spot.manipulation.behaviours.toggle_gripper_action import ToggleGripperAction
from fault_detector_spot.mapping.behaviours.enable_localization import EnableLocalization
from fault_detector_spot.mapping.behaviours.enable_slam import EnableSLAM
from fault_detector_spot.mapping.behaviours.stop_mapping import StopMapping
from fault_detector_spot.mapping.behaviours.swap_map import SwapMap
from fault_detector_spot.navigation.behaviours.cancel_movement import PublishZeroVel
from fault_detector_spot.navigation.behaviours.landmark_relocalizer import LandmarkRelocalizer
from fault_detector_spot.navigation.behaviours.move_base.base_get_goal_tag import BaseGetGoalTag
from fault_detector_spot.navigation.behaviours.move_base.base_move_relative_action import (
    BaseMoveRelativeAction,
)
from fault_detector_spot.navigation.behaviours.move_base.base_move_to_tag_action import (
    BaseMoveToTagAction,
)
from fault_detector_spot.navigation.behaviours.navigate_to_goal_pose import NavigateToGoalPose
from fault_detector_spot.navigation.behaviours.set_waypoint_as_goal import SetWaypointAsGoal
from fault_detector_spot.application.behaviour_tree.behaviours.buffer_and_status_publisher import (
    BufferStatusPublisher,
)
from fault_detector_spot.sensing.behaviours.check_tag_reachability import CheckTagReachability
from fault_detector_spot.application.behaviour_tree.behaviours.command_subscriber import CommandSubscriber
from fault_detector_spot.sensing.behaviours.tag_state_subscriber import (
    TagStateSubscriber,
)
from fault_detector_spot.sensing.behaviours.visible_tag_publisher import (
    PublishReachableTags,
)
from fault_detector_spot.sensing.behaviours.visible_tag_to_map import VisibleTagToMap

__all__ = [
    "CommandSubscriber",
    "ManipulatorGetGoalTag",
    "ManipulatorMoveArmAction",
    "ManipulatorMoveCloseToSurfaceAction",
    "ReadyArmActionSimple",
    "StowArmActionSimple",
    "NewCommandGuard",
    "StandUpActionSimple",
    "CheckTagReachability",
    "PublishZeroVel",
    "CommandManager",
    "ResetEstopFlag",
    "WaitForDuration",
    "BufferStatusPublisher",
    "ManipulatorMoveRelativeAction",
    "TagStateSubscriber",
    "PublishReachableTags",
    "ToggleGripperAction",
    "CloseGripperAction",
    "EnableSLAM",
    "SwapMap",
    "EnableLocalization",
    "StopMapping",
    "SetWaypointAsGoal",
    "NavigateToGoalPose",
    "HelperInitializer",
    "VisibleTagToMap",
    "LandmarkRelocalizer",
    "BaseGetGoalTag",
    "BaseMoveToTagAction",
    "BaseMoveRelativeAction",
    "PublishLiveInspectionObject",
    "ResolveLiveInspectionObject",
]
