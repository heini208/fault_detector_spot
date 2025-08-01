# fault_detector_spot/behaviour_tree/command_ids.py

from enum import Enum


class CommandID(str, Enum):
    STOW_ARM = "stow_arm"
    READY_ARM = "ready_arm"
    MOVE_ARM_TO_TAG = "move_to_tag"
    MOVE_ARM_TO_TAG_AND_WAIT = "move_tag_and_wait"
    MOVE_ARM_RELATIVE = "move_arm_relative"
    STAND_UP = "stand_up"
    WAIT_TIME = "wait_time"
    EMERGENCY_CANCEL = "cancel_all"
    SCAN_ALL_IN_RANGE = "scan_all_in_range"
    TOGGLE_GRIPPER = "toggle_gripper"
    CLOSE_GRIPPER = "close_gripper"
    ESTOP_STATE = "estop_state"
    STOP_BASE = "stop_base"
