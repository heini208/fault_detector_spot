# fault_detector_spot/behaviour_tree/command_ids.py

from enum import Enum

class CommandID(str, Enum):
    STOW_ARM         = "stow_arm"
    READY_ARM        = "ready_arm"
    MOVE_TO_TAG      = "move_to_tag"
    STAND_UP         = "stand_up"
    EMERGENCY_CANCEL = "stow_arm"
