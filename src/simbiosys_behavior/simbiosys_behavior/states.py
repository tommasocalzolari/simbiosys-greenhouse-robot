from enum import Enum


class MissionState(str, Enum):
    """First-period modes for the SimBioSys laptop-side coordinator."""

    WAIT_FOR_OPERATOR = "WAIT_FOR_OPERATOR"
    TELEOP = "TELEOP"
    MAPPING = "MAPPING"
    ARM_TEST = "ARM_TEST"
    AUTONOMOUS_IDLE = "AUTONOMOUS_IDLE"
    ERROR = "ERROR"
