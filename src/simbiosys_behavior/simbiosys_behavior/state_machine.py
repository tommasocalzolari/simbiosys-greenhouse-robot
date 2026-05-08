from simbiosys_behavior.states import MissionState


class MissionStateMachine:
    """Small placeholder state machine for future mission behavior."""

    def __init__(self) -> None:
        self.current_state = MissionState.WAIT_FOR_OPERATOR

    def step(self) -> MissionState:
        """Return the active mode without running autonomous transitions yet.

        TODO: Replace this with real transition logic once operator commands,
        mapping status, perception results, and arm action results are defined.
        """
        return self.current_state

    def set_mode(self, mode_name: str) -> tuple[bool, str]:
        """Set a known first-period mode by name."""
        try:
            self.current_state = MissionState(mode_name)
        except ValueError:
            known_modes = ", ".join(mode.value for mode in MissionState)
            return False, f"Unknown mode '{mode_name}'. Known modes: {known_modes}"
        return True, f"Mode set to {self.current_state.value}"

    def set_error(self) -> None:
        """Move the mission manager into the error state."""
        self.current_state = MissionState.ERROR
