"""ROS 2 JointState adapter for the IRAF boundary."""
from dataclasses import dataclass
from time import time

@dataclass(frozen=True)
class RobotState:
    robot_profile: str
    timestamp_unix_ms: int
    positions: dict
    velocities: dict
    efforts: dict

class JointStateAdapter:
    def __init__(self, profile):
        self.profile = profile
        self._last_state = None

    def convert(self, message):
        names = list(message.name)
        positions = list(message.position)
        if len(names) != len(positions):
            raise ValueError("JointState name/position length mismatch")
        unknown = set(names) - set(self.profile.joints)
        if unknown:
            raise ValueError(f"unknown joints: {sorted(unknown)}")
        velocities = self._values(message, "velocity", len(names))
        efforts = self._values(message, "effort", len(names))
        state = RobotState(self.profile.name, int(time() * 1000),
                           dict(zip(names, positions)),
                           dict(zip(names, velocities)),
                           dict(zip(names, efforts)))
        self._last_state = state
        return state

    @staticmethod
    def _values(message, field, size):
        values = list(getattr(message, field, []) or [])
        return values if len(values) == size else [0.0] * size
