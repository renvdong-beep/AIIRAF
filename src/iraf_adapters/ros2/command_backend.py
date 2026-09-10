"""ROS 2 command boundary. rclpy is injected by the process adapter."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class JointCommand:
    positions: dict
    duration_ms: int
    fencing_token: int

class Ros2CommandBackend:
    def __init__(self, publish_trajectory, publish_stop, authority, profile=None):
        self.publish_trajectory = publish_trajectory
        self.publish_stop = publish_stop
        self.authority = authority
        self.profile = profile
        self.last_command = None

    def move_joint(self, positions, duration_ms, lease):
        self.authority.validate(lease)
        if not hasattr(positions, "items") or not positions:
            raise ValueError("positions must be a non-empty mapping")
        normalized = {}
        allowed = set(getattr(self.profile, "joints", ()) or ())
        for joint, value in positions.items():
            if not isinstance(joint, str) or not joint:
                raise ValueError("joint names must be non-empty strings")
            if allowed and joint not in allowed:
                raise ValueError(f"unknown joints: {[joint]}")
            numeric = float(value)
            if not math.isfinite(numeric):
                raise ValueError(f"non-finite position for joint: {joint}")
            normalized[joint] = numeric
        duration = int(duration_ms)
        if duration <= 0:
            raise ValueError("duration_ms must be positive")
        command = JointCommand(normalized, duration, lease.fencing_token)
        self.publish_trajectory(command)
        self.last_command = command

    def stop(self, lease):
        self.authority.validate(lease)
        self.publish_stop(lease.fencing_token)
        self.last_command = None
