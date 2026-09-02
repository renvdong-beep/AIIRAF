"""ROS 2 command boundary. rclpy is injected by the process adapter."""
from dataclasses import dataclass

@dataclass(frozen=True)
class JointCommand:
    positions: dict
    duration_ms: int
    fencing_token: int

class Ros2CommandBackend:
    def __init__(self, publish_trajectory, publish_stop, authority):
        self.publish_trajectory = publish_trajectory
        self.publish_stop = publish_stop
        self.authority = authority
        self.last_command = None

    def move_joint(self, positions, duration_ms, lease):
        self.authority.validate(lease)
        command = JointCommand(dict(positions), int(duration_ms), lease.fencing_token)
        self.publish_trajectory(command)
        self.last_command = command

    def stop(self, lease):
        self.authority.validate(lease)
        self.publish_stop(lease.fencing_token)
        self.last_command = None
