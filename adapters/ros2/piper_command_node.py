"""ROS 2 wrapper for the existing Piper MuJoCo controller."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class PiperRos2CommandNode(Node):
    def __init__(self, profile, topic="/joint_states"):
        super().__init__("iraf_piper_command")
        self.profile = profile
        self.publisher = self.create_publisher(JointState, topic, 10)
        self._positions = {joint: 0.0 for joint in profile.joints}

    def publish_trajectory(self, command):
        self._positions.update(command.positions)
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.profile.joints)
        message.position = [self._positions[j] for j in self.profile.joints]
        self.publisher.publish(message)

    def publish_stop(self, fencing_token):
        self.get_logger().warn(f"stop requested, fencing_token={fencing_token}")
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.profile.joints)
        message.position = [self._positions[j] for j in self.profile.joints]
        self.publisher.publish(message)

def main():
    raise RuntimeError("Construct PiperRos2CommandNode from the IRAF runtime")
