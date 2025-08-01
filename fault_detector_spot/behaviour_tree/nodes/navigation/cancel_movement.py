from geometry_msgs.msg import Twist
import py_trees
from py_trees.common import Status
import rclpy


class PublishZeroVel(py_trees.behaviour.Behaviour):
    def __init__(self, name = "Stop_Base"):
        super().__init__(name)
        self.cmd_vel_pub = None
        self.node = None

    def setup(self, node: rclpy.node.Node) :
        # create a latched publisher (or normal) to /cmd_vel
        self.node = node
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        return True

    def update(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.linear.y = 0.0
        stop.angular.z = 0.0
        self.cmd_vel_pub.publish(stop)
        return Status.SUCCESS
