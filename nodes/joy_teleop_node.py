#!/usr/bin/env python

from geometry_msgs.msg import Twist
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty


class JoyTeleopNode(object):

    LEFT_AXIS_X = 0
    LEFT_AXIS_Y = 1
    RIGHT_AXIS_X = 2
    RIGHT_AXIS_Y = 3
    X_BUTTON = 0
    L1_BUTTON = 4
    R1_BUTTON = 5

    def __init__(self):
        self._takeoff_pub = rospy.Publisher('takeoff', Empty, queue_size=1)
        self._land_pub = rospy.Publisher('land', Empty, queue_size=1)
        self._estop_pub = rospy.Publisher('estop', Empty, queue_size=1)
        self._cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self._cmd_vel = Twist()
        rospy.Subscriber('/joy', Joy, self._joy_callback)

    def _joy_callback(self, msg):
        if msg.buttons[JoyTeleopNode.X_BUTTON]:
            self._estop_pub.publish(Empty())
        elif msg.buttons[JoyTeleopNode.L1_BUTTON]:
            self._land_pub.publish(Empty())
        elif msg.buttons[JoyTeleopNode.R1_BUTTON]:
            self._takeoff_pub.publish(Empty())

        cmd_vel = Twist()
        cmd_vel.linear.x = msg.axes[JoyTeleopNode.RIGHT_AXIS_Y]
        cmd_vel.linear.y = -msg.axes[JoyTeleopNode.RIGHT_AXIS_X]
        cmd_vel.linear.z = msg.axes[JoyTeleopNode.LEFT_AXIS_Y]
        cmd_vel.angular.z = -msg.axes[JoyTeleopNode.LEFT_AXIS_X]
        self._cmd_vel = cmd_vel

    def run(self):
        rate = rospy.Rate(20.)

        while not rospy.is_shutdown():
            rate.sleep()

            self._cmd_vel_pub.publish(self._cmd_vel)

rospy.init_node('JoyTeleopNode', anonymous=True)

node = JoyTeleopNode()

try:
    node.run()
except KeyboardInterrupt:
    print('Shutting down ros_tellopy_node.py')
