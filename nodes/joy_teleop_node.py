#!/usr/bin/env python

import numpy as np

from ros_tellopy.msg import CmdTello
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
        self._cmd_pub = rospy.Publisher('cmd', CmdTello, queue_size=1)

        self._des_height = 0.8
        self._height_lims = [0.3, 1.2]

        self._cmd = CmdTello()
        rospy.Subscriber('/joy', Joy, self._joy_callback)

    def _joy_callback(self, msg):
        if msg.buttons[JoyTeleopNode.X_BUTTON]:
            self._estop_pub.publish(Empty())
        elif msg.buttons[JoyTeleopNode.L1_BUTTON]:
            self._land_pub.publish(Empty())
        elif msg.buttons[JoyTeleopNode.R1_BUTTON]:
            self._takeoff_pub.publish(Empty())

        cmd = CmdTello()
        cmd.vx = msg.axes[JoyTeleopNode.RIGHT_AXIS_Y]
        cmd.vy = -msg.axes[JoyTeleopNode.RIGHT_AXIS_X]
        cmd.vyaw = -msg.axes[JoyTeleopNode.LEFT_AXIS_X]
        dheight = 0.1 * msg.axes[JoyTeleopNode.LEFT_AXIS_Y]
        cmd.height = self._des_height = np.clip(self._des_height + dheight, *self._height_lims)
        # cmd.height = 0.2 * msg.axes[JoyTeleopNode.LEFT_AXIS_Y] # TODO

        self._cmd = cmd

    def run(self):
        rate = rospy.Rate(20.)

        while not rospy.is_shutdown():
            rate.sleep()

            self._cmd_pub.publish(self._cmd)

rospy.init_node('JoyTeleopNode', anonymous=True)

node = JoyTeleopNode()

try:
    node.run()
except KeyboardInterrupt:
    print('Shutting down ros_tellopy_node.py')
