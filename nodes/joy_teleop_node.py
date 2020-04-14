#!/usr/bin/env python

import numpy as np

from ros_tellopy.msg import CmdTello
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

from crazyflie.msg import CFCommand, CFMotion


class JoyTeleopNode(object):

    LEFT_AXIS_X = 0
    LEFT_AXIS_Y = 1
    RIGHT_AXIS_X = 2
    RIGHT_AXIS_Y = 3
    X_BUTTON = 0
    SEQ_BUTTON = 2 #B

    L1_BUTTON = 6
    R1_BUTTON = 7

    PICKUP_BUTTON = 4
    DROPOFF_BUTTON = 5

    def __init__(self, dt=0.1): # 10Hz default
        self._ros_prefix = "/cf/0/"

        self._cmd_pub = rospy.Publisher(self._ros_prefix + 'command', CFCommand, queue_size=1)
        self._extra_cmd_pub = rospy.Publisher(self._ros_prefix + 'extra_command', CFCommand, queue_size=1)
        self._mpc_extra_cmd_pub = rospy.Publisher(self._ros_prefix + 'mpc_extra_command', CFCommand, queue_size=1)
        self._motion_pub = rospy.Publisher(self._ros_prefix + 'motion', CFMotion, queue_size=1)

        self._dt = dt

        self._motion = CFMotion()
        self._motion.is_flow_motion=True
        rospy.Subscriber('/joy', Joy, self._joy_callback)

    def _joy_callback(self, msg):
        pub = False

        if msg.buttons[JoyTeleopNode.X_BUTTON]:
            pub = True; cmd = CFCommand.ESTOP

        elif msg.buttons[JoyTeleopNode.L1_BUTTON]:
            pub = True; cmd = CFCommand.LAND

        elif msg.buttons[JoyTeleopNode.R1_BUTTON]:
            pub = True; cmd = CFCommand.TAKEOFF

        if pub:
            cmd_msg = CFCommand()
            cmd_msg.cmd = cmd
            cmd_msg.stamp.stamp = rospy.Time.now()
            self._cmd_pub.publish(cmd_msg)

        if msg.buttons[JoyTeleopNode.SEQ_BUTTON]:
            cmd_msg2 = CFCommand()
            cmd_msg2.cmd = CFCommand.TAKEOFF
            cmd_msg2.stamp.stamp = rospy.Time.now()
            self._extra_cmd_pub.publish(cmd_msg2)

        if msg.buttons[JoyTeleopNode.PICKUP_BUTTON]:
            cmd_msg3 = CFCommand()
            cmd_msg3.cmd = CFCommand.TAKEOFF
            cmd_msg3.stamp.stamp = rospy.Time.now()
            self._mpc_extra_cmd_pub.publish(cmd_msg3)

        elif msg.buttons[JoyTeleopNode.DROPOFF_BUTTON]:
            cmd_msg3 = CFCommand()
            cmd_msg3.cmd = CFCommand.LAND
            cmd_msg3.stamp.stamp = rospy.Time.now()
            self._mpc_extra_cmd_pub.publish(cmd_msg3)


        # send control cmd (right now only vx, vy, vz is supported)
        motion = CFMotion()
        motion.is_flow_motion=True
        motion.x = msg.axes[JoyTeleopNode.RIGHT_AXIS_Y]
        motion.y = -msg.axes[JoyTeleopNode.RIGHT_AXIS_X]
        motion.yaw = -msg.axes[JoyTeleopNode.LEFT_AXIS_X]
        # dz here represents vz
        motion.dz = msg.axes[JoyTeleopNode.LEFT_AXIS_Y] # 20cm per second max

        self._motion = motion

    def run(self):
        rate = rospy.Rate(1./self._dt)

        while not rospy.is_shutdown():
            # print("publishing")
            self._motion_pub.publish(self._motion)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joy_teleop', anonymous=True)

    usejoyParam = rospy.search_param("use_joy")
    use_joy = bool(rospy.get_param(usejoyParam, 'True'))

    if not use_joy:
        # we don't need to run anything
        print("[JoyTeleop] Disabled.")
    else:
        print("[JoyTeleop] Starting.")
        node = JoyTeleopNode()
        try:
            node.run()
        except KeyboardInterrupt:
            print('[JoyTeleop] Shutting down')
