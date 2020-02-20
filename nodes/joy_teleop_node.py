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
        self._height_lims = [0.4, 1.2]

        self._joy_axes = [0,0,0,0]
        rospy.Subscriber('/joy', Joy, self._joy_callback)
        self._step_noise = False
        self._yaw_noise = StepNoise(**{'reset_step_range' : [2,40], 'step_range' : [-0.2, 0.2], 'range' : [-0.8, 0.8]})
        self._x_noise = StepNoise(**{'reset_step_range' : [20,40], 'step_range' : [-0.05, 0.05], 'range' : [0.3, 0.5]})

    def _joy_callback(self, msg):
        if msg.buttons[JoyTeleopNode.X_BUTTON]:
            self._estop_pub.publish(Empty())
        elif msg.buttons[JoyTeleopNode.L1_BUTTON]:
            self._land_pub.publish(Empty())
        elif msg.buttons[JoyTeleopNode.R1_BUTTON]:
            self._takeoff_pub.publish(Empty())

        self._joy_axes = msg.axes

    def run(self):
        rate = rospy.Rate(20.)

        while not rospy.is_shutdown():
            rate.sleep()
            cmd = CmdTello()
            if self._step_noise:
                cmd.vx = self._joy_axes[JoyTeleopNode.RIGHT_AXIS_Y] + self._x_noise.step()
                cmd.vy = 0
                cmd.height = 0.4
                cmd.vyaw = min(max(2*-self._joy_axes[JoyTeleopNode.LEFT_AXIS_X] + self._yaw_noise.step(), -0.8), 0.8)
            else:
                cmd.vx = self._joy_axes[JoyTeleopNode.RIGHT_AXIS_Y]
                cmd.vy = -self._joy_axes[JoyTeleopNode.RIGHT_AXIS_X]
                cmd.vyaw = -self._joy_axes[JoyTeleopNode.LEFT_AXIS_X]
                #dheight = 0.1 * self._joy_axes[JoyTeleopNode.LEFT_AXIS_Y]
                #cmd.height = self._des_height = np.clip(self._des_height + dheight, *self._height_lims)
                cmd.height = 0.4
                
            self._cmd_pub.publish(cmd)
            
            

class StepNoise():
    def __init__(self, **kwargs):
        self._reset_step_range = kwargs['reset_step_range'] #[2,5]
        self._step_range = kwargs['step_range'] #[-0.2, 0.2]
        self._range = kwargs['range'] #[-1, 1]

        self._curr_mean = None
        self._curr_reset_step = None
        self._t = 0
        self.reset()

    def step(self):
        if self._t >= self._curr_reset_step:
            self.reset()
        noise = self._curr_mean + np.random.uniform(*self._step_range)
        noise = np.clip(noise, *self._range)
        self._t += 1
        return noise

    def reset(self):
        self._t = 0
        self._curr_mean = np.random.uniform(*self._range)
        self._curr_reset_step = np.random.randint(*self._reset_step_range)

rospy.init_node('joy_teleop', anonymous=True)

use_joy = False

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
