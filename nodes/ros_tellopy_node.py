#!/usr/bin/env python

import threading

from geometry_msgs.msg import Twist, Vector3
import rospy
import ros_numpy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

from ros_tellopy.msg import RollPitchYaw, TelloState
from dji_tellopy.tello import Tello


class ROSTellopyNode(object):

    def __init__(self, rate=20., min_cmd_vel_freq=2.):
        self._rate = rate
        self._min_cmd_vel_dt = 1. / min_cmd_vel_freq

        ### Tello

        self._tello = Tello()

        ### ROS publishers
        self._camera_pub = rospy.Publisher('camera', Image, queue_size=1)
        self._state_pub = rospy.Publisher('state', TelloState, queue_size=1)

        ### ROS subscribers
        rospy.Subscriber('takeoff', Empty, self._takeoff_callback)
        rospy.Subscriber('land', Empty, self._land_callback)
        rospy.Subscriber('estop', Empty, self._estop_callback)
        self._cmd_vel = Twist()
        self._cmd_vel_stamp = rospy.Time.now()
        rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_callback)

        ### background threads
        threading.Thread(target=self._cmd_vel_thread).start()

    ###################
    ### ROS threads ###
    ###################

    def _takeoff_callback(self, msg):
        self._tello.takeoff()

    def _land_callback(self, msg):
        self._tello.land()

    def _estop_callback(self, msg):
        self._tello.estop()

    def _cmd_vel_callback(self, msg):
        self._cmd_vel = msg
        self._cmd_vel_stamp = rospy.Time.now()
        self._cmd_vel_thread_step(self._cmd_vel)

    ###############
    ### Threads ###
    ###############

    def _cmd_vel_thread(self):
        rate = rospy.Rate(10.)
        while not rospy.is_shutdown():
            rate.sleep()

            if (rospy.Time.now() - self._cmd_vel_stamp).to_sec() < self._min_cmd_vel_dt:
                msg = self._cmd_vel
            else:
                msg = Twist()
            self._cmd_vel_thread_step(msg)

    def _cmd_vel_thread_step(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.linear.z
        vyaw = msg.angular.z

        self._tello.set_velocity(vx, vy, vz, vyaw)

    ###########
    ### Run ###
    ###########

    def run(self):
        rate = rospy.Rate(self._rate)
        seq = 0

        while not rospy.is_shutdown():
            rate.sleep()

            image = self._tello.get_video_frame()
            if image is not None:
                image_msg = ros_numpy.msgify(Image, image, encoding='rgb8')
                image_msg.header.stamp = rospy.Time.now()
                image_msg.header.seq = seq
                self._camera_pub.publish(image_msg)

            state = self._tello.get_state()
            if state is not None:
                state_msg = TelloState(
                    acceleration=Vector3(*state['acceleration']),
                    velocity=Vector3(*state['velocity']),
                    rpy=RollPitchYaw(*state['rpy']),
                    battery=state['battery'],
                    barometer=state['barometer'],
                    height=state['height']
                )
                state_msg.header.stamp = rospy.Time.now()
                state_msg.header.seq = seq
                self._state_pub.publish(state_msg)

            if state is not None or image is not None:
                seq += 1



rospy.init_node('ROSTello', anonymous=True)
node = ROSTellopyNode()
try:
    node.run()
except KeyboardInterrupt:
    print('Shutting down ros_tellopy_node.py')
