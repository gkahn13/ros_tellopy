#!/usr/bin/env python

import av
import numpy as np
import threading
import time

from geometry_msgs.msg import Vector3
import rospy
import ros_numpy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Float32

from ros_tellopy.msg import CmdTello, RollPitchYaw, TelloState
from dji_tellopy import Tello


class ROSTellopyNode(object):

    def __init__(self, min_cmd_freq=2.):
        self._min_cmd_dt = 1. / min_cmd_freq

        ### ROS publishers
        self._camera_pub = rospy.Publisher('camera', Image, queue_size=1)
        self._state_msg = TelloState()
        self._state_seq = 0
        self._state_pub = rospy.Publisher('state', TelloState, queue_size=1)

        ### ROS subscribers
        rospy.Subscriber('takeoff', Empty, self._takeoff_callback)
        rospy.Subscriber('land', Empty, self._land_callback)
        rospy.Subscriber('estop', Empty, self._estop_callback)
        self._cmd = self._default_cmd
        self._cmd_stamp = rospy.Time.now()
        rospy.Subscriber('cmd', CmdTello, self._cmd_callback)

        ### Tello
        self._is_flying = False
        self._tello_lock = threading.RLock()
        self._tello = Tello(enable_exceptions=False,
                            state_callback_fn=self._state_callback_fn,
                            is_send_control_command_without_return=True)
        self._tello.connect()
        self._tello.streamon()

        ### background threads
        cmd_thread = threading.Thread(target=self._cmd_thread)
        cmd_thread.daemon = True
        cmd_thread.start()

        video_thread = threading.Thread(target=self._video_thread)
        video_thread.daemon = True
        video_thread.start()

    ###################
    ### ROS threads ###
    ###################

    def _takeoff_callback(self, msg):
        with self._tello_lock:
            self._tello.takeoff()
        self._is_flying = True

    def _land_callback(self, msg):
        self._is_flying = False
        with self._tello_lock:
            self._tello.land()

    def _estop_callback(self, msg):
        with self._tello_lock:
            self._tello.emergency()

    def _cmd_callback(self, msg):
        self._cmd = msg
        self._cmd_stamp = rospy.Time.now()
        self._cmd_thread_step(self._cmd)

    ###############
    ### Threads ###
    ###############

    @property
    def _default_cmd(self):
        return CmdTello(vx=0., vy=0., vyaw=0., height=self._state_msg.height)

    def _cmd_thread(self):
        rate = rospy.Rate(0.25)
        while not rospy.is_shutdown():
            rate.sleep()

            if (rospy.Time.now() - self._cmd_stamp).to_sec() < self._min_cmd_dt:
                msg = self._cmd
            else:
                msg = self._default_cmd
            self._cmd_thread_step(msg)

    def _cmd_thread_step(self, msg):
        if not self._is_flying:
            return
        vx = msg.vx
        vy = msg.vy
        height = msg.height
        vyaw = msg.vyaw

        height_error = self._state_msg.height - height
        vz = np.clip(-1. * height_error, -0.2, 0.2)

        with self._tello_lock:
            self._tello.send_rc_control(forward_backward_velocity=int(100 * vx),
                                        left_right_velocity=int(100 * vy),
                                        up_down_velocity=int(100 * vz),
                                        yaw_velocity=int(100 * vyaw))

    def _video_thread(self):
        container = av.open(self._tello.get_udp_video_address())

        frame_skip = 300
        seq = 0
        while not rospy.is_shutdown():
            for frame in container.decode(video=0):
                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    continue
                start_time = time.time()

                image = np.array(frame.to_image())
                image_msg = ros_numpy.msgify(Image, image, encoding='rgb8')
                image_msg.header.stamp = rospy.Time.now()
                image_msg.header.seq = seq
                self._camera_pub.publish(image_msg)

                if frame.time_base < 1.0/60:
                    time_base = 1.0/60
                else:
                    time_base = frame.time_base
                frame_skip = int((time.time() - start_time)/time_base)

                seq += 1

    def _state_callback_fn(self, response):
        try:
            names_and_value_strs = ' '.join(response.replace(';', ' ').split()).split()
            d = dict()
            for name_and_value_str in names_and_value_strs:
                name, value_str = name_and_value_str.split(':')
                value = float(value_str)
                d[name] = value

            # convert to metric
            state = {
                'acceleration': 0.01 * np.array([d['agx'], d['agy'], d['agz']]),
                'velocity': 0.01 * np.array([d['vgx'], d['vgy'], d['vgz']]),
                'rpy': np.deg2rad(np.array([d['roll'], d['pitch'], d['yaw']])),
                'battery': d['bat'],
                'barometer': 0.01 * d['baro'],
                'height': 0.01 * d['tof'] if d['tof'] < 6550 else 0.,
            }

            state_msg = TelloState(
                acceleration=Vector3(*state['acceleration']),
                velocity=Vector3(*state['velocity']),
                rpy=RollPitchYaw(*state['rpy']),
                battery=state['battery'],
                barometer=state['barometer'],
                height=state['height']
            )
            state_msg.header.stamp = rospy.Time.now()
            state_msg.header.seq = self._state_seq
            self._state_msg = state_msg
            self._state_pub.publish(state_msg)

            self._state_seq += 1
        except Exception:
            pass

    ###########
    ### Run ###
    ###########

    def run(self):
        rospy.spin()


rospy.init_node('ROSTello', anonymous=True)
node = ROSTellopyNode()
try:
    node.run()
except KeyboardInterrupt:
    print('Shutting down ros_tellopy_node.py')
