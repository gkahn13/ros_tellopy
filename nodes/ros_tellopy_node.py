#!/usr/bin/env python

import av
import numpy as np
import threading
import sys
import time

import rospy
import ros_numpy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Empty, Float32

from ros_tellopy.msg import CmdTello, RollPitchYaw, TelloState
from ros_tellopy.tello import Tello

from cv_bridge import CvBridge
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')


class ROSTellopyNode(object):

    def __init__(self, min_cmd_freq=2.):
        self._min_cmd_dt = 1. / min_cmd_freq

        ### ROS publishers
        self._camera_pub = rospy.Publisher('camera', CompressedImage, queue_size=1)
        self.bridge = CvBridge()
        self._camera_raw_pub = rospy.Publisher('camera_raw', Image, queue_size=1)
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
        self._flight_data = None
        self._log_data = None
        self._tello_lock = threading.RLock()
        self._tello = Tello()
        self._tello.set_loglevel(Tello.LOG_ERROR)
        self._tello.subscribe(Tello.EVENT_FLIGHT_DATA, self._flight_data_callback_fn) # slow
        self._tello.subscribe(Tello.EVENT_LOG_DATA, self._log_data_callback_fn)  # fast
        self._tello.connect()

        ### background threads
        cmd_thread = threading.Thread(target=self._cmd_thread)
        cmd_thread.daemon = True
        cmd_thread.start()

        video_thread = threading.Thread(target=self._video_thread)
        video_thread.daemon = True
        video_thread.start()

    @property
    def _tello_height(self):
        if self._flight_data is None:
            return None
        return 0.1 * self._flight_data.height

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
        self._is_flying = False
        with self._tello_lock:
            self._tello.land() # TODO: no e-stop command

    def _cmd_callback(self, msg):
        self._cmd = msg
        self._cmd_stamp = rospy.Time.now()
        self._cmd_thread_step(self._cmd)

    ###############
    ### Threads ###
    ###############

    @property
    def _default_cmd(self):
        return CmdTello(vx=0., vy=0., vyaw=0., height=0.3)

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

        curr_height = self._tello_height
        if curr_height is None:
            vz = 0
        else:
            height_error = curr_height - height
            if abs(height_error) <= 0.2:
                vz = 0
            else:
                vz = np.clip(-0.1 * height_error, -0.2, 0.2)

        with self._tello_lock:
            self._tello.set_pitch(int(100 * vx))
            self._tello.set_roll(int(100 * vy))
            self._tello.set_throttle(int(100 * vz))
            self._tello.set_yaw(int(100 * vyaw))

    def _video_thread(self):
        container = av.open(self._tello.get_video_stream())

        frame_skip = 300
        seq = 0
        while not rospy.is_shutdown():
            for frame in container.decode(video=0):
                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    continue
                start_time = time.time()
                image_msg_time = rospy.Time.now()

                image = np.array(frame.to_image())
                image_msg = ros_numpy.msgify(Image, image, encoding='rgb8')
                image_msg.header.stamp = image_msg_time
                image_msg.header.seq = seq
                self._camera_raw_pub.publish(image_msg)

                image_msg = self.bridge.cv2_to_compressed_imgmsg(image)
                image_msg.header.stamp = image_msg_time
                image_msg.header.seq = seq
                self._camera_pub.publish(image_msg)

                if frame.time_base < 1.0/60:
                    time_base = 1.0/60
                else:
                    time_base = frame.time_base
                frame_skip = int((time.time() - start_time)/time_base)

                seq += 1

    def _flight_data_callback_fn(self, event, sender, data, **args):
        self._flight_data = data

    def _log_data_callback_fn(self, event, sender, data, **args):
        self._log_data = data

        if self._flight_data is None or self._log_data is None:
            return

        state_msg = TelloState()

        flight_data_msg = state_msg.flight_data
        for k, v in self._flight_data.__dict__.items():
            if hasattr(flight_data_msg, k):
                setattr(flight_data_msg, k, int(v))

        mvo_msg = state_msg.mvo
        for k, v in self._log_data.mvo.__dict__.items():
            if hasattr(mvo_msg, k):
                setattr(mvo_msg, k, float(v))

        imu_msg = state_msg.imu
        for k, v in self._log_data.imu.__dict__.items():
            if hasattr(imu_msg, k):
                setattr(imu_msg, k, float(v))

        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.seq = self._state_seq
        self._state_msg = state_msg
        self._state_pub.publish(state_msg)

        self._state_seq += 1

    ###########
    ### Run ###
    ###########

    def run(self):
        rate = rospy.Rate(2.)
        while not rospy.is_shutdown():
            rate.sleep()
        self._tello.quit()


rospy.init_node('ROSTello', anonymous=True)
node = ROSTellopyNode()
try:
    node.run()
except KeyboardInterrupt:
    print('Shutting down ros_tellopy_node.py')
