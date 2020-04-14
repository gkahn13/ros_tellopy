#!/usr/bin/env python

import av
import numpy as np
import threading
import time
import signal
import sys

from geometry_msgs.msg import Vector3
import rospy
import ros_numpy
from sensor_msgs.msg import Image, CompressedImage

from ros_tellopy import Tello

from crazyflie.msg import CFMotion, CFCommand, CFData

class ROSTellopyNode(object):

    def __init__(self, dt=0.05, config="None"):
        self._dt = dt # rate at which states are published

        self._ros_prefix = "/cf/0/"
        self._ext_config = config

        self._alt = 0.6
        self._height_lims = [0.15, 1.2]

        ### ROS publishers
        self._camera_pub = rospy.Publisher(self._ros_prefix + 'image', Image, queue_size=1)
        # self._camera_compressed_pub = rospy.Publisher(self._ros_prefix + 'image', CompressedImage, queue_size=1)
        self._state_pub = rospy.Publisher(self._ros_prefix + 'data', CFData, queue_size=10)

        ### ROS subscribers
        rospy.Subscriber(self._ros_prefix + 'command', CFCommand, self._cmd_callback)
        rospy.Subscriber(self._ros_prefix + 'extra_command', CFCommand, self._extra_cmd_callback)
        rospy.Subscriber(self._ros_prefix + 'motion', CFMotion, self._motion_callback)

        ### cb state
        self._state_msg = None
        self._state_seq = 0
        self._motion = self._default_motion

        ### Tello
        self._seq_running = False
        self._is_flying = False
        self._tello_lock = threading.RLock()
        self._tello = Tello(enable_exceptions=False,
                            state_callback_fn=self._state_callback_fn,
                            is_send_control_command_without_return=True)
        self._tello.connect()
        self._tello.streamon()
        print("[Tello] Starting.")

        ### background threads
        motion_thread = threading.Thread(target=self._motion_thread)
        motion_thread.daemon = True
        motion_thread.start()

        video_thread = threading.Thread(target=self._video_thread)
        video_thread.daemon = True
        video_thread.start()

    ###################
    ### ROS threads ###
    ###################

    def _takeoff_callback(self):
        with self._tello_lock:
            self._tello.takeoff()
        self._is_flying = True

    def _land_callback(self):
        self._is_flying = False
        with self._tello_lock:
            self._tello.land()

    def _estop_callback(self):
        with self._tello_lock:
            self._tello.emergency()

    def _cmd_callback(self, msg):
        if msg.cmd == CFCommand.ESTOP:
            print("[Tello] ESTOP")
            self._estop_callback()
        elif msg.cmd == CFCommand.TAKEOFF:
            print("[Tello] TAKEOFF")
            self._takeoff_callback()
        elif msg.cmd == CFCommand.LAND:
            print("[Tello] LAND")
            self._land_callback()
        else:
            print("[Tello] Unrecognized command: %d" % msg.cmd)

    def _extra_cmd_callback(self, msg):
        # perform seq manuever:
        if self._seq_running:
            print("Skipping extra cmd")
        self._seq_running = True

        rate = rospy.Rate(1/self._dt)

        i = 0
        while i < 35 and not rospy.is_shutdown():
            motion = CFMotion()
            motion.is_flow_motion = True
            motion.x = 0.25
            motion.y = 0
            motion.dz = 0 if i < 30 else np.exp(np.sqrt(i) / 10.) - 1

            self._motion_thread_step(motion)
            i += 1

            rate.sleep()

        motion = self._default_motion
        self._motion_thread_step(motion)

        self._seq_running = False



    def _motion_callback(self, msg):
        self._motion = msg

        if not self._seq_running:
            self._motion_thread_step(self._motion)

    ###############
    ### Threads ###
    ###############

    @property
    def _default_motion(self):
        mot = CFMotion(x=0., y=0., yaw=0., dz=0)
        mot.stamp.stamp = rospy.Time.now()
        mot.is_flow_motion = True
        return mot

    # this publishes once every dt
    def _motion_thread(self):
        rate = rospy.Rate(1/self._dt)
        while not rospy.is_shutdown():

            # if self._motion is not None:
            #     self._motion_thread_step(self._motion)

            # state message publisher
            if self._state_msg is not None:
                self._state_msg.stamp.seq = self._state_seq
                self._state_seq += 1
                self._state_pub.publish(self._state_msg)

            rate.sleep()

    def _motion_thread_step(self, msg):
        if not self._is_flying:
            return

        assert msg.is_flow_motion

        vx = msg.x
        vy = msg.y
        vyaw = msg.yaw

        vz = msg.dz

        # original_alt = self._alt
        # self._alt = np.clip(self._alt + msg.dz, *self._height_lims)

        # if self._state_msg is not None:
        #     true_alt = self._state_msg.alt

        #     # bounding
        #     if true_alt < self._height_lims[0] + 2e-2 and vz < 0:
        #         vz = 0.
        #     if true_alt > self._height_lims[1] - 2e-2 and vz > 0:
        #         vz = 0.
        # else:
        #     vz = 0

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
            response = response.decode("utf-8")
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

            state_msg = CFData(ext_config=self._ext_config, ID=0, 
                            v_batt=state['battery'],
                            alt=state['height'],
                            roll=state['rpy'][0],
                            pitch=state['rpy'][1],
                            yaw=state['rpy'][2],
                            accel_x=state['acceleration'][0],
                            accel_y=state['acceleration'][1],
                            accel_z=state['acceleration'][2],
                            kalman_vx=state['velocity'][0],
                            kalman_vy=state['velocity'][1],
                            # kalman_vz=state['velocity'][2],
            )

            state_msg.stamp.stamp = rospy.Time.now() # keeping the accurate time
            self._state_msg = state_msg

        except Exception as e:
            print("[Tello] FAILURE: %s" % e)

    ###########
    ### Run ###
    ###########

    def run(self):
        rospy.spin()

def signal_handler(sig, node):
    print('You pressed Ctrl+C!')
    node._estop_callback()
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('ros_tellopy', anonymous=True)
    # import ipdb; ipdb.set_trace()

    configParam = rospy.search_param("config")
    dtParam = rospy.search_param("dt")
    config = rospy.get_param(configParam, 'None')
    dt = float(rospy.get_param(dtParam, '0.05'))
    print("[Tello] Config: %s, dt: %f" % (config, dt))

    node = ROSTellopyNode(dt=dt, config=config)

    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, node))

    try:
        node.run()
    except KeyboardInterrupt:
        print('[Tello] Shutting down ros_tellopy_node.py')
