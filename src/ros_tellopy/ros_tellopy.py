import av
import numpy as np
import threading
import time

from geometry_msgs.msg import Twist, Vector3Stamped
import rospy
import ros_numpy
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Empty

from ros_tellopy.msg import TelloState
from tellopy import Tello


class ROSTellopy(object):

    def __init__(self, video_enabled=True, rate=20.):
        self._video_enabled = video_enabled
        self._rate = rate

        ### Tello

        self._tello_lock = threading.RLock()
        self._tello = Tello()
        self._tello.set_loglevel(Tello.LOG_ERROR)
        self._tello.connect()
        self._tello.wait_for_connection()

        import IPython; IPython.embed(); import sys; sys.exit(0)

        self._tello_flight_data_seq = 0
        self._tello.subscribe(self._tello.EVENT_FLIGHT_DATA, self._tello_flight_data_callback)
        self._tello_log_data_seq = 0
        self._tello.subscribe(self._tello.EVENT_LOG_DATA, self._tello_log_data_callback)

        ### ROS publishers

        if video_enabled:
            self._tello.start_video()

            self._most_recent_image = None
            self._most_recent_image_time = None
            threading.Thread(target=self._tello_video_thread).start()
            self._camera_pub = rospy.Publisher('camera', Image, queue_size=1)

        self._state_pub = rospy.Publisher('state', TelloState, queue_size=1)
        self._imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
        self._position_pub = rospy.Publisher('position', Vector3Stamped, queue_size=1)
        self._velocity_pub = rospy.Publisher('velocity', Vector3Stamped, queue_size=1)

        ### ROS subscribers

        rospy.Subscriber('takeoff', Empty, self._takeoff_callback)
        rospy.Subscriber('land', Empty, self._land_callback)
        rospy.Subscriber('estop', Empty, self._estop_callback)
        rospy.Subscriber('cmd_vel', Twist, self._cmd_vel_callback)

        ### run loop

    #####################
    ### Tello threads ###
    #####################

    def _tello_video_thread(self):
        container = av.open(self._tello.get_video_stream())
        # skip first 300 frames
        frame_skip = 300

        while not rospy.is_shutdown():
            # self._tello.set_video_mode(zoom=False)
            try:
                for frame in container.decode(video=0):
                    if 0 < frame_skip:
                        frame_skip = frame_skip - 1
                        continue
                    start_time = time.time()
                    self._most_recent_image_time = rospy.Time.now()
                    self._most_recent_image = np.array(frame.to_image())

                    if frame.time_base < 1.0 / 60:
                        time_base = 1.0 / 60
                    else:
                        time_base = frame.time_base
                    frame_skip = int((time.time() - start_time) / time_base)
            except:
                pass

    def _tello_flight_data_callback(self, event, sender, data, **args):
        msg = TelloState()
        attrnames = [attrname for attrname in dir(msg) if
                     not attrname.startswith('_') and
                     'serialize' not in attrname and
                     'header' not in attrname]
        for attrname in attrnames:
            value = getattr(data, attrname)
            setattr(msg, attrname, value)
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self._tello_flight_data_seq
        self._tello_flight_data_seq += 1

        self._state_pub.publish(msg)

    def _tello_log_data_callback(self, event, sender, data, **args):
        stamp = rospy.Time.now()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.seq = self._tello_log_data_seq
        imu_msg.orientation.w = data.imu.q0
        imu_msg.orientation.x = data.imu.q1
        imu_msg.orientation.y = data.imu.q2
        imu_msg.orientation.z = data.imu.q3
        imu_msg.angular_velocity.x = data.imu.gyro_x
        imu_msg.angular_velocity.y = data.imu.gyro_y
        imu_msg.angular_velocity.z = data.imu.gyro_z
        imu_msg.linear_acceleration.x = data.imu.acc_x
        imu_msg.linear_acceleration.y = data.imu.acc_y
        imu_msg.linear_acceleration.z = data.imu.acc_z

        pos_msg = Vector3Stamped()
        pos_msg.header.stamp = stamp
        pos_msg.header.seq = self._tello_log_data_seq
        pos_msg.vector.x = data.mvo.pos_x
        pos_msg.vector.y = data.mvo.pos_y
        pos_msg.vector.z = data.mvo.pos_z

        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = stamp
        vel_msg.header.seq = self._tello_log_data_seq
        vel_msg.vector.x = data.mvo.vel_x
        vel_msg.vector.y = data.mvo.vel_y
        vel_msg.vector.z = data.mvo.vel_z

        self._imu_pub.publish(imu_msg)
        self._position_pub.publish(pos_msg)
        self._velocity_pub.publish(vel_msg)
        self._tello_log_data_seq += 1

        # print('')
        # print(str(data))

    ###################
    ### ROS threads ###
    ###################

    def _takeoff_callback(self, msg):
        with self._tello_lock:
            self._tello.takeoff()

    def _land_callback(self, msg):
        with self._tello_lock:
            self._tello.land()

    def _estop_callback(self, msg):
        with self._tello_lock:
            self._tello.estop()

    def _cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.linear.z
        vyaw = msg.angular.z

        with self._tello_lock:
            self._tello.set_pitch(vx)
            self._tello.set_roll(vy)
            self._tello.set_throttle(vz)
            self._tello.set_yaw(vyaw)

    def run(self):
        rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            rate.sleep()

            if self._video_enabled:
                image, image_time = self._most_recent_image, self._most_recent_image_time
                if image is not None:
                    msg = ros_numpy.msgify(Image, image, encoding='rgb8')
                    msg.header.stamp = image_time
                    self._camera_pub.publish(msg)

