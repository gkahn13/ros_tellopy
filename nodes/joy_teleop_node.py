#!/usr/bin/env python
import os
import numpy as np
import rospy
import rosbag
from datetime import datetime
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged, Ardrone3PilotingStateSpeedChanged, Ardrone3PilotingStateAttitudeChanged, Ardrone3PilotingStateAirSpeedChanged, Ardrone3CameraStateOrientationV2

dt = 0.2
secs_per_rosbag_save = 20
directory = "/home/katie/Desktop/rosbags/"

drone_type_param = rospy.get_param("drone_type")
use_joy = rospy.get_param("use_joy", True)
collect_data = rospy.get_param("collect_data", False)


print("drone type: "+ drone_type_param)
print("use joy: "+ str(use_joy))
print("collect data: "+ str(collect_data))


if drone_type_param == "ardrone":
	drone_type = "/ardrone/"
elif drone_type_param == "bebop":
	drone_type = "/bebop/"
else:
	raise Exception("drone type must be ardrone or beebop")


class JoyTeleopNode(object):
	LEFT_AXIS_X = 0
	LEFT_AXIS_Y = 1
	RIGHT_AXIS_X = 2
	RIGHT_AXIS_Y = 3
	X_BUTTON = 0
	L1_BUTTON = 4
	R1_BUTTON = 5

	def __init__(self):
		self._takeoff_pub = rospy.Publisher(drone_type+'takeoff', Empty, queue_size=1)
		self._land_pub = rospy.Publisher(drone_type+'land', Empty, queue_size=1)
		self._estop_pub = rospy.Publisher(drone_type+'reset', Empty, queue_size=1)
		self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		self._cmd = Twist()
		self._cmd.linear.x = 0
		self._cmd.linear.y = 0
		self._cmd.linear.z = 0
		self._cmd.angular.z = 0
		rospy.Subscriber('/joy', Joy, self._joy_callback)
		self._joy_axes = [0,0,0,0]
		self._next_update_time = rospy.get_time()
		self._yaw_noise = StepNoise(**{'reset_step_range' : [2,40], 'step_range' : [-0.2, 0.2], 'range' : [-0.3, 0.3]})
		self._x_noise = StepNoise(**{'reset_step_range' : [20,40], 'step_range' : [-0.05, 0.05], 'range' : [0.1, 0.5]})
		
		self._ros_msgs = dict()
		self._ros_msg_times = dict()
		if collect_data:
			datetime.now()
			self.bag = rosbag.Bag(directory+drone_type_param+"_"+datetime.now().strftime("%m_%d_%H_%M_%S")+".bag", 'w')
			if drone_type_param == "ardrone":
				self._ros_topics_and_types = dict([('/ardrone/odometry', Odometry)])
			else:
				self._ros_topics_and_types = dict([('odom', Odometry),
				                                   ('states/ardrone3/PilotingState/FlyingStateChanged', Ardrone3PilotingStateFlyingStateChanged),
				                                   ('states/ardrone3/PilotingState/SpeedChanged', Ardrone3PilotingStateSpeedChanged),
				                                   ('states/ardrone3/PilotingState/AttitudeChanged', Ardrone3PilotingStateAttitudeChanged),
				                                   ('states/ardrone3/PilotingState/AirSpeedChanged', Ardrone3PilotingStateAirSpeedChanged),
				                                   ('states/ardrone3/CameraState/OrientationV2',     Ardrone3CameraStateOrientationV2),])
				self._ros_msgs['states/ardrone3/PilotingState/FlyingStateChanged'] = Ardrone3PilotingStateFlyingStateChanged
				self._ros_msgs['states/ardrone3/PilotingState/SpeedChanged'] = Ardrone3PilotingStateSpeedChanged
				self._ros_msgs['states/ardrone3/PilotingState/AttitudeChanged'] = Ardrone3PilotingStateAttitudeChanged
				self._ros_msgs['states/ardrone3/PilotingState/AirSpeedChanged'] = Ardrone3PilotingStateAirSpeedChanged
				self._ros_msgs['states/ardrone3/CameraState/OrientationV2'] = Ardrone3CameraStateOrientationV2
			for topic, type in self._ros_topics_and_types.items():
				rospy.Subscriber(topic, type, self.ros_msg_update, (topic,))

		rospy.sleep(1.)

	def ros_msg_update(self, msg, args):
		topic = args[0]
		self._ros_msgs[topic] = msg			

	def _joy_callback(self, msg):
		if msg.buttons[JoyTeleopNode.X_BUTTON]:
			print("estop")
			self._estop_pub.publish(Empty())
		elif msg.buttons[JoyTeleopNode.L1_BUTTON]:
			print("land")
			self._land_pub.publish(Empty())
		elif msg.buttons[JoyTeleopNode.R1_BUTTON]:
			print("take off")
			self._takeoff_pub.publish(Empty())
		self._joy_axes = msg.axes

		

	def run(self):
		rate = rospy.Rate(20.)
		next_bag_save_time = rospy.get_time() + secs_per_rosbag_save
		while not rospy.is_shutdown():
			rate.sleep()
			
			if rospy.get_time()>self._next_update_time or (not collect_data):
				self._next_update_time += dt
				cmd = Twist()
				cmd.linear.y = self._joy_axes[JoyTeleopNode.RIGHT_AXIS_X]
				cmd.linear.z = 0.1 * self._joy_axes[JoyTeleopNode.LEFT_AXIS_Y]
				if collect_data:
					cmd.linear.x = min(max(self._joy_axes[JoyTeleopNode.RIGHT_AXIS_Y] + self._x_noise.step(), 0), 1)
					cmd.angular.z = min(max(2*self._joy_axes[JoyTeleopNode.LEFT_AXIS_X] + self._yaw_noise.step(), -1), 1)
				else:
					cmd.linear.x = self._joy_axes[JoyTeleopNode.RIGHT_AXIS_Y]
					cmd.angular.z = self._joy_axes[JoyTeleopNode.LEFT_AXIS_X]
				print(cmd)
				self._cmd = cmd
				
				if collect_data:
					self.bag.write('/cmd_vel', cmd)
					for topic in self._ros_topics_and_types.keys():
						self.bag.write(topic, self._ros_msgs[topic])

					if rospy.get_time() > next_bag_save_time:
						next_bag_save_time += secs_per_rosbag_save

						self.bag.close()
						print("saved bag")
						self.bag = rosbag.Bag(directory+drone_type_param+"_"+datetime.now().strftime("%m_%d_%H_%M_%S")+".bag", 'w')

			self._cmd_pub.publish(self._cmd)


class StepNoise():
	def __init__(self, **kwargs):
		self._reset_step_range = kwargs['reset_step_range']
		self._step_range = kwargs['step_range']
		self._range = kwargs['range']

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


rospy.init_node('JoyTeleopNode', anonymous=True)

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




