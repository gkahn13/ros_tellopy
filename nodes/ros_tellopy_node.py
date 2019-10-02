#!/usr/bin/env python

import rospy

from ros_tellopy.ros_tellopy import ROSTellopy


rospy.init_node('ROSTello', anonymous=True)

node = ROSTellopy()

try:
    node.run()
except KeyboardInterrupt:
    print('Shutting down ros_tellopy_node.py')
