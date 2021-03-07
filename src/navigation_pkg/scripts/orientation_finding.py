#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64



def listener():
    im = Imu()
    print(im.orientation.z)

if __name__ == '__main__':
    listener()
