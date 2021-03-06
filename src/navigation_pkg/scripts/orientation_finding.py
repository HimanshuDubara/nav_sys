#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

global value
def callback(data):

    value.data = data.orientation.z


def listener():
    value = Float64()
    rospy.init_node('orientation_finding', anonymous=True)

    pub = rospy.Publisher('orient',Float64)
    rospy.Subscriber("imu", Imu, callback)
    pub.publish(value)
    rospy.spin()

if __name__ == '__main__':
    listener()
