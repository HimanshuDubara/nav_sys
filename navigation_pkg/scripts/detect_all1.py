#!/usr/bin/env python
import roslib

import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from geometry_msgs.msg import PointStamped
import tf
from sensor_msgs.msg import Range
import numpy as np

ir_1_dist = Range()


def ir1_callback(msg):
    ir_1_dist = msg


if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener',anonymous = True)

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans1,rot1) = listener.lookupTransform('/odom', '/ir_1', rospy.Time(0))
            rospy.Subscriber('/sensor/ir_1', Range , ir1_callback)
            pub1 = rospy.Publisher('/abs_ir1', PointStamped, queue_size=10)
            rel_ir1 = np.zeros((2,1))
            rel_ir1[0] = ir_1_dist.range
            print(rel_ir1[0])
            eu = tf.transformations.euler_from_quaternion(rot1)
            theta = eu[2]
            #theta = self.im.pose.pose.orientation.z
            print(theta)
            # theta = -(self.im.orientation.z) #Clockwise
            x = trans1[0]
            y = trans[1]
            transform_matrix = np.zeros((3,3))
            temp = np.zeros((3,1))
            transform_matrix[0][0] = math.cos(theta)
            transform_matrix[0][1] = -math.sin(theta)
            transform_matrix[0][2] = x
            transform_matrix[1][0] = math.sin(theta)
            transform_matrix[1][1] = math.cos(theta)
            transform_matrix[1][2] = y
            transform_matrix[2][0] = 0
            transform_matrix[2][1] = 0
            transform_matrix[2][2] = 1

            temp[0] = rel_ir1[0]
            temp[1] = rel_ir1[1]
            temp[2] = 1
            temp = np.dot(transform_matrix,temp)
            temp = temp / temp[2]


            ir1_sensor = PointStamped()
            ir1_sensor.point.x = float(temp[0])
            ir1_sensor.point.y = float(temp[1])
            ir1_sensor.point.z = 0.096
            ir1_sensor.header.stamp = rospy.Time.now()
            ir1_sensor.header.frame_id = "map"
            pub1.publish(ir1_sensor)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
