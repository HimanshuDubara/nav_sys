#! /usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import PointStamped,Twist
import tf
from nav_msgs.msg import Odometry

class Work:
    def __init__(self):
        self.xbot = np.zeros((2,1))
        self.xg =  np.zeros((2,1))
        self.ugtg = np.zeros((2,1))
        self.u = np.zeros((2,1))
        self.xo1 = np.zeros((2,1))
        self.xo2 = np.zeros((2,1))
        self.xo3 = np.zeros((2,1))
        self.xo4 = np.zeros((2,1))
        self.xo5 = np.zeros((2,1))
        self.xo6 = np.zeros((2,1))
        self.xo7 = np.zeros((2,1))
        self.xo8 = np.zeros((2,1))
    def localization_callback(self,msg):
        self.xbot[0][0] = msg.point.x
        self.xbot[1][0] = msg.point.y

    def set_goal(self,a,b):
        self.xg[0][0] = a
        self.xg[1][0] = b

    def publish_goal(self):
        goal_point = PointStamped()
        goal_point.point.x = float(self.xg[0][0])
        goal_point.point.y = float(self.xg[1][0])
        goal_point.point.z = 0.096
        goal_point.header.stamp = rospy.Time.now()
        goal_point.header.frame_id = "map"
        goal_publisher.publish(goal_point)


    def get_goal_vel(self):

        self.ugtg = 0.04*(self.xg - self.xbot)
        self.u =self.ugtg
        print(self.u)

    def publish_vel(self):
        vel_msg = Twist()
        vel_msg.linear.x = 2*math.sqrt(math.pow(self.u[0][0],2)+math.pow(self.u[1][0],2))
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z =math.atan2(self.u[1][0],self.u[0][0])
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('gtg',anonymous = True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    goal_publisher = rospy.Publisher('/goal',PointStamped,queue_size = 1)
    w = Work()
    w.set_goal(-0.5,0.5)
    while not rospy.is_shutdown():
        rospy.Subscriber('/localization_data_topic',PointStamped,w.localization_callback)
        w.publish_goal()
        w.get_goal_vel()
        w.publish_vel()
