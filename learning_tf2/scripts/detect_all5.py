#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
import numpy as np
import math
from geometry_msgs.msg import PointStamped
import tf
from nav_msgs.msg import Odometry



class Obstacle:
    def __init__(self):
        self.ir_1_dist = Range()
        self.ir_2_dist = Range()
        self.ir_3_dist = Range()
        self.ir_4_dist = Range()
        self.ir_5_dist = Range()
        self.ir_6_dist = Range()
        self.ir_7_dist = Range()
        self.ir_8_dist = Range()
        self.bot_pose = PointStamped()#Bot's position to be read as a Subscriber topic
        self.listener = tf.TransformListener()
        self.ir_array = np.zeros((9,2))




    def ir1_callback(self,msg):
        #Get ir_1 message
        self.ir_1_dist = msg


    def ir2_callback(self,msg):
        #Get ir_2_joint message
        self.ir_2_dist = msg


    def ir3_callback(self,msg):
        #Get ir3 message
        self.ir_3_dist = msg


    def ir4_callback(self,msg):
        #Get ir_4 message
        self.ir_4_dist = msg


    def ir5_callback(self,msg):
        #Get ir_5 message
        self.ir_5_dist = msg


    def ir6_callback(self,msg):
        #Get ir_6 message
        self.ir_6_dist = msg


    def ir7_callback(self,msg):
        #Get ir_7 message
        self.ir_7_dist = msg


    def ir8_callback(self,msg):
        #Get ir_8 message
        self.ir_8_dist = msg


    def ir1_pose(self):
        while(True):
            try:
                [trans1,rot1] =self.listener.lookupTransform('/odom', '/ir_1', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        eu = tf.transformations.euler_from_quaternion(rot1)
        #theta = self.im.pose.pose.orientation.z

        # theta = -(self.im.orientation.z) #Clockwise
        x = trans1[0]
        y = trans1[1]
        transform_matrix = np.zeros((3,3))
        temp = np.zeros((3,1))
        transform_matrix[0][0] = math.cos(eu[2])
        transform_matrix[0][1] = -math.sin(eu[2])
        transform_matrix[0][2] = x
        transform_matrix[1][0] = math.sin(eu[2])
        transform_matrix[1][1] = math.cos(eu[2])
        transform_matrix[1][2] = y
        transform_matrix[2][0] = 0
        transform_matrix[2][1] = 0
        transform_matrix[2][2] = 1

        temp[0][0] = self.ir_1_dist.range
        temp[1][0] = 0
        temp[2][0] = 1
        temp = np.dot(transform_matrix,temp)
        temp = temp / temp[2][0]

        ir1_sensor = PointStamped()
        ir1_sensor.point.x = float(temp[0][0])
        ir1_sensor.point.y = float(temp[1][0])
        ir1_sensor.point.z = 0.096
        ir1_sensor.header.stamp = rospy.Time.now()
        ir1_sensor.header.frame_id = "map"
        pub1.publish(ir1_sensor)

    def ir2_pose(self):
        while(True):
            try:
                [trans1,rot1] =self.listener.lookupTransform('/odom', '/ir_2', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        eu = tf.transformations.euler_from_quaternion(rot1)
        #theta = self.im.pose.pose.orientation.z

        # theta = -(self.im.orientation.z) #Clockwise
        x = trans1[0]
        y = trans1[1]
        transform_matrix = np.zeros((3,3))
        temp = np.zeros((3,1))
        transform_matrix[0][0] = math.cos(eu[2])
        transform_matrix[0][1] = -math.sin(eu[2])
        transform_matrix[0][2] = x
        transform_matrix[1][0] = math.sin(eu[2])
        transform_matrix[1][1] = math.cos(eu[2])
        transform_matrix[1][2] = y
        transform_matrix[2][0] = 0
        transform_matrix[2][1] = 0
        transform_matrix[2][2] = 1

        temp[0][0] = self.ir_2_dist.range
        temp[1][0] = 0
        temp[2][0] = 1
        temp = np.dot(transform_matrix,temp)
        temp = temp / temp[2][0]

        ir1_sensor = PointStamped()
        ir1_sensor.point.x = float(temp[0][0])
        ir1_sensor.point.y = float(temp[1][0])
        ir1_sensor.point.z = 0.096
        ir1_sensor.header.stamp = rospy.Time.now()
        ir1_sensor.header.frame_id = "map"
        pub2.publish(ir1_sensor)

    def ir3_pose(self):
        while(True):
            try:
                [trans1,rot1] =self.listener.lookupTransform('/odom', '/ir_3', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        eu = tf.transformations.euler_from_quaternion(rot1)
        #theta = self.im.pose.pose.orientation.z

        # theta = -(self.im.orientation.z) #Clockwise
        x = trans1[0]
        y = trans1[1]
        transform_matrix = np.zeros((3,3))
        temp = np.zeros((3,1))
        transform_matrix[0][0] = math.cos(eu[2])
        transform_matrix[0][1] = -math.sin(eu[2])
        transform_matrix[0][2] = x
        transform_matrix[1][0] = math.sin(eu[2])
        transform_matrix[1][1] = math.cos(eu[2])
        transform_matrix[1][2] = y
        transform_matrix[2][0] = 0
        transform_matrix[2][1] = 0
        transform_matrix[2][2] = 1

        temp[0][0] = self.ir_3_dist.range
        temp[1][0] = 0
        temp[2][0] = 1
        temp = np.dot(transform_matrix,temp)
        temp = temp / temp[2][0]

        ir1_sensor = PointStamped()
        ir1_sensor.point.x = float(temp[0][0])
        ir1_sensor.point.y = float(temp[1][0])
        ir1_sensor.point.z = 0.096
        ir1_sensor.header.stamp = rospy.Time.now()
        ir1_sensor.header.frame_id = "map"
        pub3.publish(ir1_sensor)

    def ir4_pose(self):
        while(True):
            try:
                [trans1,rot1] =self.listener.lookupTransform('/odom', '/ir_4', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        eu = tf.transformations.euler_from_quaternion(rot1)
        #theta = self.im.pose.pose.orientation.z

        # theta = -(self.im.orientation.z) #Clockwise
        x = trans1[0]
        y = trans1[1]
        transform_matrix = np.zeros((3,3))
        temp = np.zeros((3,1))
        transform_matrix[0][0] = math.cos(eu[2])
        transform_matrix[0][1] = -math.sin(eu[2])
        transform_matrix[0][2] = x
        transform_matrix[1][0] = math.sin(eu[2])
        transform_matrix[1][1] = math.cos(eu[2])
        transform_matrix[1][2] = y
        transform_matrix[2][0] = 0
        transform_matrix[2][1] = 0
        transform_matrix[2][2] = 1

        temp[0][0] = self.ir_4_dist.range
        temp[1][0] = 0
        temp[2][0] = 1
        temp = np.dot(transform_matrix,temp)
        temp = temp / temp[2][0]

        ir1_sensor = PointStamped()
        ir1_sensor.point.x = float(temp[0][0])
        ir1_sensor.point.y = float(temp[1][0])
        ir1_sensor.point.z = 0.096
        ir1_sensor.header.stamp = rospy.Time.now()
        ir1_sensor.header.frame_id = "map"
        pub4.publish(ir1_sensor)

    def ir5_pose(self):
        while(True):
            try:
                [trans1,rot1] =self.listener.lookupTransform('/odom', '/ir_5', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        eu = tf.transformations.euler_from_quaternion(rot1)
        #theta = self.im.pose.pose.orientation.z

        # theta = -(self.im.orientation.z) #Clockwise
        x = trans1[0]
        y = trans1[1]
        transform_matrix = np.zeros((3,3))
        temp = np.zeros((3,1))
        transform_matrix[0][0] = math.cos(eu[2])
        transform_matrix[0][1] = -math.sin(eu[2])
        transform_matrix[0][2] = x
        transform_matrix[1][0] = math.sin(eu[2])
        transform_matrix[1][1] = math.cos(eu[2])
        transform_matrix[1][2] = y
        transform_matrix[2][0] = 0
        transform_matrix[2][1] = 0
        transform_matrix[2][2] = 1

        temp[0][0] = self.ir_5_dist.range
        temp[1][0] = 0
        temp[2][0] = 1
        temp = np.dot(transform_matrix,temp)
        temp = temp / temp[2][0]

        ir1_sensor = PointStamped()
        ir1_sensor.point.x = float(temp[0][0])
        ir1_sensor.point.y = float(temp[1][0])
        ir1_sensor.point.z = 0.096
        ir1_sensor.header.stamp = rospy.Time.now()
        ir1_sensor.header.frame_id = "map"
        pub5.publish(ir1_sensor)

    def ir6_pose(self):
        while(True):
            try:
                [trans1,rot1] =self.listener.lookupTransform('/odom', '/ir_6', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        eu = tf.transformations.euler_from_quaternion(rot1)
        #theta = self.im.pose.pose.orientation.z

        # theta = -(self.im.orientation.z) #Clockwise
        x = trans1[0]
        y = trans1[1]
        transform_matrix = np.zeros((3,3))
        temp = np.zeros((3,1))
        transform_matrix[0][0] = math.cos(eu[2])
        transform_matrix[0][1] = -math.sin(eu[2])
        transform_matrix[0][2] = x
        transform_matrix[1][0] = math.sin(eu[2])
        transform_matrix[1][1] = math.cos(eu[2])
        transform_matrix[1][2] = y
        transform_matrix[2][0] = 0
        transform_matrix[2][1] = 0
        transform_matrix[2][2] = 1

        temp[0][0] = self.ir_6_dist.range
        temp[1][0] = 0
        temp[2][0] = 1
        temp = np.dot(transform_matrix,temp)
        temp = temp / temp[2][0]

        ir1_sensor = PointStamped()
        ir1_sensor.point.x = float(temp[0][0])
        ir1_sensor.point.y = float(temp[1][0])
        ir1_sensor.point.z = 0.096
        ir1_sensor.header.stamp = rospy.Time.now()
        ir1_sensor.header.frame_id = "map"
        pub6.publish(ir1_sensor)

    def ir7_pose(self):
        while(True):
            try:
                [trans1,rot1] =self.listener.lookupTransform('/odom', '/ir_7', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        eu = tf.transformations.euler_from_quaternion(rot1)
        #theta = self.im.pose.pose.orientation.z

        # theta = -(self.im.orientation.z) #Clockwise
        x = trans1[0]
        y = trans1[1]
        transform_matrix = np.zeros((3,3))
        temp = np.zeros((3,1))
        transform_matrix[0][0] = math.cos(eu[2])
        transform_matrix[0][1] = -math.sin(eu[2])
        transform_matrix[0][2] = x
        transform_matrix[1][0] = math.sin(eu[2])
        transform_matrix[1][1] = math.cos(eu[2])
        transform_matrix[1][2] = y
        transform_matrix[2][0] = 0
        transform_matrix[2][1] = 0
        transform_matrix[2][2] = 1

        temp[0][0] = self.ir_7_dist.range
        temp[1][0] = 0
        temp[2][0] = 1
        temp = np.dot(transform_matrix,temp)
        temp = temp / temp[2][0]

        ir1_sensor = PointStamped()
        ir1_sensor.point.x = float(temp[0][0])
        ir1_sensor.point.y = float(temp[1][0])
        ir1_sensor.point.z = 0.096
        ir1_sensor.header.stamp = rospy.Time.now()
        ir1_sensor.header.frame_id = "map"
        pub7.publish(ir1_sensor)

    def ir8_pose(self):
        while(True):
            try:
                [trans1,rot1] =self.listener.lookupTransform('/odom', '/ir_8', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        eu = tf.transformations.euler_from_quaternion(rot1)
        #theta = self.im.pose.pose.orientation.z

        # theta = -(self.im.orientation.z) #Clockwise
        x = trans1[0]
        y = trans1[1]
        transform_matrix = np.zeros((3,3))
        temp = np.zeros((3,1))
        transform_matrix[0][0] = math.cos(eu[2])
        transform_matrix[0][1] = -math.sin(eu[2])
        transform_matrix[0][2] = x
        transform_matrix[1][0] = math.sin(eu[2])
        transform_matrix[1][1] = math.cos(eu[2])
        transform_matrix[1][2] = y
        transform_matrix[2][0] = 0
        transform_matrix[2][1] = 0
        transform_matrix[2][2] = 1

        temp[0][0] = self.ir_8_dist.range
        temp[1][0] = 0
        temp[2][0] = 1
        temp = np.dot(transform_matrix,temp)
        temp = temp / temp[2][0]

        ir1_sensor = PointStamped()
        ir1_sensor.point.x = float(temp[0][0])
        ir1_sensor.point.y = float(temp[1][0])
        ir1_sensor.point.z = 0.096
        ir1_sensor.header.stamp = rospy.Time.now()
        ir1_sensor.header.frame_id = "map"
        pub8.publish(ir1_sensor)

if __name__ == '__main__':


    rospy.init_node('obstacle5',anonymous=True)

    obs = Obstacle()

    while not rospy.is_shutdown():
        #rospy.Subscriber('/sensor/ir_1', Range , obs.ir1_callback)
        #rospy.Subscriber('/sensor/ir_2', Range , obs.ir2_callback)
        #rospy.Subscriber('/sensor/ir_3', Range , obs.ir3_callback)
        #rospy.Subscriber('/sensor/ir_4', Range , obs.ir4_callback)
        rospy.Subscriber('/sensor/ir_5', Range , obs.ir5_callback)
        #rospy.Subscriber('/sensor/ir_6', Range , obs.ir6_callback)
        #rospy.Subscriber('/sensor/ir_7', Range , obs.ir7_callback)
        #rospy.Subscriber('/sensor/ir_8', Range , obs.ir8_callback)

<<<<<<< HEAD
        #pub1 = rospy.Publisher('/abs_ir1', PointStamped, queue_size=10)
        #pub2 = rospy.Publisher('/abs_ir2', PointStamped, queue_size=10)
        #pub3 = rospy.Publisher('/abs_ir3', PointStamped, queue_size=10)
        #pub4 = rospy.Publisher('/abs_ir4', PointStamped, queue_size=10)
        pub5 = rospy.Publisher('/abs_ir5', PointStamped, queue_size=1)
        #pub6 = rospy.Publisher('/abs_ir6', PointStamped, queue_size=10)
        #pub7 = rospy.Publisher('/abs_ir7', PointStamped, queue_size=10)
        #pub8 = rospy.Publisher('/abs_ir8', PointStamped, queue_size=10)
=======
        #pub1 = rospy.Publisher('/abs_ir1', PointStamped, queue_size=1)
        #pub2 = rospy.Publisher('/abs_ir2', PointStamped, queue_size=1)
        #pub3 = rospy.Publisher('/abs_ir3', PointStamped, queue_size=1)
        #pub4 = rospy.Publisher('/abs_ir4', PointStamped, queue_size=1)
        pub5 = rospy.Publisher('/abs_ir5', PointStamped, queue_size=1)
        #pub6 = rospy.Publisher('/abs_ir6', PointStamped, queue_size=1)
        #pub7 = rospy.Publisher('/abs_ir7', PointStamped, queue_size=1)
        #pub8 = rospy.Publisher('/abs_ir8', PointStamped, queue_size=1)
>>>>>>> 73e2f9c597c2939bcfa572f146beb4475dd996c6

        #obs.ir1_pose()
        #obs.ir2_pose()
        #obs.ir3_pose()
        #obs.ir4_pose()
        obs.ir5_pose()
        #obs.ir6_pose()
        #obs.ir7_pose()
        #obs.ir8_pose()
