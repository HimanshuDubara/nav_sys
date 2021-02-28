#! /usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import numpy as np
import math





class Obstacle:
    def __init__(self):
        #define variables to handle ir messages
        ir_1_dist = Range()
        ir_2_dist = Range()
        ir_3_dist = Range()
        ir_4_dist = Range()
        ir_5_dist = Range()
        ir_6_dist = Range()
        ir_7_dist = Range()
        ir_8_dist = Range()
        bot_pose = Odometry()#Bot's position to be read as a Subscriber topic
        ir_1_angle = -0.2627
        ir_2_angle = 0.2627
        ir_3_angle = 0.5263
        ir_4_angle = -0.5263
        ir_5_angle =  1.5708
        ir_6_angle = -1.5708
        ir_7_angle = -2.879
        ir_8_angle = 2.879
        ir_1_pose = [0.09093, -0.02625]
        ir_2_pose = [0.09093, 0.02625]
        ir_3_pose = [0.0525, 0.09093]
        ir_4_pose = [0.0525, -0.09093]
        ir_5_pose = [0.0, 0.105]
        ir_6_pose = [0.0, -0.105]
        ir_7_pose = [-0.15, -0.02625]
        ir_8_pose = [-0.15, 0.02625]
        front_pose = [0,0]
        back_pose = [0,0]



    def __init__(self):
        self.ir_1_dist = Range()
        self.ir_2_dist = Range()
        self.ir_3_dist = Range()
        self.ir_4_dist = Range()
        self.ir_5_dist = Range()
        self.ir_6_dist = Range()
        self.ir_7_dist = Range()
        self.ir_8_dist = Range()
        self.bot_pose = Odometry()#Bot's position to be read as a Subscriber topic
        self.ir_1_angle = -0.2627
        self.ir_2_angle = 0.2627
        self.ir_3_angle = 0.5263
        self.ir_4_angle = -0.5263
        self.ir_5_angle =  1.5708
        self.ir_6_angle = -1.5708
        self.ir_7_angle = -2.879
        self.ir_8_angle = 2.879
        self.ir_1_pose = [0.09093, -0.02625]
        self.ir_2_pose = [0.09093, 0.02625]
        self.ir_3_pose = [0.0525, 0.09093]
        self.ir_4_pose = [0.0525, -0.09093]
        self.ir_5_pose = [0.0, 0.105]
        self.ir_6_pose = [0.0, -0.105]
        self.ir_7_pose = [-0.15, -0.02625]
        self.ir_8_pose = [-0.15, 0.02625]
        self.front_pose = [0,0]
        self.back_pose = [0,0]


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

    def bot_pose(self,msg):
        #Get bot position
        self.bot_pose = msg


#(1+dist) is used as weight to avoid blowing the pose out of proportion, as dist can be <1
    def front_object_position(self):
        if self.ir_1_dist.range <=0.15:#Considering only of distance is less than 0.15m
            self.front_pose[0] = (self.front_pose[0] + self.ir_1_dist.range*(math.cos(self.ir_1_angle)) + self.ir_1_pose[0] )/(1+self.ir_1_dist.range) #Weighted according to the distance
            self.front_pose[1] = (self.front_pose[1] + self.ir_1_dist.range*(math.sin(self.ir_1_angle)) + self.ir_1_pose[1])/(1+self.ir_1_dist.range) #Weighted according to the distance

        if self.ir_2_dist.range <=0.15:#Considering only of distance is less than 0.15m
                self.front_pose[0] = (self.front_pose[0] + self.ir_2_dist.range*(math.cos(self.ir_2_angle)) + self.ir_2_pose[0])/(1+self.ir_2_dist.range)  #Weighted according to the distance
                self.front_pose[1] = (self.front_pose[1] + self.ir_2_dist.range*(math.sin(self.ir_2_angle)) + self.ir_2_pose[1])/(1+self.ir_2_dist.range) #Weighted according to the distance

        if self.ir_3_dist.range <=0.15:#Considering only of distance is less than 0.15m
                    self.front_pose[0] = (self.front_pose[0] + self.ir_3_dist.range*(math.cos(self.ir_3_angle)) + self.ir_3_pose[0])/(1+self.ir_3_dist.range) #Weighted according to the distance
                    self.front_pose[1] = (self.front_pose[1] + self.ir_3_dist.range*(math.sin(self.ir_3_angle)) + self.ir_3_pose[1])/(1+self.ir_3_dist.range) #Weighted according to the distance

        if self.ir_4_dist.range <=0.15:#Considering only of distance is less than 0.15m
                    self.front_pose[0] = (self.front_pose[0] + self.ir_4_dist.range*(math.cos(self.ir_4_angle)) + self.ir_4_pose[0])/(1+self.ir_4_dist.range) #Weighted according to the distance
                    self.front_pose[1] = (self.front_pose[1] + self.ir_4_dist.range*(math.sin(self.ir_4_angle)) + self.ir_4_pose[1])/(1+self.ir_4_dist.range)  #Weighted according to the distance

        if self.ir_5_dist.range <=0.15:#Considering only of distance is less than 0.15m
                    self.front_pose[0] = (self.front_pose[0] + self.ir_5_dist.range*(math.cos(self.ir_5_angle)) + self.ir_5_pose[0])/(1+self.ir_5_dist.range) #Weighted according to the distance
                    self.front_pose[1] = (self.front_pose[1] + self.ir_5_dist.range*(math.sin(self.ir_5_angle)) + self.ir_5_pose[1])/(1+self.ir_5_dist.range) #Weighted according to the distance

        if self.ir_6_dist.range <=0.15:#Considering only of distance is less than 0.15m
                    self.front_pose[0] = (self.front_pose[0] + self.ir_6_dist.range*(math.cos(self.ir_6_angle)) + self.ir_6_pose[0])/(1+self.ir_6_dist.range) #Weighted according to the distance
                    self.front_pose[1] = (self.front_pose[1] + self.ir_6_dist.range*(math.sin(self.ir_6_angle)) + self.ir_6_pose[1])/(1+self.ir_6_dist.range) #Weighted according to the distance

        #Adding bot position
        self.front_pose[0] = self.front_pose[0] + self.bot_pose.pose.pose.position.x
        self.front_pose[1] = self.front_pose[1] + self.bot_pose.pose.pose.position.y

    def back_object_position(self):
        if self.ir_7_dist.range <=0.15:#Considering only of distance is less than 0.15m
            self.back_pose[0] = (self.back_pose[0] + self.ir_7_dist.range*(math.cos(self.ir_7_angle)) + self.ir_7_pose[0])/(1+self.ir_7_dist.range)  #Weighted according to the distance
            self.back_pose[1] = (self.back_pose[1] + self.ir_7_dist.range*(math.sin(self.ir_7_angle)) + self.ir_7_pose[1])/(1+self.ir_7_dist.range) #Weighted according to the distance

        if self.ir_8_dist.range <=0.15:#Considering only of distance is less than 0.15m
                self.back_pose[0] = (self.back_pose[0] + self.ir_8_dist.range*(math.cos(self.ir_8_angle)) + self.ir_8_pose[0])/(1+self.ir_8_dist.range) #Weighted according to the distance
                self.back_pose[1] = (self.back_pose[1] + self.ir_8_dist.range*(math.sin(self.ir_8_angle)) + self.ir_8_pose[1])/(1+self.ir_8_dist.range)  #Weighted according to the distance

        #Adding bot position
        self.back_pose[0] = self.back_pose[0] + self.bot_pose.pose.pose.position.x
        self.back_pose[1] = self.back_pose[1] + self.bot_pose.pose.pose.position.y

    def publish_values(self):
        abs_front = Odometry()
        abs_back = Odometry()

        abs_front.pose.pose.position.x = self.front_pose[0]
        abs_front.pose.pose.position.y = self.front_pose[1]
        abs_front.pose.pose.position.z = 0.096
        abs_front.header.stamp = rospy.Time.now()
        abs_front.header.frame_id = "map"
        pub1.publish(abs_front)
        print("Published abs_front")


        abs_back.pose.pose.position.x = self.back_pose[0]
        abs_back.pose.pose.position.y = self.back_pose[1]
        abs_back.pose.pose.position.z = 0.096
        abs_back.header.stamp = rospy.Time.now()
        abs_back.header.frame_id = "map"
        pub2.publish(abs_back)
        print("Published abs_back")

if __name__ == '__main__':
    rospy.init_node('obstacle',anonymous=True)

    obs = Obstacle()

    while not rospy.is_shutdown():
        rospy.Subscriber('/sensor/ir_1', Range , obs.ir1_callback)
        rospy.Subscriber('/sensor/ir_2', Range , obs.ir2_callback)
        rospy.Subscriber('/sensor/ir_3', Range , obs.ir3_callback)
        rospy.Subscriber('/sensor/ir_4', Range , obs.ir4_callback)
        rospy.Subscriber('/sensor/ir_5', Range , obs.ir5_callback)
        rospy.Subscriber('/sensor/ir_6', Range , obs.ir6_callback)
        rospy.Subscriber('/sensor/ir_7', Range , obs.ir7_callback)
        rospy.Subscriber('/sensor/ir_8', Range , obs.ir8_callback)
        rospy.Subscriber('/localization_data_topic', Odometry , obs.bot_pose)
        pub1 = rospy.Publisher('/abs_front_obs', Odometry, queue_size=1)
        pub2 = rospy.Publisher('/abs_back_obs',Odometry,queue_size = 1)
        obs.front_object_position()
        obs.back_object_position()
        obs.publish_values()


    
