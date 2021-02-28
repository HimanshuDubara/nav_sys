#! /usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import numpy as np
import math





class Obstacle:
    def __init__(self):
        #define variables to handle ir messages
        self.ir_1_dist = Range()
        self.ir_2_dist = Range()
        self.ir_3_dist = Range()
        self.ir_4_dist = Range()
        self.ir_5_dist = Range()
        self.ir_6_dist = Range()
        self.ir_7_dist = Range()
        self.ir_8_dist = Range()
        self.bot_pose = Odometry()#Bot's position to be read as a Subscriber topic
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
        if ir_1_dist.range <=0.15:#Considering only of distance is less than 0.15m
            front_pose[0] = (front_pose[0] + ir_1_dist.range*(math.cos(ir_1_angle)) + ir_1_pose[0] )/(1+ir_1_dist.range) #Weighted according to the distance
            front_pose[1] = (front_pose[1] + ir_1_dist.range*(math.sin(ir_1_angle)) + ir_1_pose[1])/(1+ir_1_dist.range) #Weighted according to the distance

        if ir_2_dist.range <=0.15:#Considering only of distance is less than 0.15m
                front_pose[0] = (front_pose[0] + ir_2_dist.range*(math.cos(ir_2_angle)) + ir_2_pose[0])/(1+ir_2_dist.range)  #Weighted according to the distance
                front_pose[1] = (front_pose[1] + ir_2_dist.range*(math.sin(ir_2_angle)) + ir_2_pose[1])/(1+ir_2_dist.range) #Weighted according to the distance

        if ir_3_dist.range <=0.15:#Considering only of distance is less than 0.15m
                    front_pose[0] = (front_pose[0] + ir_3_dist.range*(math.cos(ir_3_angle)) + ir_3_pose[0])/(1+ir_3_dist.range) #Weighted according to the distance
                    front_pose[1] = (front_pose[1] + ir_3_dist*(math.sin(ir_3_angle)) + ir_3_pose[1])/(1+ir_3_dist.range) #Weighted according to the distance

        if ir_4_dist.range <=0.15:#Considering only of distance is less than 0.15m
                    front_pose[0] = (front_pose[0] + ir_4_dist.range*(math.cos(ir_4_angle)) + ir_4_pose[0])/(1+ir_4_dist.range) #Weighted according to the distance
                    front_pose[1] = (front_pose[1] + ir_4_dist.range*(math.sin(ir_4_angle)) + ir_4_pose[1])/(1+ir_4_dist.range)  #Weighted according to the distance

        if ir_5_dist.range <=0.15:#Considering only of distance is less than 0.15m
                    front_pose[0] = (front_pose[0] + ir_5_dist.range*(math.cos(ir_5_angle)) + ir_5_pose[0])/(1+ir_5_dist.range) #Weighted according to the distance
                    front_pose[1] = (front_pose[1] + ir_5_dist.range*(math.sin(ir_5_angle)) + ir_5_pose[1])/(1+ir_5_dist.range) #Weighted according to the distance

        if ir_6_dist.range <=0.15:#Considering only of distance is less than 0.15m
                    front_pose[0] = (front_pose[0] + ir_6_dist.range*(math.cos(ir_6_angle)) + ir_6_pose[0])/(1+ir_6_dist.range) #Weighted according to the distance
                    front_pose[1] = (front_pose[1] + ir_6_dist.range*(math.sin(ir_6_angle)) + ir_6_pose[1])/(1+ir_6_dist.range) #Weighted according to the distance

        #Adding bot position
        front_pose[0] = front_pose[0] + bot_pose.pose.pose.position.x
        front_pose[1] = front_pose[1] + bot_pose.pose.pose.position.y

    def back_object_position(self):
        if ir_7_dist.range <=0.15:#Considering only of distance is less than 0.15m
            back_pose[0] = (front_pose[0] + ir_7_dist.range*(math.cos(ir_7_angle)) + ir_7_pose[0])/(1+ir_7_dist.range)  #Weighted according to the distance
            back_pose[1] = (front_pose[1] + ir_7_dist*(math.sin(ir_7_angle)) + ir_7_pose[1])/(1+ir_7_dist.range) #Weighted according to the distance

        if ir_8_dist.range <=0.15:#Considering only of distance is less than 0.15m
                back_pose[0] = (front_pose[0] + ir_8_dist.range*(math.cos(ir_8_angle)) + ir_8_pose[0])/(1+ir_8_dist.range) #Weighted according to the distance
                back_pose[1] = (front_pose[1] + ir_8_dist.range*(math.sin(ir_8_angle)) + ir_8_pose[1])/(1+ir_8_dist.range)  #Weighted according to the distance

        #Adding bot position
        back_pose[0] = back_pose[0] + bot_pose.pose.pose.position.x
        back_pose[1] = back_pose[1] + bot_pose.pose.pose.position.y

    def publish_values(self):
        abs_front = Odometry()
        abs_back = Odometry()

        abs_front.pose.pose.position.x = front_pose[0]
        abs_front.pose.pose.position.y = front_pose[1]
        abs_front.pose.pose.position.z = 0.096
        abs_front.header.stamp = rospy.Time.now()
        abs_front.header.frame_id = "map"
        pub1.publish(abs_front)


        abs_back.pose.pose.position.x = back_pose[0]
        abs_back.pose.pose.position.y = back_pose[1]
        abs_back.pose.pose.postion.z = 0.096
        abs_back.header.stamp = rospy.Time.now()
        abs_back.header.frame_id = "map"
        pub2.publish(abs_back)

if __name__ == '__main__':
    rospy.init_node('obstacle',anonymous=True)

    obs = Obstacle()

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


    rospy.spin()
