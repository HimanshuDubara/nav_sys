#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
import numpy as np
import math
from geometry_msgs.msg import PointStamped
import tf


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
        self.im = Imu()
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

        self.ir_array = np.zeros((9,2))
        self.abs_range = np.zeros((9,1))

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

    def botpose(self,msg):
        #Get bot position
        self.bot_pose = msg

    def imu_read(self,msg):
        self.im = msg


#exp(dist) used as Weight
    def detect_ir1(self):
        abs_range = 0
        # if self.ir_1_dist.range <=0.15(self):#Considering only of distance is less than 0.15m
        self.ir_array[1][0] = (self.front_pose[0] + self.ir_1_dist.range*(math.cos(self.ir_1_angle)) + self.ir_1_pose[0] ) #Weighted according to the distance
        self.ir_array[1][1] = (self.front_pose[1] + self.ir_1_dist.range*(math.sin(self.ir_1_angle)) + self.ir_1_pose[1]) #Weighted according to the distance
        #self.abs_range[1][0] = self.abs_range[1][0] + self.ir_1_dist.range

    def detect_ir2(self):
        abs_range = 0
        # if self.ir_1_dist.range <=0.15(self):#Considering only of distance is less than 0.15m
        self.ir_array[2][0] = (self.front_pose[0] + self.ir_2_dist.range*(math.cos(self.ir_2_angle)) + self.ir_2_pose[0] ) #Weighted according to the distance
        self.ir_array[2][1] = (self.front_pose[1] + self.ir_2_dist.range*(math.sin(self.ir_2_angle)) + self.ir_2_pose[1]) #Weighted according to the distance
        #self.abs_range[2][0] = self.abs_range[2][0] + self.ir_2_dist.range

    def detect_ir3(self):
        abs_range = 0
        # if self.ir_1_dist.range <=0.15(self):#Considering only of distance is less than 0.15m
        self.ir_array[3][0] = (self.front_pose[0] + self.ir_3_dist.range*(math.cos(self.ir_3_angle)) + self.ir_3_pose[0] ) #Weighted according to the distance
        self.ir_array[3][1] = (self.front_pose[1] + self.ir_3_dist.range*(math.sin(self.ir_3_angle)) + self.ir_3_pose[1]) #Weighted according to the distance
        #self.abs_range[3][0] = self.abs_range[3][0] + self.ir_3_dist.range

    def detect_ir4(self):
        abs_range = 0
        # if self.ir_1_dist.range <=0.15(self):#Considering only of distance is less than 0.15m
        self.ir_array[4][0] = (self.front_pose[0] + self.ir_4_dist.range*(math.cos(self.ir_4_angle)) + self.ir_4_pose[0] ) #Weighted according to the distance
        self.ir_array[4][1] = (self.front_pose[1] + self.ir_4_dist.range*(math.sin(self.ir_4_angle)) + self.ir_4_pose[1]) #Weighted according to the distance
        #self.abs_range[4][0] = self.abs_range[4][0] + self.ir_4_dist.range

    def detect_ir5(self):
        abs_range = 0
        # if self.ir_1_dist.range <=0.15(self):#Considering only of distance is less than 0.15m
        self.ir_array[5][0] = (self.front_pose[0] + self.ir_5_dist.range*(math.cos(self.ir_5_angle)) + self.ir_5_pose[0] ) #Weighted according to the distance
        self.ir_array[5][1] = (self.front_pose[1] + self.ir_5_dist.range*(math.sin(self.ir_5_angle)) + self.ir_5_pose[1]) #Weighted according to the distance
        #self.abs_range[5][0] = self.abs_range[5][0] + self.ir_5_dist.range

    def detect_ir6(self):
        abs_range = 0
        # if self.ir_1_dist.range <=0.15(self):#Considering only of distance is less than 0.15m
        self.ir_array[6][0] = (self.front_pose[0] + self.ir_6_dist.range*(math.cos(self.ir_6_angle)) + self.ir_6_pose[0] ) #Weighted according to the distance
        self.ir_array[6][1] = (self.front_pose[1] + self.ir_6_dist.range*(math.sin(self.ir_6_angle)) + self.ir_6_pose[1]) #Weighted according to the distance
        #self.abs_range[6][0] = self.abs_range[6][0] + self.ir_6_dist.range

    def detect_ir7(self):
        abs_range = 0
        # if self.ir_1_dist.range <=0.15(self):#Considering only of distance is less than 0.15m
        self.ir_array[7][0] = (self.front_pose[0] + self.ir_7_dist.range*(math.cos(self.ir_7_angle)) + self.ir_7_pose[0] ) #Weighted according to the distance
        self.ir_array[7][1] = (self.front_pose[1] + self.ir_7_dist.range*(math.sin(self.ir_7_angle)) + self.ir_7_pose[1]) #Weighted according to the distance
        #self.abs_range[7][0] = self.abs_range[7][0] + self.ir_7_dist.range

    def detect_ir8(self):
        abs_range = 0
        # if self.ir_1_dist.range <=0.15:#Considering only of distance is less than 0.15m
        self.ir_array[8][0] = (self.front_pose[0] + self.ir_8_dist.range*(math.cos(self.ir_8_angle)) + self.ir_8_pose[0] ) #Weighted according to the distance
        self.ir_array[8][1] = (self.front_pose[1] + self.ir_8_dist.range*(math.sin(self.ir_8_angle)) + self.ir_8_pose[1]) #Weighted according to the distance
        #self.abs_range[8][0] = self.abs_range[8][0] + self.ir_8_dist.range

    def read_all_ir(self):
            self.detect_ir1()
            self.detect_ir2()
            self.detect_ir3()
            self.detect_ir4()
            self.detect_ir5()
            self.detect_ir6()
            self.detect_ir7()
            self.detect_ir8()


    def change_coordinate(self):
        theta = -(self.im.orientation.z) #Clockwise
        x = self.bot_pose.point.x
        y = self.bot_pose.point.y
        transform_matrix = np.zeros((3,3))
        temp = np.zeros((3,1))
        temp[2][0] = 1
        transform_matrix[0][0] = math.cos(theta)
        transform_matrix[0][1] = -math.sin(theta)
        transform_matrix[0][2] = x
        transform_matrix[1][0] = math.sin(theta)
        transform_matrix[1][1] = math.cos(theta)
        transform_matrix[1][2] = y
        transform_matrix[2][0] = 0
        transform_matrix[2][1] = 0
        transform_matrix[2][2] = 1
        for i in range(1,8):
            temp[0][0] = self.ir_array[i][0]
            temp[1][0] = self.ir_array[i][1]
            temp = np.dot(transform_matrix,temp)
            self.ir_array[i][0] = temp[0][0]
            self.ir_array[i][1] = temp[1][0]




    def publish_values(self):
        ir1_sensor = PointStamped()
        ir1_sensor.point.x = float(self.ir_array[1][0])
        ir1_sensor.point.y = float(self.ir_array[1][1])
        ir1_sensor.point.z = 0.096
        ir1_sensor.header.stamp = rospy.Time.now()
        ir1_sensor.header.frame_id = "map"
        pub1.publish(ir1_sensor)

        ir2_sensor = PointStamped()
        ir2_sensor.point.x = float(self.ir_array[2][0])
        ir2_sensor.point.y = float(self.ir_array[2][1])
        ir2_sensor.point.z = 0.096
        ir2_sensor.header.stamp = rospy.Time.now()
        ir2_sensor.header.frame_id = "map"
        pub2.publish(ir2_sensor)

        ir3_sensor = PointStamped()
        ir3_sensor.point.x = float(self.ir_array[3][0])
        ir3_sensor.point.y = float(self.ir_array[3][1])
        ir3_sensor.point.z = 0.096
        ir3_sensor.header.stamp = rospy.Time.now()
        ir3_sensor.header.frame_id = "map"
        pub3.publish(ir3_sensor)

        ir4_sensor = PointStamped()
        ir4_sensor.point.x = float(self.ir_array[4][0])
        ir4_sensor.point.y = float(self.ir_array[4][1])
        ir4_sensor.point.z = 0.096
        ir4_sensor.header.stamp = rospy.Time.now()
        ir4_sensor.header.frame_id = "map"
        pub4.publish(ir4_sensor)

        ir5_sensor = PointStamped()
        ir5_sensor.point.x = float(self.ir_array[5][0])
        ir5_sensor.point.y = float(self.ir_array[5][1])
        ir5_sensor.point.z = 0.096
        ir5_sensor.header.stamp = rospy.Time.now()
        ir5_sensor.header.frame_id = "map"
        pub5.publish(ir5_sensor)

        ir6_sensor = PointStamped()
        ir6_sensor.point.x = float(self.ir_array[6][0])
        ir6_sensor.point.y = float(self.ir_array[6][1])
        ir6_sensor.point.z = 0.096
        ir6_sensor.header.stamp = rospy.Time.now()
        ir6_sensor.header.frame_id = "map"
        pub6.publish(ir6_sensor)

        ir7_sensor = PointStamped()
        ir7_sensor.point.x = float(self.ir_array[7][0])
        ir7_sensor.point.y = float(self.ir_array[7][1])
        ir7_sensor.point.z = 0.096
        ir7_sensor.header.stamp = rospy.Time.now()
        ir7_sensor.header.frame_id = "map"
        pub7.publish(ir7_sensor)

        ir8_sensor = PointStamped()
        ir8_sensor.point.x = float(self.ir_array[8][0])
        ir8_sensor.point.y = float(self.ir_array[8][1])
        ir8_sensor.point.z = 0.096
        ir8_sensor.header.stamp = rospy.Time.now()
        ir8_sensor.header.frame_id = "map"
        pub8.publish(ir8_sensor)






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
        rospy.Subscriber('/localization_data_topic', PointStamped , obs.botpose)
        rospy.Subscriber('/imu',Imu,obs.imu_read)

        ################ publishing 8 IRs ################
        pub1 = rospy.Publisher('/abs_ir1', PointStamped, queue_size=10)
        pub2 = rospy.Publisher('/abs_ir2', PointStamped, queue_size=10)
        pub3 = rospy.Publisher('/abs_ir3', PointStamped, queue_size=10)
        pub4 = rospy.Publisher('/abs_ir4', PointStamped, queue_size=10)
        pub5 = rospy.Publisher('/abs_ir5', PointStamped, queue_size=10)
        pub6 = rospy.Publisher('/abs_ir6', PointStamped, queue_size=10)
        pub7 = rospy.Publisher('/abs_ir7', PointStamped, queue_size=10)
        pub8 = rospy.Publisher('/abs_ir8', PointStamped, queue_size=10)

        obs.read_all_ir()
        obs.change_coordinate()

        obs.publish_values()
