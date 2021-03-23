#! /usr/bin/env python

import rospy
import numpy as np
import math
import smach
import smach_ros
from geometry_msgs.msg import PointStamped,Twist


x = np.zeros((2,1))
xg = np.zeros((2,1))
xo = np.zeros((2,1))
xo1 = np.zeros((2,1))
xo2 = np.zeros((2,1))
xo3 = np.zeros((2,1))
xo4 = np.zeros((2,1))
xo5 = np.zeros((2,1))
xo6 = np.zeros((2,1))
xo7 = np.zeros((2,1))
xo8 = np.zeros((2,1))

###Defining goal at -0.5,0.5####
xg[0][0] = -0.5
xg[1][0] = 0.5
#############################

ugtg = np.zeros((2,1))
u = np.zeros((2,1))
uao1  =  np.zeros((2,1))
uao2 =  np.zeros((2,1))
uao3 = np.zeros((2,1))
uao4 =  np.zeros((2,1))
uao5 =  np.zeros((2,1))
uao6 =  np.zeros((2,1))
uao7 =  np.zeros((2,1))
uao8 =  np.zeros((2,1))

uao = np.zeros((2,1))
ucw = np.zeros((2,1))
uccw = np.zeros((2,1))

alpha = 0.025
g_mar = 0.05
epsilon = -0.018
delta = 0.2
vo= 1.0
c = 0.00472
e21 = 0
e22 = 0
e23 = 0
e24 = 0
e25 = 0
e26 = 0
e27 = 0
e28 = 0
e1 = 0
e2 = 0

def publish_goal():
    goal_point = PointStamped()
    goal_point.point.x = float(xg[0][0])
    goal_point.point.y = float(xg[1][0])
    goal_point.point.z = 0.096
    goal_point.header.stamp = rospy.Time.now()
    goal_point.header.frame_id = "map"
    goal_publisher.publish(goal_point)

def localization_callback(msg):
    x[0][0] = msg.point.x
    x[1][0] = msg.point.y
    egtg = xg-x
    #print(egtg)
    e1 = np.linalg.norm(egtg)
    #print(e1)
    kgtg = vo*(1-math.exp(-alpha*np.power(e1,2)))/e1
    #print(kgtg)
    eogtg = -kgtg*egtg
    ugtg = -eogtg
    #print(ugtg)

def avoid_obs_1(msg):
    xo1[0][0] = msg.point.x
    xo1[1][0] = msg.point.y
    eao = xo1-x
    e21 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e21*(np.power(e21,2)+epsilon))
    #print(kao)
    uao1 = kao*eao
    #print(uao)

def avoid_obs_2(msg):
    xo2[0][0] = msg.point.x
    xo2[1][0] = msg.point.y
    eao = xo2-x
    e22 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e22*(np.power(e22,2)+epsilon))
    #print(kao)
    uao2 = kao*eao
    #print(uao)

def avoid_obs_3(msg):
    xo3[0][0] = msg.point.x
    xo3[1][0] = msg.point.y
    eao = xo3-x
    e23 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e23*(np.power(e23,2)+epsilon))
    #print(kao)
    uao3 = kao*eao
    #print(uao)

def avoid_obs_4(msg):
    xo4[0][0] = msg.point.x
    xo4[1][0] = msg.point.y
    eao = xo4-x
    e24 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e24*(np.power(e24,2)+epsilon))
    #print(kao)
    uao4 = kao*eao
    #print(uao)

def avoid_obs_5(msg):
    xo5[0][0] = msg.point.x
    xo5[1][0] = msg.point.y
    eao = xo5-x
    e25 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e25*(np.power(e25,2)+epsilon))
    #print(kao)
    uao5 = kao*eao
    #print(uao)

def avoid_obs_6(msg):
    xo6[0][0] = msg.point.x
    xo6[1][0] = msg.point.y
    eao = xo6-x
    e26 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e26*(np.power(e26,2)+epsilon))
    #print(kao)
    uao6 = kao*eao
    #print(uao)

def avoid_obs_7(msg):
    xo7[0][0] = msg.point.x
    xo7[1][0] = msg.point.y
    eao = xo7-x
    e27 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e27*(np.power(e27,2)+epsilon))
    #print(kao)
    uao7 = kao*eao
    #print(uao)

def avoid_obs_8(msg):
    xo8[0][0] = msg.point.x
    xo8[1][0] = msg.point.y
    eao = xo8-x
    e28 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e28*(np.power(e28,2)+epsilon))
    #print(kao)
    uao8 = kao*eao
    #print(uao)


def weighted_avoidance()
    dist_weight = [math.exp(-e21),math.exp(-e22),math.exp(-e23),math.exp(-e24),math.exp(-e25),math.exp(-e26),math.exp(-e27),math.exp(-e28)]

    dir_weight =   [(np.dot(-uao1,ugtg)/(np.linalg.norm(uao1)*np.linalg.norm(ugtg))) if (np.dot(-uao1,ugtg)/(np.linalg.norm(uao1)*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao2,ugtg)/(np.linalg.norm(uao2)*np.linalg.norm(ugtg))) if (np.dot(-uao2,ugtg)/(np.linalg.norm(uao2)*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao3,ugtg)/(np.linalg.norm(uao3)*np.linalg.norm(ugtg))) if (np.dot(-uao3,ugtg)/(np.linalg.norm(uao3)*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao4,ugtg)/(np.linalg.norm(uao4)*np.linalg.norm(ugtg))) if (np.dot(-uao4,ugtg)/(np.linalg.norm(uao4)*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao5,ugtg)/(np.linalg.norm(uao5)*np.linalg.norm(ugtg))) if (np.dot(-uao5,ugtg)/(np.linalg.norm(uao5)*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao6,ugtg)/(np.linalg.norm(uao6)*np.linalg.norm(ugtg))) if (np.dot(-uao6,ugtg)/(np.linalg.norm(uao6)*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao7,ugtg)/(np.linalg.norm(uao7)*np.linalg.norm(ugtg))) if (np.dot(-uao7,ugtg)/(np.linalg.norm(uao7)*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao8,ugtg)/(np.linalg.norm(uao8)*np.linalg.norm(ugtg))) if (np.dot(-uao8,ugtg)/(np.linalg.norm(uao8)*np.linalg.norm(ugtg))) >= 0 else 0]

    xo = (xo1-x)*dist_weight[0]*dir_weight[0] + (xo2-x)*dist_weight[1]*dir_weight[1] + (xo3-x)*dist_weight[2]*dir_weight[2] + (xo4-x)*dist_weight[3]*dir_weight[3] + (xo5-x)*dist_weight[4]*dir_weight[4] + (xo6-x)*dist_weight[5]*dir_weight[5] + (xo7-x)*dist_weight[6]*dir_weight[6] + (xo8-x)*dist_weight[7]*dir_weight[7]
    e2 = np.linalg.norm(xo)
    kao = c/(e2*(np.power(e2,2)+epsilon))
    #print(kao)
    uao = kao*eao




#1. Goal to goal

#xg = np.array([[10],[15]])
#egtg = xg-x
#print(egtg)
#e1 = np.linalg.norm(egtg)
#print(e1)
#kgtg = vo*(1-math.exp(-alpha*np.power(e1,2)))/e1
#print(kgtg)
#eogtg = -kgtg*egtg
#ugtg = -eogtg
#print(ugtg)

#2. Avoid obstacle

#xo = np.array([[6],[4]])
#eao = xo-x
#e2 = np.linalg.norm(eao)
#print(e2)
#kao = c/(e2*(np.power(e2,2)+epsilon))
#print(kao)
#uao = kao*eao
#print(uao)

#3. Follow wall

def follow_wall():
    t = np.array([[0,1],[-1,0]])
    ucw = alpha*(t@uao)
    uccw = alpha*(-t@uao)
    #Unit vectors:
    uv0 = ugtg/np.linalg.norm(ugtg)
    uv1 = ucw/np.linalg.norm(ucw)
    uv2 = uccw/np.linalg.norm(uccw)
    uv3 = uao/np.linalg.norm(uao)

    #print(uccw)
    dirc = np.dot(uv0.T,uv1)
    dircc = np.dot(uv0.T,uv2)
    print(dirc,dircc)
    dirao = np.dot(uv0.T,uv3)
    print(dirao)

#4. Automation
u=ugtg

class gtg(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['ftwc','ftwcc','stop'],input_keys=['dT'],output_keys=['dT'])

    def execute(self, userdata):
        rospy.loginfo('Executing state go-to-goal')
        while(e2>delta and e1>=g_mar):
            u=ugtg

        if e1 <= epsilon:
           return 'stop'
        else:
            if dirc>0 and e2<=delta:
                userdata.dT=e2
                return 'ftwc'
            else:
                userdata.dT=e2
                return 'ftwcc'

class ao(smach.State):
    def __init__(self):
         smach.State.__init__(self,outcomes=['ftwc','ftwcc'],input_keys=['dT'],output_keys=['dT'])

    def execute(self,userdata):
        rospy.loginfo('Executing state avoid obstacle')
        while(e2<=delta):
            u=uao

        if dirc>0 and e2<=delta:
            userdata.dT=e2
            return 'ftwc'
        else:
            userdata.dT=e2
            return 'ftwcc'

class ftwc(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['ao','gtg'],input_keys=['dT'],output_keys=['dT'])

    def execute(self,userdata):
        rospy.loginfo('Executing ftw clockwise')
        while e1>=userdata.dT and dirao<=0 and e2>=delta:
            u=ucw
        if e1<=userdata.dT or dirao>0:
            return 'gtg'
        else:
            return 'ao'



class ftwcc(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['gtg','ao'],input_keys=['dT'],output_keys=['dT'])

    def execute(self,userdata):
        rospy.loginfo('Executing ftw counter clockwise')
        while e1>=userdata.dT and dirao<=0 and e2>=delta:
            u=uccw
        if e1<=userdata.dT or dirao>0:
            return 'gtg'
        else:
            return 'ao'

def my_loop():

    bot = smach.StateMachine(outcomes=['At_Destination'])

    with bot:
        smach.StateMachine.add('gtg',gtg(),transitions={'ftwc':'ftwc','ftwcc':'ftwcc','stop':'At_Destination'})
        smach.StateMachine.add('ao',ao(),transitions={'ftwc':'ftwc','ftwcc':'ftwcc'})
        smach.StateMachine.add('ftwc',ftwc(),transitions={'gtg':'gtg','ao':'ao'})
        smach.StateMachine.add('ftwcc',ftwcc(),transitions={'gtg':'gtg','ao':'ao'})

    outcome = bot.execute()


def publish_vel():
    vel_msg = Twist()
    vel_msg.linear.x = u[0][0]
    vel_msg.linear.y = u[1][0]
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


# Run my_loop() function when starting code:
if __name__ == "__main__":
    while not rospy.is_shutdown():

        rospy.init_node('sm_bot')
        rospy.Subcriber('/localization_data_topic',localization_callback)
        rospy.Subscriber('/abs_ir1',avoid_obs_1)
        rospy.Subscriber('/abs_ir2',avoid_obs_2)
        rospy.Subscriber('/abs_ir3',avoid_obs_3)
        rospy.Subscriber('/abs_ir4',avoid_obs_4)
        rospy.Subscriber('/abs_ir5',avoid_obs_5)
        rospy.Subscriber('/abs_ir6',avoid_obs_6)
        rospy.Subscriber('/abs_ir7',avoid_obs_7)
        rospy.Subscriber('/abs_ir8',avoid_obs_8)
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        goal_publisher = rospy.Publisher('/goal',PointStamped,queue_size = 1)
        publish_goal()
        weighted_avoidance()
        follow_wall()
        my_loop()
        publish_vel()

# Create and start the introspection server
sis = smach_ros.IntrospectionServer('server_name', bot, '/SM_ROOT')
sis.start()

# Wait for ctrl-c to stop the application
rospy.spin()
sis.stop()
