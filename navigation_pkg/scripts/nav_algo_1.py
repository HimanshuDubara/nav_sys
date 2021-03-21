#!/usr/bin/env python

import numpy as np
import math
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PointStamped,Twist


x = np.zeros((2,1))
xg = np.zeros((2,1))

###Defining goal at -0.5,0.5####
xg[0][0] = -0.5
xg[1][0] = 0.5



uao1  =  np.zeros((2,1))
uao2 =  np.zeros((2,1))
uao3 = np.zeros((2,1))
uao4 =  np.zeros((2,1))
uao5 =  np.zeros((2,1))
uao6 =  np.zeros((2,1))
uao7 =  np.zeros((2,1))
uao8 =  np.zeros((2,1))
xo = np.zeros((2,1))
uao = np.zeros((2,1))
ucw = np.zeros((2,1))
uccw = np.zeros((2,1))
alpha = 0.025
epsilon = 0.05
delta = 0.1
vo= 1.0
c = 1.5
e21 = 0
e22 = 0
e23 = 0
e24 = 0
e25 = 0
e26 = 0
e27 = 0
e28 = 0

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
    xo[0][0] = msg.point.x
    xo[1][0] = msg.point.y
    eao = xo-x
    e21 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e2*(np.power(e2,2)+epsilon))
    #print(kao)
    uao1 = kao*eao
    #print(uao)

def avoid_obs_2(msg):
    xo[0][0] = msg.point.x
    xo[1][0] = msg.point.y
    eao = xo-x
    e22 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e2*(np.power(e2,2)+epsilon))
    #print(kao)
    uao2 = kao*eao
    #print(uao)

def avoid_obs_3(msg):
    xo[0][0] = msg.point.x
    xo[1][0] = msg.point.y
    eao = xo-x
    e23 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e2*(np.power(e2,2)+epsilon))
    #print(kao)
    uao3 = kao*eao
    #print(uao)

def avoid_obs_4(msg):
    xo[0][0] = msg.point.x
    xo[1][0] = msg.point.y
    eao = xo-x
    e24 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e2*(np.power(e2,2)+epsilon))
    #print(kao)
    uao4 = kao*eao
    #print(uao)

def avoid_obs_5(msg):
    xo[0][0] = msg.point.x
    xo[1][0] = msg.point.y
    eao = xo-x
    e25 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e2*(np.power(e2,2)+epsilon))
    #print(kao)
    uao5 = kao*eao
    #print(uao)

def avoid_obs_6(msg):
    xo[0][0] = msg.point.x
    xo[1][0] = msg.point.y
    eao = xo-x
    e26 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e2*(np.power(e2,2)+epsilon))
    #print(kao)
    uao6 = kao*eao
    #print(uao)

def avoid_obs_7(msg):
    xo[0][0] = msg.point.x
    xo[1][0] = msg.point.y
    eao = xo-x
    e27 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e2*(np.power(e2,2)+epsilon))
    #print(kao)
    uao7 = kao*eao
    #print(uao)

def avoid_obs_8(msg):
    xo[0][0] = msg.point.x
    xo[1][0] = msg.point.y
    eao = xo-x
    e28 = np.linalg.norm(eao)
    #print(e2)
    kao = c/(e2*(np.power(e2,2)+epsilon))
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

    uao = uao1*dist_weight[0]*dir_weight[0] + uao2*dist_weight[1]*dir_weight[1] + uao3*dist_weight[2]*dir_weight[2] + uao4*dist_weight[3]*dir_weight[3] + uao5*dist_weight[4]*dir_weight[4] + uao6*dist_weight[5]*dir_weight[5] + uao7*dist_weight[6]*dir_weight[6] + uao8*dist_weight[7]*dir_weight[7]





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
        while(e2>delta and e1>=epsilon):
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


# Run my_loop() function when starting code:
if __name__ == "__main__":
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


    my_loop()

# Create and start the introspection server
sis = smach_ros.IntrospectionServer('server_name', bot, '/SM_ROOT')
sis.start()

# Wait for ctrl-c to stop the application
rospy.spin()
sis.stop()
