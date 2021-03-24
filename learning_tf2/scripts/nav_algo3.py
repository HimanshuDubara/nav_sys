#! /usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import PointStamped,Twist
from enum import Enum





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
xg[0][0] = 0.5
xg[1][0] = -0.7
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
epsilon = -0.008
delta = 0.2
vo= 0
dirc = 0
dircc = 0
dtfwc =0
dtfwcc = 0
dirao = 0
c = 0.00472
e21= 0
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
    print(ugtg)

def avoid_obs_1(msg):
    xo1[0][0] = msg.point.x
    xo1[1][0] = msg.point.y
    eao = xo1-x
    e21 = np.linalg.norm(eao)
    #print(e2)
    kao = c#/(e21*(np.power(e21,2)+epsilon))
    #print(kao)
    uao1 = kao*eao
    #print(uao)

def avoid_obs_2(msg):
    xo2[0][0] = msg.point.x
    xo2[1][0] = msg.point.y
    eao = xo2-x
    e22 = np.linalg.norm(eao)
    #print(e2)
    kao = c#/(e22*(np.power(e22,2)+epsilon))
    #print(kao)
    uao2 = kao*eao
    #print(uao)

def avoid_obs_3(msg):
    xo3[0][0] = msg.point.x
    xo3[1][0] = msg.point.y
    eao = xo3-x
    e23 = np.linalg.norm(eao)
    #print(e2)
    kao = c#/(e23*(np.power(e23,2)+epsilon))
    #print(kao)
    uao3 = kao*eao
    #print(uao)

def avoid_obs_4(msg):
    xo4[0][0] = msg.point.x
    xo4[1][0] = msg.point.y
    eao = xo4-x
    e24 = np.linalg.norm(eao)
    #print(e2)
    kao = c#/(e24*(np.power(e24,2)+epsilon))
    #print(kao)
    uao4 = kao*eao
    #print(uao)

def avoid_obs_5(msg):
    xo5[0][0] = msg.point.x
    xo5[1][0] = msg.point.y
    eao = xo5-x
    e25 = np.linalg.norm(eao)
    #print(e2)
    kao = c#/(e25*(np.power(e25,2)+epsilon))
    #print(kao)
    uao5 = kao*eao
    #print(uao)

def avoid_obs_6(msg):
    xo6[0][0] = msg.point.x
    xo6[1][0] = msg.point.y
    eao = xo6-x
    e26 = np.linalg.norm(eao)
    #print(e2)
    kao = c#/(e26*(np.power(e26,2)+epsilon))
    #print(kao)
    uao6 = kao*eao
    #print(uao)

def avoid_obs_7(msg):
    xo7[0][0] = msg.point.x
    xo7[1][0] = msg.point.y
    eao = xo7-x
    e27 = np.linalg.norm(eao)
    #print(e2)
    kao = c#/(e27*(np.power(e27,2)+epsilon))
    #print(kao)
    uao7 = kao*eao
    #print(uao)

def avoid_obs_8(msg):
    xo8[0][0] = msg.point.x
    xo8[1][0] = msg.point.y
    eao = xo8-x
    e28 = np.linalg.norm(eao)
    #print(e2)
    kao = c#/(e28*(np.power(e28,2)+epsilon))
    #print(kao)
    uao8 = kao*eao
    #print(uao)


def weighted_avoidance():
    dist_weight = [math.exp(-e21),math.exp(-e22),math.exp(-e23),math.exp(-e24),math.exp(-e25),math.exp(-e26),math.exp(-e27),math.exp(-e28)]

    dir_weight =   [(np.dot(-uao1.transpose(),ugtg)/(np.linalg.norm(uao1.transpose())*np.linalg.norm(ugtg))) if (np.dot(-uao1.transpose(),ugtg)/(np.linalg.norm(uao1.transpose())*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao2.transpose(),ugtg)/(np.linalg.norm(uao2.transpose())*np.linalg.norm(ugtg))) if (np.dot(-uao2.transpose(),ugtg)/(np.linalg.norm(uao2.transpose())*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao3.transpose(),ugtg)/(np.linalg.norm(uao3.transpose())*np.linalg.norm(ugtg))) if (np.dot(-uao3.transpose(),ugtg)/(np.linalg.norm(uao3.transpose())*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao4.transpose(),ugtg)/(np.linalg.norm(uao4.transpose())*np.linalg.norm(ugtg))) if (np.dot(-uao4.transpose(),ugtg)/(np.linalg.norm(uao4.transpose())*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao5.transpose(),ugtg)/(np.linalg.norm(uao5.transpose())*np.linalg.norm(ugtg))) if (np.dot(-uao5.transpose(),ugtg)/(np.linalg.norm(uao5.transpose())*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao6.transpose(),ugtg)/(np.linalg.norm(uao6.transpose())*np.linalg.norm(ugtg))) if (np.dot(-uao6.transpose(),ugtg)/(np.linalg.norm(uao6.transpose())*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao7.transpose(),ugtg)/(np.linalg.norm(uao7.transpose())*np.linalg.norm(ugtg))) if (np.dot(-uao7.transpose(),ugtg)/(np.linalg.norm(uao7.transpose())*np.linalg.norm(ugtg))) >= 0 else 0,
                    (np.dot(-uao8.transpose(),ugtg)/(np.linalg.norm(uao8.transpose())*np.linalg.norm(ugtg))) if (np.dot(-uao8.transpose(),ugtg)/(np.linalg.norm(uao8.transpose())*np.linalg.norm(ugtg))) >= 0 else 0]


    xo = (xo1-x)*dist_weight[0]*dir_weight[0] + (xo2-x)*dist_weight[1]*dir_weight[1] + (xo3-x)*dist_weight[2]*dir_weight[2] + (xo4-x)*dist_weight[3]*dir_weight[3] + (xo5-x)*dist_weight[4]*dir_weight[4] + (xo6-x)*dist_weight[5]*dir_weight[5] + (xo7-x)*dist_weight[6]*dir_weight[6] + (xo8-x)*dist_weight[7]*dir_weight[7]
    e2 = np.linalg.norm(xo)
    kao = c#/(e2*(np.power(e2,2)+epsilon))
    #print(kao)
    uao = kao*xo




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
    ucw = alpha*(np.dot(t,uao))
    uccw = alpha*(np.dot(-t,uao))
    #Unit vectors:
    uv0 = ugtg/np.linalg.norm(ugtg)
    uv1 = ucw/np.linalg.norm(ucw)
    uv2 = uccw/np.linalg.norm(uccw)
    uv3 = uao/np.linalg.norm(uao)

    #print(uccw)
    dirc = np.dot(uv0.transpose(),uv1)
    dircc = np.dot(uv0.transpose(),uv2)
    #print(dirc,dircc)
    dirao = np.dot(uv0.transpose(),uv3)
    #print(dirao)

##############################################################################

class States(Enum):
    GTG = 1
    FWC = 2
    FWCC = 3
    AO = 4
    STP = 5


class Automaton:
    def __init__(self):
        self.__state = States.GTG

    def set_state(self, state):
        self.__state = state

    def get_state(self):
        return self.__state

################################################################################
if __name__ == '__main__':
    rospy.init_node('nav_bot', anonymous = True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    goal_publisher = rospy.Publisher('/goal',PointStamped,queue_size = 1)
    automaton = Automaton()
    while not rospy.is_shutdown():
        rospy.Subscriber('/localization_data_topic',PointStamped,localization_callback)
        rospy.Subscriber('/abs_ir1',PointStamped,avoid_obs_1)
        rospy.Subscriber('/abs_ir2',PointStamped,avoid_obs_2)
        rospy.Subscriber('/abs_ir3',PointStamped,avoid_obs_3)
        rospy.Subscriber('/abs_ir4',PointStamped,avoid_obs_4)
        rospy.Subscriber('/abs_ir5',PointStamped,avoid_obs_5)
        rospy.Subscriber('/abs_ir6',PointStamped,avoid_obs_6)
        rospy.Subscriber('/abs_ir7',PointStamped,avoid_obs_7)
        rospy.Subscriber('/abs_ir8',PointStamped,avoid_obs_8)
        publish_goal()
        weighted_avoidance()
        follow_wall()
        ############# Working of automaton ###############

        if automaton.get_state() == States.GTG:
            if e1<= epsilon:
                automaton.set_state(States.STP)
            elif e2<=delta and dirc>0:
                automaton.set_state(States.FWC)
            elif e2<=delta and dircc>0:
                automaton.set_state(States.FWCC)
            else:
                u = ugtg
                print("GTG")
                print(u)


        elif automaton.get_state() == States.FWC:
            if setfwc == 0:
                dtfwc = e1
                u = ucw
                setfwc = 1
            elif dirao>0 and e1<dtfwc:
                automaton.set_state(States.GTG)
                setfwc = 0
            elif e2 < delta/2:
                automaton.set_state(States.AO)
                setfwc = 0
            else:
                u = ucw
                print("FWC")
                print(u)


        elif automaton.get_state() == States.FWCC:
            if setfwcc == 0:
                dtfwcc = e1
                u = uccw
                setfwcc = 1
            elif dirao>0 and e1<dtfwc:
                automaton.set_state(States.GTG)
                setfwcc = 0
            elif e2 < delta/2:
                automaton.set_state(States.AO)
                setfwcc = 0
            else:
                u = uccw
                print("FWCC")
                print(u)



        elif automaton.get_state() == States.AO:
            if e2 >=delta/2:
                if dirc>0:
                    automaton.set_state(States.FWC)
                else:
                    automaton.set_state(States.FWCC)
            else:
                u = uao
                print("AO")
                print(u)


        else:
            u = np.zeros((2,1))
            print("STP")
            print(u)
######################## Publishing velocity ######################################
        vel_msg = Twist()
        vel_msg.linear.x = u[0][0]
        vel_msg.linear.y = u[1][0]
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
