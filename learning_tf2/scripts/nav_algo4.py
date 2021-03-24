#! /usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import PointStamped,Twist
from enum import Enum



class Robot:
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
    setfwc = 0
    setfwcc = 0
    def __init__(self):
            self.x = np.zeros((2,1))
            self.xg = np.zeros((2,1))
            self.xo = np.zeros((2,1))
            self.xo1 = np.zeros((2,1))
            self.xo2 = np.zeros((2,1))
            self.xo3 = np.zeros((2,1))
            self.xo4 = np.zeros((2,1))
            self.xo5 = np.zeros((2,1))
            self.xo6 = np.zeros((2,1))
            self.xo7 = np.zeros((2,1))
            self.xo8 = np.zeros((2,1))
            self.ugtg = np.zeros((2,1))
            #self.u = np.zeros((2,1))
            self.uao1  =  np.zeros((2,1))
            self.uao2 =  np.zeros((2,1))
            self.uao3 = np.zeros((2,1))
            self.uao4 =  np.zeros((2,1))
            self.uao5 =  np.zeros((2,1))
            self.uao6 =  np.zeros((2,1))
            self.uao7 =  np.zeros((2,1))
            self.uao8 =  np.zeros((2,1))

            self.uao = np.zeros((2,1))
            self.ucw = np.zeros((2,1))
            self.uccw = np.zeros((2,1))
            self.alpha = 0.025
            self.g_mar = 0.05
            self.epsilon = -0.008
            self.delta = 0.2
            self.vo= 0
            self.dirc = 0
            self.dircc = 0
            self.dtfwc =0
            self.dtfwcc = 0
            self.dirao = 0
            self.c = 0.00472
            self.e21= 0
            self.e22 = 0
            self.e23 = 0
            self.e24 = 0
            self.e25 = 0
            self.e26 = 0
            self.e27 = 0
            self.e28 = 0
            self.e1 = 0
            self.e2 = 0
            self.setfwcc = 0
            self.setfwc = 0

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

    def localization_callback(self,msg):
        self.x[0][0] = msg.point.x
        self.x[1][0] = msg.point.y
        egtg = xg-x
                #print(egtg)
        self.e1 = np.linalg.norm(egtg)
                #print(e1)
        kgtg = vo*(1-math.exp(-alpha*np.power(e1,2)))/e1
                #print(kgtg)
        eogtg = -kgtg*egtg
        self.ugtg = -eogtg
        print(ugtg)

    def avoid_obs_1(self,msg):
        self.xo1[0][0] = msg.point.x
        self.xo1[1][0] = msg.point.y
        eao = xo1-x
        self.e21 = np.linalg.norm(eao)
                #print(e2)
        kao = self.c#/(e21*(np.power(e21,2)+epsilon))
                #print(kao)
        self.uao1 = kao*eao
                #print(uao)

    def avoid_obs_2(self,msg):
        self.xo2[0][0] = msg.point.x
        self.xo2[1][0] = msg.point.y
        eao = xo2-x
        self.e22 = np.linalg.norm(eao)
                #print(e2)
        kao = self.c#/(e22*(np.power(e22,2)+epsilon))
                #print(kao)
        self.uao2 = kao*eao
                #print(uao)

    def avoid_obs_3(self,msg):
        self.xo3[0][0] = msg.point.x
        self.xo3[1][0] = msg.point.y
        eao = xo3-x
        self.e23 = np.linalg.norm(eao)
                #print(e2)
        kao = self.c#/(e23*(np.power(e23,2)+epsilon))
                #print(kao)
        self.uao3 = kao*eao
                #print(uao)

    def avoid_obs_4(self,msg):
        self.xo4[0][0] = msg.point.x
        self.xo4[1][0] = msg.point.y
        eao = xo4-x
        self.e24 = np.linalg.norm(eao)
                #print(e2)
        kao = self.c#/(e24*(np.power(e24,2)+epsilon))
                #print(kao)
        self.uao4 = kao*eao
                #print(uao)

    def avoid_obs_5(self,msg):
        self.xo5[0][0] = msg.point.x
        self.xo5[1][0] = msg.point.y
        eao = xo5-x
        self.e25 = np.linalg.norm(eao)
                #print(e2)
        kao = self.c#/(e25*(np.power(e25,2)+epsilon))
                #print(kao)
        self.uao5 = kao*eao
                #print(uao)

    def avoid_obs_6(self,msg):
        self.xo6[0][0] = msg.point.x
        self.xo6[1][0] = msg.point.y
        eao = xo6-x
        self.e26 = np.linalg.norm(eao)
                #print(e2)
        kao = self.c#/(e26*(np.power(e26,2)+epsilon))
                #print(kao)
        self.uao6 = kao*eao
                #print(uao)

    def avoid_obs_7(self,msg):
        self.xo7[0][0] = msg.point.x
        self.xo7[1][0] = msg.point.y
        eao = xo7-x
        self.e27 = np.linalg.norm(eao)
                #print(e2)
        kao = self.c#/(e27*(np.power(e27,2)+epsilon))
                #print(kao)
        self.uao7 = kao*eao
                #print(uao)

    def avoid_obs_8(self,msg):
        self.xo8[0][0] = msg.point.x
        self.xo8[1][0] = msg.point.y
        eao = xo8-x
        self.e28 = np.linalg.norm(eao)
                #print(e2)
        kao = self.c#/(e28*(np.power(e28,2)+epsilon))
                #print(kao)
        self.uao8 = kao*eao
                #print(uao)

    def weighted_avoidance(self):
        dist_weight = [math.exp(-self.e21),math.exp(-self.e22),math.exp(-self.e23),math.exp(-self.e24),math.exp(-self.e25),math.exp(-self.e26),math.exp(-self.e27),math.exp(-self.e28)]

        dir_weight =   [(np.dot(-self.uao1.transpose(),self.ugtg)/(np.linalg.norm(self.uao1.transpose())*np.linalg.norm(self.ugtg))) if (np.dot(-self.uao1.transpose(),self.ugtg)/(np.linalg.norm(self.uao1.transpose())*np.linalg.norm(self.ugtg))) >= 0 else 0,
                        (np.dot(-self.uao2.transpose(),self.ugtg)/(np.linalg.norm(self.uao2.transpose())*np.linalg.norm(sef.ugtg))) if (np.dot(-self.uao2.transpose(),self.ugtg)/(np.linalg.norm(self.uao2.transpose())*np.linalg.norm(self.ugtg))) >= 0 else 0,
                        (np.dot(-self.uao3.transpose(),self.ugtg)/(np.linalg.norm(self.uao3.transpose())*np.linalg.norm(self.ugtg))) if (np.dot(-self.uao3.transpose(),self.ugtg)/(np.linalg.norm(self.uao3.transpose())*np.linalg.norm(self.ugtg))) >= 0 else 0,
                        (np.dot(-self.uao4.transpose(),self.ugtg)/(np.linalg.norm(self.uao4.transpose())*np.linalg.norm(self.ugtg))) if (np.dot(-self.uao4.transpose(),self.ugtg)/(np.linalg.norm(self.uao4.transpose())*np.linalg.norm(self.ugtg))) >= 0 else 0,
                        (np.dot(-self.uao5.transpose(),self.ugtg)/(np.linalg.norm(self.uao5.transpose())*np.linalg.norm(self.ugtg))) if (np.dot(-self.uao5.transpose(),self.ugtg)/(np.linalg.norm(self.uao5.transpose())*np.linalg.norm(self.ugtg))) >= 0 else 0,
                        (np.dot(-self.uao6.transpose(),self.ugtg)/(np.linalg.norm(self.uao6.transpose())*np.linalg.norm(self.ugtg))) if (np.dot(-self.uao6.transpose(),self.ugtg)/(np.linalg.norm(self.uao6.transpose())*np.linalg.norm(self.ugtg))) >= 0 else 0,
                        (np.dot(-self.uao7.transpose(),self.ugtg)/(np.linalg.norm(self.uao7.transpose())*np.linalg.norm(self.ugtg))) if (np.dot(-self.uao7.transpose(),self.ugtg)/(np.linalg.norm(self.uao7.transpose())*np.linalg.norm(self.ugtg))) >= 0 else 0,
                        (np.dot(-self.uao8.transpose(),self.ugtg)/(np.linalg.norm(self.uao8.transpose())*np.linalg.norm(self.ugtg))) if (np.dot(-self.uao8.transpose(),self.ugtg)/(np.linalg.norm(self.uao8.transpose())*np.linalg.norm(self.ugtg))) >= 0 else 0]


        self.xo = (self.xo1-self.x)*dist_weight[0]*dir_weight[0] + (self.xo2-self.x)*dist_weight[1]*dir_weight[1] + (self.xo3-self.x)*dist_weight[2]*dir_weight[2] + (self.xo4-self.x)*dist_weight[3]*dir_weight[3] + (self.xo5-self.x)*dist_weight[4]*dir_weight[4] + (self.xo6-self.x)*dist_weight[5]*dir_weight[5] + (self.xo7-self.x)*dist_weight[6]*dir_weight[6] + (self.xo8-self.x)*dist_weight[7]*dir_weight[7]
        self.e2 = np.linalg.norm(self.xo)
        kao = self.c#/(e2*(np.power(e2,2)+epsilon))
                #print(kao)
        self.uao = kao*self.xo

    def follow_wall(self):
        t = np.array([[0,1],[-1,0]])
        self.ucw = self.alpha*(np.dot(t,self.uao))
        self.uccw = self.alpha*(np.dot(-t,self.uao))
                #Unit vectors:
        uv0 = self.ugtg/np.linalg.norm(self.ugtg)
        uv1 = self.ucw/np.linalg.norm(self.ucw)
        uv2 = self.uccw/np.linalg.norm(self.uccw)
        uv3 = self.uao/np.linalg.norm(self.uao)

                #print(uccw)
        self.dirc = np.dot(uv0.transpose(),uv1)
        self.dircc = np.dot(uv0.transpose(),uv2)
                #print(dirc,dircc)
        self.dirao = np.dot(uv0.transpose(),uv3)
                #print(dirao)

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
    r = Robot()
    a = Automaton()
    while not rospy.is_shutdown():
        rospy.Subscriber('/localization_data_topic',PointStamped,r.localization_callback)
        rospy.Subscriber('/abs_ir1',PointStamped,r.avoid_obs_1)
        rospy.Subscriber('/abs_ir2',PointStamped,r.avoid_obs_2)
        rospy.Subscriber('/abs_ir3',PointStamped,r.avoid_obs_3)
        rospy.Subscriber('/abs_ir4',PointStamped,r.avoid_obs_4)
        rospy.Subscriber('/abs_ir5',PointStamped,r.avoid_obs_5)
        rospy.Subscriber('/abs_ir6',PointStamped,r.avoid_obs_6)
        rospy.Subscriber('/abs_ir7',PointStamped,r.avoid_obs_7)
        rospy.Subscriber('/abs_ir8',PointStamped,r.avoid_obs_8)
        r.publish_goal()
        r.weighted_avoidance()
        r.follow_wall()
        ############# Working of automaton ###############

        if a.get_state() == States.GTG:
            if r.e1<= r.epsilon:
                automaton.set_state(States.STP)
            elif r.e2<=r.delta and r.dirc>0:
                automaton.set_state(States.FWC)
            elif r.e2<=r.delta and r.dircc>0:
                automaton.set_state(States.FWCC)
            else:
                u = r.ugtg
                print("GTG")
                print(u)


        elif a.get_state() == States.FWC:
            if r.setfwc == 0:
                r.dtfwc = e1
                u = r.ucw
                r.setfwc = 1
            elif r.dirao>0 and r.e1<r.dtfwc:
                a.set_state(States.GTG)
                r.setfwc = 0
            elif r.e2 < r.delta/2:
                a.set_state(States.AO)
                r.setfwc = 0
            else:
                u = r.ucw
                print("FWC")
                print(u)


        elif automaton.get_state() == States.FWCC:
            if r.setfwcc == 0:
                r.dtfwcc = e1
                u = r.uccw
                r.setfwcc = 1
            elif r.dirao>0 and r.e1<r.dtfwc:
                a.set_state(States.GTG)
                r.setfwcc = 0
            elif r.e2 < r.delta/2:
                a.set_state(States.AO)
                r.setfwcc = 0
            else:
                u = r.uccw
                print("FWCC")
                print(u)



        elif a.get_state() == States.AO:
            if r.e2 >=r.delta/2:
                if r.dirc>0:
                    a.set_state(States.FWC)
                else:
                    a.set_state(States.FWCC)
            else:
                u = r.uao
                print("AO")
                print(u)


        else:
            u = np.zeros((2,1))
            print("STP")
            print(u)
######################## Publishing velocity ######################################
        vel_msg = Twist()
        vel_msg.linear.x = float(u[0][0])
        vel_msg.linear.y = float(u[1][0])
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
