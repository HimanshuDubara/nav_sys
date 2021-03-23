#! /usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import PointStamped,Twist


rospy.init_node('nav_bot', anonymous = True)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
goal_publisher = rospy.Publisher('/goal',PointStamped,queue_size = 1)


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
dirc = 0
dircc = 0
dtfwc =0
dtfwcc = 0
dirao = 0
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


def weighted_avoidance():
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
    ucw = alpha*(np.dot(t,uao))
    uccw = alpha*(np.dot(-t,uao))
    #Unit vectors:
    uv0 = ugtg/np.linalg.norm(ugtg)
    uv1 = ucw/np.linalg.norm(ucw)
    uv2 = uccw/np.linalg.norm(uccw)
    uv3 = uao/np.linalg.norm(uao)

    #print(uccw)
    dirc = np.dot(uv0,uv1)
    dircc = np.dot(uv0,uv2)
    print(dirc,dircc)
    dirao = np.dot(uv0,uv3)
    print(dirao)


#########################################################################################################################

##===============================================
## FINITE STATE MACHINES

class FSM(object):
	#''' Holds the states and transitions available,
	#	executes current states main functions and transitions '''
	def __init__(self, character):
		self.char = character
		self.states = {}
		self.transitions = {}
		self.curState = None
		self.prevState = None ## USE TO PREVENT LOOPING 2 STATES FOREVER
		self.trans = None

	def AddTransition(self, transName, transition):
		self.transitions[transName] = transition

	def AddState(self, stateName, state):
		self.states[stateName] = state

	def SetState(self, stateName):
		self.prevState = self.curState
		self.curState = self.states[stateName]

	def ToTransition(self, toTrans):
		self.trans = self.transitions[toTrans]

	def Execute(self):
		if (self.trans):
			self.curState.Exit()
			self.trans.Execute()
			self.SetState(self.trans.toState)
			self.curState.Enter()
			self.trans = None
		self.curState.Execute()



##===============================================
## TRANSITIONS
class Transition(object):
	''' Code executed when transitioning from one state to another '''
	def __init__(self, toState):
		self.toState = toState

	def Execute(self):
		print ("Transitioning...")


##===============================================

## STATES
class State(object):
	''' The base template state which all others will inherit from  '''
	def __init__(self, FSM):
		self.FSM = FSM

	def Enter(self):
		pass
	def Execute (self):
		pass
	def Exit(self):
		pass

class GoToGoal(State):
	#''' Going to the goal '''
	def __init__(self, FSM):
		super(GoToGoal, self).__init__(FSM)

	def Enter(self):
		print ("Entering GoToGoal")
		#super(GoToGoal, self).Enter()

	def Execute (self):
		print ("GoToGoal")
		u = ugtg
        vel_msg = Twist()
        vel_msg.linear.x = u[0][0]
        vel_msg.linear.y = u[1][0]
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        if e1<= epsilon:
            self.FSM.ToTransition("toStop")
        else:
            if e2<=delta and dirc>0:
                self.FSM.ToTransition("toFWC")
            if e2<=delta and dircc>0:
                self.FSM.ToTransition("toFWCC")

	def Exit(self):
	       print ("Exiting GoToGoal")

class FWC(State):
	#''' State for vacuuming '''
	def __init__(self, FSM):
		super(FWC, self).__init__(FSM)


	def Enter(self):
		print ("Entering into FWC")
		super(FWC, self).Enter()
        dtfwc = e1

	def Execute(self):
		print ("FWC")
        u = ucw
        vel_msg = Twist()
        vel_msg.linear.x = u[0][0]
        vel_msg.linear.y = u[1][0]
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

        if dirao>0 and e1<dtfwc:
            self.FSM.ToTransition("toGoToGoal")
        if e2 < delta/2:
            self.FSM.ToTransition("toAO")

	def Exit(self):
		print ("Exiting FWC")
        dtfwc =0

class FWCC(State):
	#''' State for vacuuming '''
	def __init__(self, FSM):
		super(FWCC, self).__init__(FSM)


	def Enter(self):
		print ("Entering into FWCC")
		#super(FWCC, self).Enter()
        dtfwcc = e1

	def Execute(self):
		print ("FWCC")
        u = uccw
        vel_msg = Twist()
        vel_msg.linear.x = u[0][0]
        vel_msg.linear.y = u[1][0]
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

        if dirao>0 and e1<dtfwc:
            self.FSM.ToTransition("toGoToGoal")
        else:
            if e2 <= delta/2:
                self.FSM.ToTransition("toAO")


	def Exit(self):
		print ("Exiting FWCC")
        dtfwcc = 0

class AO(State):
	#''' Going to the goal '''
	def __init__(self, FSM):
		super(AO, self).__init__(FSM)

	def Enter(self):
		print ("Entering AO")
		#super(AO, self).Enter()

	def Execute (self):
		print ("AO")
		u = uao
        vel_msg = Twist()
        vel_msg.linear.x = u[0][0]
        vel_msg.linear.y = u[1][0]
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        if e1<= epsilon:
            self.FSM.ToTransition("toStop")
        else:
            if e2 >=delta/2:
                if dirc>0:
                    self.FSM.ToTransition("toFWC")
                else:
                    self.FSM.ToTransition("toFWCC")

	def Exit(self):
		print ("Exiting AO")


class Stop(State):
	#''' State for Sleeping. Even robots get tired sometimes. :) '''
	def __init__(self, FSM):
		super(Stop, self).__init__(FSM)

	def Enter(self):
		print ("Entering Stop")
		super(Sleep, self).Enter()

	def Execute(self):
		print ("Stop")
        u = uao
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)


	def Exit(self):
		print ("Exiting Stop")



#===============================================
## IMPLEMENTATION

Char = type("Char", (object,), {})


class Nav_Bot(Char):
    def __init__(self):
        self.FSM = FSM(self)
        #STATES
        self.FSM.AddState("GoToGoal", GoToGoal(self.FSM))
        self.FSM.AddState("FWC", FWC(self.FSM))
        self.FSM.AddState("FWCC", FWCC(self.FSM))
        self.FSM.AddState("AO", AO(self.FSM))
        self.FSM.AddState("Stop", Stop(self.FSM))

        #TRANSITIONS
        self.FSM.AddTransition("toGoToGoal", Transition("GoToGoal"))
        self.FSM.AddTransition("toFWC", Transition("FWC"))
        self.FSM.AddTransition("toFWCC", Transition("FWCC"))
        self.FSM.AddTransition("toAO", Transition("AO"))
        self.FSM.AddTransition("toStop", Transition("Stop"))
        self.FSM.SetState("GoToGoal")

    def Execute(self):
        self.FSM.Execute()




if __name__ == '__main__':
    r = Nav_Bot()
    while not rospy.is_shutdown():
        rospy.Subcriber('/localization_data_topic',localization_callback)
        rospy.Subscriber('/abs_ir1',avoid_obs_1)
        rospy.Subscriber('/abs_ir2',avoid_obs_2)
        rospy.Subscriber('/abs_ir3',avoid_obs_3)
        rospy.Subscriber('/abs_ir4',avoid_obs_4)
        rospy.Subscriber('/abs_ir5',avoid_obs_5)
        rospy.Subscriber('/abs_ir6',avoid_obs_6)
        rospy.Subscriber('/abs_ir7',avoid_obs_7)
        rospy.Subscriber('/abs_ir8',avoid_obs_8)
        publish_goal()
        weighted_avoidance()
        follow_wall()
        r.execute()

    print ('Exiting program...')
