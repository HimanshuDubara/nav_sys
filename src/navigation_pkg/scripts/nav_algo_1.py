#!/usr/bin/env python

import numpy as np
import math
import rospy
import smach
import smach_ros

x = np.array([[2],[3]])
alpha = 0.025
epsilon = 0.05
delta = 0.1
vo= 1.0
c = 1.5

#1. Goal to goal

xg = np.array([[10],[15]])
egtg = xg-x
print(egtg)
e1 = np.linalg.norm(egtg)
print(e1)
kgtg = vo*(1-math.exp(-alpha*np.power(e1,2)))/e1
print(kgtg)
eogtg = -kgtg*egtg
ugtg = -eogtg
print(ugtg)

#2. Avoid obstacle

xo = np.array([[6],[4]])
eao = xo-x
e2 = np.linalg.norm(eao)
print(e2)
kao = c/(e2*(np.power(e2,2)+epsilon))
print(kao)
uao = kao*eao
print(uao)

#3. Follow wall

t = np.array([[0,1],[-1,0]])
ucw = alpha*(t@uao)
uccw = alpha*(-t@uao)
#Unit vectors:
uv0 = ugtg/np.linalg.norm(ugtg)   
uv1 = ucw/np.linalg.norm(ucw)
uv2 = uccw/np.linalg.norm(uccw)
uv3 = uao/np.linalg.norm(uao)

print(uccw)
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
    rospy.init_node('sm_bot')
    bot = smach.StateMachine(outcomes=['At_Destination'])

    with bot:
        smach.StateMachine.add('gtg',gtg(),transitions={'ftwc':'ftwc','ftwcc':'ftwcc','stop':'At_Destination'}) 
        smach.StateMachine.add('ao',ao(),transitions={'ftwc':'ftwc','ftwcc':'ftwcc'})
        smach.StateMachine.add('ftwc',ftwc(),transitions={'gtg':'gtg','ao':'ao'})
        smach.StateMachine.add('ftwcc',ftwcc(),transitions={'gtg':'gtg','ao':'ao'})  
        
    outcome = bot.execute()    

# Run my_loop() function when starting code:
if __name__ == "__main__":
    my_loop()

# Create and start the introspection server
sis = smach_ros.IntrospectionServer('server_name', bot, '/SM_ROOT')
sis.start()

# Wait for ctrl-c to stop the application
rospy.spin()
sis.stop()
