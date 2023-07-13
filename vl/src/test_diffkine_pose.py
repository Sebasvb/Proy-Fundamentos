#!/usr/bin/env python3
#
 
from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState
 
#from pyquaternion import Quaternion
 
from markers import *
from functions import *
 
 
# Initialize the node
rospy.init_node("testKineControlPose")
print('starting motion ... ')
# Publisher: publish to the joint_states topic
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
# Files for the logs
fxcurrent = open("/tmp/x.txt", "w")                
fxdesired = open("/tmp/xd.txt", "w")
fq = open("/tmp/q.txt", "w")
# Markers for the current and desired positions
bmarker_current  = FrameMarker()
bmarker_desired = FrameMarker(0.5)
 
# Joint names
jnames = ['waist_q1', 'shoulder_q2', 'slider_q3','elbow_q4', 'revolution_q5', 'wrist_q6']

# Desired pose
Rd = np.array([[0,1,0],[1,0,0],[0,0,-1]])
 
qd = rot2quat(Rd)
print(qd)
# Find an xd that the robot can reach
xd = np.array([0.1, -0.1, 0.2, qd[0], qd[1], qd[2], qd[3]])
xd = np.array([-0.2, 0.2, 0.3, qd[0], qd[1], qd[2], qd[3]])
xd = np.array([0.3, -0.3, 0.2, qd[0], qd[1], qd[2], qd[3]])
# Initial configuration
q0  = np.array([pi/4, pi/4, 0.2, pi/4, pi/4, pi/4])
q0  = np.array([1.43, 1.80, 0.07, -2.2, 0.00, 2.93])
q0  = np.array([0.48, 1.15, 0.22, -3.45, 0.03, 2.13])
 
# Resulting initial pose (end effector with respect to the base link)
T = fkine(q0)
x0 = TF2xyzquat(T)
 
# Markers for the current and the desired pose
bmarker_current.setPose(x0)
bmarker_desired.setPose(xd)
 
# Instance of the JointState message
jstate = JointState()
# Values of the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q0
 
# Frequency (in Hz) and control period 
freq = 200
dt = 1.0/freq
rate = rospy.Rate(freq)
 
# Initial joint configuration
q = copy(q0)
x = copy(x0)

kp = 2
ko = 9
ka = 1e-2
 
# Main loop
#for i in range(1):
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Kinematic control law for the pose (complete here)
    # --------------------------------------------------
    # Calculo del error
    e = x-xd
    if np.linalg.norm(e) >= 1e-3:
        # Velocidad de articulaciones
        J = jacobian_pose(q)
        if(np.linalg.det(J.T @ J)<=1e-3):
            Jpinv = np.linalg.inv(J.T@J + (ka**2)*np.eye(6))@J.T
        else:
            Jpinv = np.linalg.pinv(J)
        de = np.hstack([-kp*e[0:3],-ko*e[3:7]])
        dq = Jpinv @ de
        # Aplicar velocidad
        nq = q + dt*dq
        if (nq[0]<=2*pi and nq[0]>=-2*pi): q[0]=nq[0]
        if (nq[1]<=pi   and nq[1]>=0): q[1]=nq[1]
        if (nq[2]<=0.27 and nq[2]>=0): q[2]=nq[2]
        if (nq[3]<=2*pi and nq[3]>=-2*pi): q[3]=nq[3]
        if (nq[4]<=pi/2 and nq[4]>=-pi/2): q[4]=nq[4]
        if (nq[5]<=2*pi and nq[5]>=-2*pi): q[5]=nq[5]

    T = fkine(q)
    x = TF2xyzquat(T)
    # -----------------------------
 
    # Log values                                                      
    fxcurrent.write(str(x[0])+' '+str(x[1]) +' '+str(x[2])+' '+str(x[3])+' '+str(x[4])+' '+str(x[5])+' '+str(x[6])+'\n')
    fxdesired.write(str(xd[0])+' '+str(xd[1]) +' '+str(xd[2])+' '+str(xd[3])+' '+str(xd[4])+' '+str(xd[5])+' '+str(xd[6])+'\n')
    fq.write(str(q[0])+" "+str(q[1])+" "+str(q[2])+" "+str(q[3])+" "+str(q[4])+" "+str(q[5])+"\n")
    # Publish the message
    jstate.position = q
    pub.publish(jstate)
    bmarker_desired.setPose(xd)
    bmarker_current.setPose(x)
    # Wait for the next iteration
    rate.sleep()


print('ending motion ...')
fxcurrent.close()
fxdesired.close()
fq.close()
