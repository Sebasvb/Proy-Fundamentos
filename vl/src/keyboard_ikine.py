#!/usr/bin/env python3
 
import rospy
 
# Publisher
from sensor_msgs.msg import JointState
from markers import *
from functions import *
 
# Subscriber
from std_msgs.msg import String
global press_key
press_key = "0"
def callback(msg):
    global press_key
    press_key = msg.data
 
# Main
if __name__ == '__main__':
    # Nodo
    rospy.init_node("keyboard_ikine")
    # Publisher
    pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
    bmarker = BallMarker(color['RED'])



    bmarker      = BallMarker(color['GREEN'])
    #bmarker_des  = BallMarker(color['GREEN'])



    # Subscriber
    rospy.Subscriber("/keys", String, callback)
 
    # -------------- Pelotita y robot --------------

    # Archivos donde se almacenara los datos
   
    fqdes = open("/tmp/qdeseado.txt", "w")
 
    fxdes = open("/tmp/xdeseado.txt", "w")



    # Joint names
    jnames = ['waist_q1', 'shoulder_q2', 'slider_q3','elbow_q4', 'revolution_q5', 'wrist_q6']
    # Joint Configuration
    q0 = np.array([0, 0, 0, 0, 0, 0]) # CI  
    q = q0
 
    # End effector with respect to the base
    T = fkine(q0)
    print('Obtained value:\n', np.round(T, 3) )
    bmarker.position(T)
 
    xdes = T[0:3,3]
 
    deltax = [0, 0, 0]
    l_max = 0.27
 
    # Object (message) whose type is JointState
    jstate = JointState()
    # Set values to the message
    jstate.header.stamp = rospy.Time.now()
    jstate.name = jnames
    # Add the head joint value (with value 0) to the joints
    jstate.position = q0
 
 
    # Loop rate (in Hz)
    rate = rospy.Rate(10)
    t = 1/10
    # Continuous execution loop
    while not rospy.is_shutdown():



         # Almacenamiento de datos
        fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
        fqdes.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
 



        # Muestra tecla
 
        # x+
        if (press_key == 'w'):
            print ('Input:', press_key)
            xdes[0] = xdes[0]  +0.02
            q,ee = ikine(xdes,q0)
            q0 = q
                
        # x-
        elif (press_key == 's'):
            print ('Input:', press_key)
            xdes[0] = xdes[0] -0.02
            q,ee = ikine(xdes,q0)
            q0 = q
 
        # y+
        elif (press_key == 'a'):
            if (q[2] < l_max):
                print ('Input:', press_key)
                xdes[1] = xdes[1] +0.02
                q,ee = ikine(xdes,q0)
                q0 = q
            else:
                q0[2] = l_max
                q,ee = ikine(xdes,q0)
                q0 = q
        # y-
        elif (press_key == 'd'):
            if (q[2] >= 0.0):
                print ('Input:', press_key)
                xdes[1] = xdes[1] -0.02
            else:
                q0[2] = 0.0
                q,ee = ikine(xdes,q0)
                q0 = q
        # z+
        elif (press_key == 'r'):
            print ('Input:', press_key)
            xdes[2] = xdes[2]+0.02
            q,ee = ikine(xdes,q0)
            q0 = q
 
        # z-
        elif (press_key == 'f'):
            print ('Input:', press_key)
            xdes[2] = xdes[2]-0.02
            q,ee = ikine(xdes,q0)
            q0 = q
 
 
     
        print(q)

        if (q[2] >= l_max):
            q0[2] = l_max
        
        if (q[2] <= 0.0):
            q0[2] = 0.0



            

     




 
        # Current time (needed for ROS)
        jstate.header.stamp = rospy.Time.now()
        # Add the head joint value (with value 0) to the joints
        jstate.position = q
        # Publish the message
        pub.publish(jstate)
        bmarker.position(T)
        bmarker.publish()
 
        # Wait for the next iteration
        rate.sleep()
    fqdes.close()
    fxdes.close()