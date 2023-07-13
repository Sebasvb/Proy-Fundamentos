#!/usr/bin/env python3
import time
import roslib; #roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from functions import *
from std_msgs.msg import String



global press_key
press_key = "0"

def callback(msg):

  global press_key
  press_key = msg.data



if __name__ == '__main__':
    
  rospy.init_node("test1", disable_signals=True)

  rospy.Subscriber("/keys", String, callback)



  robot_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)

  print("Waiting for server...")
  robot_client.wait_for_server()
  print("Connected to server")

  # Joint names
  joint_names = ['waist_q1', 'shoulder_q2', 'slider_q3','elbow_q4', 'revolution_q5', 'wrist_q6']
  Q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  

  g = FollowJointTrajectoryGoal()
  g.trajectory = JointTrajectory()
  g.trajectory.joint_names = joint_names

  # Initial position
  g.trajectory.points = [ JointTrajectoryPoint(positions=Q0, velocities=[0]*6,time_from_start=rospy.Duration(2.0))]
  robot_client.send_goal(g)
  robot_client.wait_for_result()


  # Desired position
  xd = np.array([0.054, 0.125, 0.05449])
  # Initial configuration
  # q0 = Q0
  # Inverse kinematics
  q,ee = ikine(xd, Q0)
  print('holllaa')
  print(q)
  # Resulting position (end effector with respect to the base link)
  T = fkine(q)
  print('Obtained value:\n', np.round(T,3))


  rospy.sleep(1)
    
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():















    #Q0  = [2, 1.5707, 0, 0, 0, 0]
    #robot_client.cancel_goal()


    # ============== keys
    # Show the axes and buttons
    print('keys:', press_key)
    # =======================haciendo el IF sumar

    if (press_key  == '9'):
    
    # Desired position2
        xd[0] = xd[0]+0.05


        q,ee = ikine(xd, q)
        # Resulting position (end effector with respect to the base link)
        T = fkine(q)
        print('Obtained value:\n', np.round(T,3))
        
    elif (press_key  == '7'):

        # Desired position2
        xd[0] = xd[0]-0.05

        
        q,ee = ikine(xd, q)
        # Resulting position (end effector with respect to the base link)
        T = fkine(q)
        print('Obtained value:\n', np.round(T,3))

    
    elif (press_key  == '6'):

        # Desired position2
        xd[1] = xd[1]+0.05

        
        q,ee = ikine(xd, q)
        # Resulting position (end effector with respect to the base link)
        T = fkine(q)
        print('Obtained value:\n', np.round(T,3))
        
    elif (press_key  == '4'):
        # Desired position2
        xd[1] = xd[1]-0.05

        
        q,ee = ikine(xd, q)
        # Resulting position (end # ============== keys
    # Show the axes and buttons
    print('keys:', press_key)
    # =======================haciendo el IF sumar

    if (press_key  == '9'):
    
    # Desired position2
        xd[0] = xd[0]+0.05


        q,ee = ikine(xd, q)
        # Resulting position (end effector with respect to the base link)
        T = fkine(q)
        print('Obtained value:\n', np.round(T,3))
        
    elif (press_key  == '7'):

        # Desired position2
        xd[0] = xd[0]-0.05

        
        q,ee = ikine(xd, q)
        # Resulting position (end effector with respect to the base link)
        T = fkine(q)
        print('Obtained value:\n', np.round(T,3))

    
    elif (press_key  == '6'):

        # Desired position2
        xd[1] = xd[1]+0.05

        
        q,ee = ikine(xd, q)
        # Resulting position (end effector with respect to the base link)
        T = fkine(q)
        print('Obtained value:\n', np.round(T,3))
        
    elif (press_key  == '4'):
        # Desired position2
        xd[1] = xd[1]-0.05

        
        q,ee = ikine(xd, q)
        # Resulting position (end effector with respect to the base link)
        T = fkine(q)
        print('Obtained value:\n', np.round(T,3))
    
    elif (press_key  == '3'):
        # Desired position2
        xd[2] = xd[2]+0.05

        
        q,ee = ikine(xd, q)
        # Resulting position (end effector with respect to the base link)
        T = fkine(q)
        print('Obtained value:\n', np.round(T,3))

    elif (press_key  == '1'):

        # Desired position2
        xd[2] = xd[2]-0.05

        
        q,ee = ikine(xd, q)
        # Resulting position (end effector with respect to the base link)
        T = fkine(q)
        print('Obtained value:\n', np.round(T,3))
    # Modification of the motion

    g.trajectory.points = [ JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(0.01))]
    robot_client.send_goal(g)
    robot_client.wait_for_result()

    rate.sleep()

  robot_client.cancel_goal()