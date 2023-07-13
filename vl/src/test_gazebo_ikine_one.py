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



  # -------------- Pelotita y robot --------------

  # Archivos donde se almacenara los datos
   
  fee = open("/home/user/project_ws/src/vl/src/erroracumuladoFinal2.txt", "w")
  fitera = open("/home/user/project_ws/src/vl/src/iteracumuladoFinal2.txt", "w")
  fnormal= open("/home/user/project_ws/src/vl/src/enormalcumuladoFinal2.txt", "w")


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


  ## Desired position
  xd = np.array([0.9,  -0.8,  0.249])

  # Almacenamiento del error
  #ee = []

    # Almacenamiento de iteraciones
  #itera = []



  # Initial configuration
  # q0 = Q0
  # Inverse kinematics

   # Almacenamiento del error
  ee = []
  eenormal = []
    # Almacenamiento de iteraciones
  itera = []



  q,ee,itera,eenormal= ik_gradient(xd, Q0)
  print('holllaa')
  print(q)
  # Resulting position (end effector with respect to the base link)
  T = fkine(q)
  print('Obtained value:\n', np.round(T,3))


  # Almacenamiento de datos
  fee.write(str(ee)+' '+'\n')
  fitera.write(str(itera)+' '+'\n')
  fnormal.write(str(eenormal)+' '+'\n')

  fee.close()
  fitera.close()
  fnormal.close()



  # Bucle de ejecucion continua
  t = 0.0


  # Almacenamiento del error
  #ee = []

    # Almacenamiento de iteraciones
  #itera = []



  rospy.sleep(1)

  
  #  Frecuencia del envio (en Hz)
  freq = 10
  dt = 1.0/freq
  rate = rospy.Rate(freq)
  while not rospy.is_shutdown():

    




   t=t+dt

  # Desired position  
   #1xd = np.array([0.054, 0.125, 0.1])
   #2xd = np.array([0.2, -0.085, 0.2])
   #q,ee, itera = ik_gradient(xd, q)

   robot_client.cancel_goal()

  
    # Modification of the motion
  
   g.trajectory.points = [ JointTrajectoryPoint(positions=q, velocities=[0]*6, time_from_start=rospy.Duration(0.01))]
   robot_client.send_goal(g)
   robot_client.wait_for_result()
   #fee.write(str(t)+' '+str(ee)+' '+'\n')
   rate.sleep()
  

  robot_client.cancel_goal()