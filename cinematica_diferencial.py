#!/usr/bin/env python
#

#IN:
#posición cartesiana deseada (xd)
#pos articular inicial (q0)

#OUT:
#Movimiento hacia la posicion deseada (se guardaran datos de xactual xd, error)

from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState
 
from markers import *
from funciones import *
 
from numpy import matrix, rank
 
# Initialize the node
rospy.init_node("testKineControlPosition")
print('starting motion ... ')
# Publisher: publish to the joint_states topic
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
# Files for the logs
fxcurrent = open("/tmp/xcurrent.txt", "w")                
fxdesired = open("/tmp/xdesired.txt", "w")
fq = open("/tmp/q.txt", "w")
 
# Markers for the current and desired positions
bmarker_current  = BallMarker(color['RED'])
bmarker_desired = BallMarker(color['GREEN'])
 
# Joint names
#jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
 
# Desired position
#xd = np.array([-0.8, -0.2, 0.0])
# Initial configuration
#q0  = np.array([1.0, 0.0, 1.0, 3.0, 2.0, 0.5])
 
# Resulting initial position (end effector with respect to the base link)
T = fkine_ur5(q0)
x0 = T[0:3,3]      #initial position(x,y,z)
 
# Red marker shows the achieved position
bmarker_current.xyz(x0)
# Green marker shows the desired position
bmarker_desired.xyz(xd)
 
# Instance of the JointState message
jstate = JointState()
# Values of the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q0
 
# Frequency (in Hz) and control period 
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)
 
# Initial joint configuration
q = copy(q0)
# Main loop
while not rospy.is_shutdown():
  
  k = 100
  # Current time (needed for ROS)
  jstate.header.stamp = rospy.Time.now()
  # Kinematic control law for position (complete here)
  # -----------------------------
  #n=np.linalg.matrix_rank(J); m=J.ndim 
  J = jacobian_ur5(q0, delta=0.0001)
  e = x0 - xd
  e_der = -k*e
  
  n=np.linalg.matrix_rank(J); m=J.ndim               #np.linalg.det(J)==0
  ##############################
  if (n<m):
    q_der = np.dot(np.dot(np,linalg.inv(J.T*J),J.T), e_der) #pseudo inversa amortiguada
  else:
    q_der = np.dot(np.linalg.pinv(J), e_der)
 
  #q_der = np.dot(np.linalg.pinv(J), e_der)
  q = q + dt*q_der
  # -----------------------------
  
  T=fkine_ur5(q)   
  x=T[0:3,3]
  x0=x
  # Log values                                                      
  fxcurrent.write(str(x[0])+' '+str(x[1]) +' '+str(x[2])+'\n')
  fxdesired.write(str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
  fq.write(str(q[0])+" "+str(q[1])+" "+str(q[2])+" "+str(q[3])+" "+
            str(q[4])+" "+str(q[5])+"\n")
    
 
  min_err=0.01
  if(np.linalg.norm(e)<min_err):
    break
 
 
  # Publish the message
  jstate.position = q
  pub.publish(jstate)
  bmarker_desired.xyz(xd)
  bmarker_current.xyz(x)
 
  # Wait for the next iteration
  rate.sleep()
 
print('ending motion ...')
fxcurrent.close()
fxdesired.close()
fq.close()
