#!/usr/bin/env python
#
 
from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState
 
from markers import *
from Funciones import *
 
from numpy import matrix, rank
 
# Initialize the node
rospy.init_node("testKineControlPose")
print('starting motion ... ')
# Publisher: publish to the joint_states topic
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
# Markers for the current and desired positions
bmarker_current  = FrameMarker()
bmarker_desired = FrameMarker(0.5)
 
# Joint names
#jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
 
#alcanzado_max_z = (89.2+425+392+94.75)/100
#alcanzado_max_xy = (425+392+94.75)/100
alcanzado_max = [alcanzado_max_xy, alcanzado_max_z]
 
# Desired pose
ang = pi/3
Rd = np.array([[0,1,0],[1,0,0],[0,0,-1]])
qd = rot2quat(Rd)
# Find an xd that the robot can reach
xd = np.array([0.4, 0.4, 0.4, qd[0], qd[1], qd[2], qd[3]])
if (xd[2] >= alcanzado_max[1]):
  xd[2]=alcanzado_max[1]
if (xd[1] >= alcanzado_max[0]):
  xd[1]=alcanzado_max[0]
if (xd[0] >= alcanzado_max[0]):
  xd[0]=alcanzado_max[0]
 
#xd  = np.array([0.5, 0.5, 0.6, np.cos(ang/2.0), 0, 0, np.sin(ang/2.0)])
# Initial configuration
q0  = np.array([0.0, -1.0, 1.7, -2.2, -1.6, 0.0])
 
# Resulting initial pose (end effector with respect to the base link)
T = fkine_ur5(q0)
x0 = TF2xyzquat(T)    #x0 = [x,y,z,w,ex,ey,ez]
 
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
xpos = x[0:3]
quat = x[3:7]
xpos_des = xd[0:3]
quat_des = xd[3:7]
 
# Initialize the derror vector (derivative of the error)
derror = np.zeros(7)
# Main loop
#for i in range(1):
while not rospy.is_shutdown():
  k = 1
  # Current time (needed for ROS)
  jstate.header.stamp = rospy.Time.now()
  # Kinematic control law for the pose (complete here)
  J = jacobian_pose(q, delta=0.0001)
  wd = qd[0]
  ed = qd[1:3]
  w = q[0]
  e = q[1:3]
 
  #Qe = [wd*w + ed.T*e, -w*e + w*ed - np.cross(ed,e)]
  Qe = np.hstack((wd*w + ed.T*e, -w*e + w*ed - np.cross(ed,e)))
 
  #eo = [Qe[0]-1, Qe[1:3]]
  eo = np.hstack((Qe[0]-1, Qe[1:3]))   
 
  #derror = [x-xd, eo]
  derror = np.hstack((xpos-xpos_des, eo))    
  print(derror.shape)
  e_der = -k*derror   
 
  n=np.linalg.matrix_rank(J); m=J.ndim               #np.linalg.det(J)==0
  if (n<m):
    q_der = np.dot(np.dot(np,linalg.inv(J.T*J),J.T), e_der)      #pseudo inversa amortiguada
  else:
    q_der = np.dot(np.linalg.pinv(J), e_der)         
 
  q = q + dt*q_der
    
  # --------------------------------------------------
 
 
  # Current configuration trnaformation to current position
  T = fkine_ur5(q)
  x = TF2xyzquat(T)
 
  min_err=0.1
  if(np.linalg.norm(xd-x0)<min_err):
    break
  pass
 
  # Publish the message
  jstate.position = q
  pub.publish(jstate)
  bmarker_desired.setPose(xd)
  bmarker_current.setPose(x)
    
 
  # Wait for the next iteration
  rate.sleep()
