#!/usr/bin/env python

#IN:
#config q_deseado

#OUT:
#robot con las articulaciones indicadas
#1 Marker en la posicion espacial del efector final


import rospy
from sensor_msgs.msg import JointState

from markers import *
from Funciones import *

rospy.init_node("testForwardKinematics")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker = BallMarker(color['GREEN'])

# Joint names
#jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Joint Configuration (configuracion articular deseada)
#q = [0.8, 0.1, 0.8, 0, 0, 0]

# End effector with respect to the base
T = fkine_ur5(q)
print( np.round(T, 3) )
bmarker.position(T)   #Set position (4x4 NumPy homogeneous matrix) for the ball and publish it

# Object (message) whose type is JointState
jstate = JointState()
# Set values to the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    # Wait for the next iteration
    rate.sleep()
