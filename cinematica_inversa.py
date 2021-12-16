#!/usr/bin/env python

#IN:
#posicion deseada x_des
#q0 inicial

#OUT:
#robot con efector final en posicion del x_des
#1 Marker en la posicion espacial del efector final
#1 Marker en la posicion espacial del x deseado

import rospy
from sensor_msgs.msg import JointState
from markers import *
from lab4functions import *


rospy.init_node("testInvKine")
#Publica los JointState en el topico joint_states
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['GREEN'])


# Joint names
#jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Desired position
xd = np.array([0.42, 0.125, 0.349])
# Initial configuration
q0 = np.array([0.0, -1.0, 1.7, -2.2, -1.6, 0.0])

# Inverse kinematics -> obtiene el valor de q necesario para llegar a la posicion deseada
#q = ikine_ur5(xd, q0)              #usando metodo de Newton
q = ik_gradient_ur5(xd, q0)       #usando metodo de la gradiente

# Se obtiene la nueva Matriz Homogenea en base al nuevo q -> (0-T-xdes)
# Resulting position (end effector with respect to the base link)
T = fkine_ur5(q)
print('Obtained value:\n', np.round(T,3))

# Red marker shows the achieved position
bmarker.xyz(T[0:3,3])
# Green marker shows the desired position
bmarker_des.xyz(xd)

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Asignar valores al mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q
print('q:\n', np.round(jstate.position,3))

# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():              #mientras que no se presiona Ctrl+C
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publicando el estado de las articulaciones del robot y las posiciones de las bolas roja y verde
    pub.publish(jstate)
    bmarker.publish()
    bmarker_des.publish()
    # Wait for the next iteration
    rate.sleep()
