#!/usr/bin/env python

#IN:
#pos articular inicial (q0)
#velocidad articular inicial (dq)
#pos articular deseada (qdes)

#OUT:
#desplazamiento del robot hacia el punto indicado, asi como la aparicion de los markers, uno en el lugar deseado y otro en el efector final del robot

# Configuracion articular inicial (en radianes)
#q = np.array([0.0, -1.0, 1.7, -2.2, -1.6, 0.0])
# Velocidad inicial
#dq = np.array([0., 0., 0., 0., 0., 0.])
# Configuracion articular deseada
#qdes = np.array([1.0, -1.0, 1.0, 1.3, -1.5, 1.0])


import rospy
from sensor_msgs.msg import JointState
from markers import *
from funciones import *
from roslib import packages

import rbdl


rospy.init_node("control_pdg")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual  = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])
# Archivos donde se almacenara los datos
#fqact = open("/tmp/qactual.dat", "w")
#fqdes = open("/tmp/qdeseado.dat", "w")
#fxact = open("/tmp/xactual.dat", "w")
#fxdes = open("/tmp/xdeseado.dat", "w")

# Nombres de las articulaciones
#jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
#      	'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# =============================================================
# Configuracion articular inicial (en radianes)
#q = np.array([0.0, -1.0, 1.7, -2.2, -1.6, 0.0])
# Velocidad inicial
#dq = np.array([0., 0., 0., 0., 0., 0.])
# Configuracion articular deseada
#qdes = np.array([1.0, -1.0, 1.0, 1.3, -1.5, 1.0])
# =============================================================

# Posicion resultante de la configuracion articular deseada
xdes = fkine_ur5(qdes)[0:3,3]

# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)

# Modelo del robot
#modelo = rbdl.loadModel('../urdf/ur5_robot.urdf')
#ndof   = modelo.q_size 	# Grados de libertad
ndof = 6

# Frecuencia del envio (en Hz)
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)

# Se definen las ganancias del controlador
valores = 0.1*np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
Kp = np.diag(valores)
Kd = 2*np.sqrt(Kp)

# Bucle de ejecucion continua
t = 0.0


# Arrays numpy
zeros = np.zeros(ndof)      	# Vector de ceros
tau   = np.zeros(ndof)      	# Para torque
g 	= np.zeros(ndof)      	# Para la gravedad
#c 	= np.zeros(ndof)      	# Para el vector de Coriolis+centrifuga
#M 	= np.zeros([ndof, ndof])  # Para la matriz de inercia
e 	= np.eye(6)           	# Vector identidad



while not rospy.is_shutdown():

	# Leer valores del simulador
	q  = robot.read_joint_positions()
	dq = robot.read_joint_velocities()
	# Posicion actual del efector final
	x = ur5_fkine(q)[0:3,3]
	# Tiempo actual (necesario como indicador para ROS)
	jstate.header.stamp = rospy.Time.now()

	# Almacenamiento de datos
	#fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
	#fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
	#fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
	#fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+'\n ')

	# ----------------------------
	# Control dinamico (COMPLETAR)
	# ----------------------------
	u = np.zeros(ndof)   # Reemplazar por la ley de control

	#rbdl.InverseDynamics(modelo,q,zeros,zeros,g)
	#Hallando matriz g:
	#Jv1=
	#Jv2=
	#Jv11 = Jv1[0:3, 0]
	#Jv12 = Jv1[0:3, 1]

	#Jv21 = Jv2[0:3, 0]
	#Jv22 = Jv2[0:3, 1]
	#m1g0 = sp.Matrix([[0],
#			[0],
#			[-m1*g0]])
	#m2g0 = sp.Matrix([[0],
	#		[0],
	#		[-m2*g0]])
	#g1 = -Jv11.T*m1g0 - Jv21.T*m2g0
	#g2 = -Jv12.T*m1g0 - Jv22.T*m2g0
	#g = sp.Matrix([[g1],
	#		[g2]])
	#print("Matriz g: "); display(sp.simplify(g))
	
	error_q = qdes - q
	error_dq = 0 - dq

	torque = Kp.dot(error_q) + Kd.dot(error_dq)
	#torque = Kp.dot(error_q) + Kd.dot(error_dq) + g
	u = torque
	print(u)

	# Simulacion del robot
	robot.send_command(u)

	# Publicacion del mensaje
	jstate.position = q
	pub.publish(jstate)
	bmarker_deseado.xyz(xdes)
	bmarker_actual.xyz(x)
	t = t+dt
	# Esperar hasta la siguiente  iteracion
	rate.sleep()

#fqact.close()
#fqdes.close()
#fxact.close()
#fxdes.close()