import numpy as np
from copy import copy
import rbdl


pi = np.pi
cos=np.cos; sin=np.sin; pi=np.pi


def dh(d, theta, a, alpha):

  cth=cos(theta)
  sth=sin(theta)
  ca=cos(alpha)
  sa=sin(alpha)
  T = np.array([[cth, -ca*sth, sa*sth, a*cth],
                [sth, ca*cth, -sa*cth, a*sth],
                [0, sa, ca, d],
                [0,0,0,1]])
 
  return T
    
    

def fkine_ur5(q):
 
  # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
  T1 = dh( q[0], 0, 0, pi/2)
  T2 = dh( 0.550, q[1], 0, pi/2)
  T3 = dh( 0.6344, q[2]-pi/2, -1, 0)
  T4 = dh( -0.4249, q[3], -1, 0)
  T5 = dh( -0.4445, q[4]-pi/2, 0, pi/2)
  T6 = dh( 0.950, q[5]+pi/2, 0, 0)
  # Efector final con respecto a la base
  T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
  return T


Obtención del Jacobiano (se recibe el q actual y la variación que se le hará)
def jacobian_ur5(q, delta=0.0001):
  """
  Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
  entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
  """
  # Crear una matriz 3x6
  J = np.zeros((3,6))
  # Transformacion homogenea (0-T-6) usando el q dado
  T = fkine_ur5(q)
  # Iteracion para la derivada de cada columna
  for i in xrange(6):
    # Copiar la configuracion articular(q) y almacenarla en dq
    dq = copy(q)
    # Incrementar los valores de cada q sumandoles un delta a cada uno
    dq[i] = dq[i] + delta
    # Obtención de la nueva Matriz Homogenea con los nuevos valores articulares, luego del incremento (q+delta)
    Td = fkine_ur5(dq)
    # Aproximacion del Jacobiano de posicion usando diferencias finitas
    for j in xrange(3):
      J[j][i] = (Td[j][3]-T[j][3])/delta
  return J
  
  
  #Método de Newton, se recibe la posición a la que se desea ir(xdes) y el valor actual del q
#responde a: ¿cuáles son los valores articulares(q) que debe tener el robot para estar en la posición deseada?
def ikine_ur5(xdes, q0):
  """
  Calcular la cinematica inversa de UR5 numericamente a partir de la 
  configuracion articular inicial de q0. 
  Emplear el metodo de newton
  """
  epsilon  = 0.001      #error(distancia) mínima requerida
  max_iter = 1000       #max cantidad de iteraciones
  delta    = 0.00001    #variación(movimiento) en el espacio
 
  q  = copy(q0)  #se copia el valor de la articulación inicial(q0) y se almacena en q
  for i in range(max_iter):
    J=jacobian_ur5(q,delta)   # Matriz Jacobiana
    Td=fkine_ur5(q)           # Matriz Actual
    #Posicion Actual: extrayendo la parte traslacional de la Matriz Homogenea(d1,d2,d3) y almacenandolos en el vector xact
    xact=Td[0:3,3]
 
    # Error entre pos deseada y pos actual (cuanta distancia separa ambos puntos)
    e=xdes-xact   #difencia de ambos, dado que son 2 vectores con mismo origen, la resta significa un vector que va desde xact a xdes
                  #e=var(x,y,z) -> variación espacial para llegar a pos deseada
                  
    # Metodo de Newton (se actualiza el valor de q y vuelve al loop si tdv no llega a la max iteración)
    q=q+np.dot(np.linalg.pinv(J),e)   #q=q+var(q) -> q=q+(inv(J)*var(x,y,z)) -> q=q+(inv(J)*e)
    #Condicion de termino
    if(np.linalg.norm(e)<epsilon):  #norma=modulo, el modulo es la distancia en magnitud faltante para llegar a la posición deseada
        break
    pass
  return q
  
  
  # Lo mismo que se realizó anteriormente, solo cambiando la línea de método de Newton por el del Metodo de la Gradiente
#Este método necesita además del parámetro delta
def ik_gradient_ur5(xdes, q0):
  """
  Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
  Emplear el metodo gradiente
  """
  epsilon  = 0.001
  max_iter = 1000
  delta    = 0.00001
  alpha    = 0.5
 
  q  = copy(q0)
  for i in range(max_iter):
    # Main loop
    #Matriz Jacobiana
    J=jacobian_ur5(q,delta)
    #Matriz Actual
    Td=fkine_ur5(q)
    #Posicion Actual
    xact=Td[0:3,3]
    # Error entre pos deseada y pos actual
    e=xdes-xact
 
    # Metodo de la gradiente
    q=q+alpha*np.dot(J.T,e)
    #Condicion de termino
    if(np.linalg.norm(e)<epsilon):
        break
    pass
  return q
  
  
  
class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/ur5_robot.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq
