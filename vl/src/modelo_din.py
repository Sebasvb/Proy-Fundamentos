from roslib import packages
from functions import *
from markers import *
from sensor_msgs.msg import JointState
import rospy
import rbdl
import numpy as np

# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('../urdf/VL.urdf')

# Grados de libertad
ndof = modelo.q_size

# Configuracion articular
q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5])

# Inicialización de arrays numpy
zeros = np.zeros(ndof)     # Vector de ceros
tau = np.zeros(ndof)       # Para torque
tau2 = np.zeros(ndof)      # Para torque2
m_row = np.zeros(ndof)     # Para la matriz de masa
g = np.zeros(ndof)         # Para la gravedad
c = np.zeros(ndof)         # Para el vector de Coriolis + centrifuga
M = np.zeros([ndof, ndof])  # Para la matriz de inercia
e = np.eye(6)              # Vector identidad

# Torque dada la configuracion del robot
# rbdl.InverseDynamics(modelo, q, dq, ddq, tau)

# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics
print('InverseDynamics')

# vector g
rbdl.InverseDynamics(modelo, q, zeros, zeros, g)

# vector c
rbdl.InverseDynamics(modelo, q, dq, zeros, c)
c = c - g

# matriz M
for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i, :], m_row)
    M[i, :] = m_row - g

print('\n vector g: \n', np.round(g, 2))
print('\n vector c: \n', np.round(c, 2))
print('\n matriz M: \n', np.round(M, 2))

# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
# en los arreglos llamados M2 y b2
b2 = np.zeros(ndof)         # Para efectos no lineales
M2 = np.zeros([ndof, ndof])  # Para matriz de inercia

print('Metodo 2')
# Confirmacion de M
rbdl.CompositeRigidBodyAlgorithm(modelo, q, M2, 1)
print('\n matriz M: \n', np.round(M2, 2))

# confirmacion de b
rbdl.NonlinearEffects(modelo, q, dq, b2)
print('\n matriz b: \n', np.round(b2, 2))

# Parte 3: Verificacion de valores
print('Diferencia de M y M2 \n', np.round(M-M2, 2))

c2 = np.zeros(ndof)         # Para el vector de Coriolis+centrifuga
rbdl.InverseDynamics(modelo, q, dq, zeros, c2)
print('Diferencia de b2 y c2 \n', np.round(b2-c2, 2))

# Parte 4: Verificacion de la expresion de la dinamica

# Tau con ID
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)

# Tau con CRBA y NLE
tau2 = M2.dot(ddq)+b2

print('Tau mediante ID: \n', np.round(tau, 2))
print('Tau mediante CRBA y NLE: \n', np.round(tau2, 2))

print('Diferencia: \n', np.round(tau - tau2, 2))  # !/usr/bin/env python
