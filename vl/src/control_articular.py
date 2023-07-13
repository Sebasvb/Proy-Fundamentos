from roslib import packages
from functions import *
from markers import *
from sensor_msgs.msg import JointState
import rospy
import rbdl
import numpy as np

rospy.init_node("control_articular_dininv")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])

# Archivos donde se almacenara los datos
fqact = open("Data/qactual_cart3.txt", "w")
fqdes = open("Data/qdeseado_cart3.txt", "w")
fxact = open("Data/xactual_cart3.txt", "w")
fxdes = open("Data/xdeseado_cart3.txt", "w")

# Nombres de las articulaciones
jnames = ['waist_q1', 'shoulder_q2', 'slider_q3',
          'elbow_q4', 'revolution_q5', 'wrist_q6']
# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# =============================================================
# Configuracion articular inicial (en radianes)
# q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# q = np.array([0.0, 1.57, 0.2, -1.57, 1.0, 0.1])
q = np.array([0, 0.57, 0.2, 0.0, 1.57, 1.57])
# Velocidad inicial
dq = np.array([0., 0., 0., 0., 0., 0.])
# Aceleracion inicial
ddq = np.array([0., 0., 0., 0., 0., 0.])
# Configuracion articular deseada
# qdes = np.array([0.0, 1.57, 0.2, -1.57, 1.0, 0.1])
# qdes = np.array([0, 0.57, 0.2, 0.0, 1.57, 1.57])
qdes = np.array([1.57, 0., 0.25, -1.57, 0, 1.])
# Velocidad articular deseada
dqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# Aceleracion articular deseada
ddqdes = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# =============================================================

# Posicion resultante de la configuracion articular deseada
xdes = fkine(qdes)[0:3, 3]
# Posicion resultante de la configuracion articular inicial
x = fkine(q)[0:3, 3]
# Red marker shows the achieved position
bmarker_actual.xyz(x)
# Green marker shows the desired position
bmarker_deseado.xyz(xdes)
# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel('../urdf/VL.urdf')
ndof = modelo.q_size       # Grados de libertad
temp = np.zeros(ndof)
zeros = np.zeros(ndof)
g = np.zeros(ndof)         # Para la gravedad
c = np.zeros(ndof)
M = np.zeros([ndof, ndof])

# Frecuencia del envio (en Hz)
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)
epsilon = 1e-3

# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)

g = np.zeros(ndof)
zeros = np.zeros(ndof)       # Vector de ceros

# Se definen las ganancias del controlador
valores = 20*np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
Kp = np.diag(valores)
Kd = 2*np.sqrt(Kp)

# Bucle de ejecucion continua
t = 0.0

while not rospy.is_shutdown():

    # Leer valores del simulador
    q = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Posicion actual del efector final
    x = fkine(q)[0:3, 3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()

    # Almacenamiento de datos
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' ' +
                str(q[2])+' ' + str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' ' + str(
        qdes[2])+' ' + str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+'\n ')

    # ----------------------------
    # Control dinamico
    # ----------------------------

    e = qdes - q
    # Matriz G
    rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
    # Matriz M
    rbdl.CompositeRigidBodyAlgorithm(modelo, q, M)
    # Matriz C
    rbdl.InverseDynamics(modelo, q, dq, zeros, temp)
    c = temp - g

    y = ddqdes + (Kd.dot(dqdes - dq)) + (Kp.dot(qdes - q))

    # Ley de control
    u = M@y + c + g

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
    if (np.linalg.norm(e) < epsilon):
        print('xdes: ', np.round(xdes, 2))
        print('xactual: ', np.round(x, 2))
        print('qdes: ', qdes)
        print('qactual: ', np.round(q, 2))
        print('Se llego al punto deseado.')
        break

print('Fin del movimiento.')
fqact.close()
fqdes.close()
fxact.close()
fxdes.close()
