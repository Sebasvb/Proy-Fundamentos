from copy import copy
import numpy as np
# import rbdl

pi = np.pi

# ----------------------------------
# Matriz de transformacion homogenea

def dh(d, theta, a, alpha):
    """
    Matriz de transformacion homogenea asociada a los parametros DH.
    Retorna una matriz 4x4
    """

    sth = np.sin(theta)
    cth = np.cos(theta)
    sa = np.sin(alpha)
    ca = np.cos(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                  [sth,  ca*cth, -sa*cth, a*sth],
                  [0.0,      sa,      ca,     d],
                  [0.0,     0.0,     0.0,   1.0]])
    return T

# ----------------------------------
# Cinematica directa


def fkine(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """

    # Matrices DH
    T1 = dh(  0.1131338,  q[0]+pi/2,  0, pi/2)
    T2 = dh(    0,  q[1]+pi/2,         0, pi/2)
    T3 = dh(    0.4+q[2],       0,         0, pi/2)
    T4 = dh(    0.086,       q[3]+pi/2,         0.1535, pi/2)
    T5 = dh(0,          q[4]+pi/2,         0,   pi/2)
    T6 = dh(    0.2235,       q[5],         0,    0)

    # Efector final con respecto a la base
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
    return T

# ----------------------------------
# Jacobiano de posicion


def jacobian_position(q):
    delta = 0.0001

    # Crear una matriz 3x6
    J = np.zeros((3, 6))
    # Transformacion homogenea inicial (usando q)
    To = fkine(q)
    To = To[0:3, -1:]  # vector posicion

    # Iteracion para la derivada de cada columna
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i]+delta
        # Transformacion homogenea luego del incremento (q+delta)
        T = fkine(dq)
        T = T[0:3, -1:]  # vector posicion
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        Jq = 1/delta*(T-To)
        J[:, i:i+1] = Jq

    return J

# ----------------------------------
# Cinematica inversa


def ikine(xdes, q0):
    # Error
    epsilon = 0.001
    # Maximas iteraciones
    max_iter = 1000
    # Delta de la jacobiana
    delta = 0.00001
    # Copia de las articulaciones
    q = copy(q0)
    # Almacenamiento del error
    ee = []
    # Transformacion homogenea (usando q)
    To = fkine(q)
    To = To[0:3, 3]  # vector posicion
    # Resetear cuando se llega a la cantidad maxima de iteraciones
    restart = True

    while restart:
        for i in range(max_iter):
            if(q[4]<0):
                q[4]=0
            # Hacer el for 1 vez
            restart = False
            # Pseudo-inversa del jacobiano
            J = jacobian_position(q)
            J = np.linalg.pinv(J)
            # Error entre el x deseado y x actual
            e = xdes - To
            # q_k+1
            q = q + np.dot(J, e)
            # Nueva mtransformada homogenea
            To = fkine(q)
            To = To[0:3, 3]  # vector posicion

            # Norma del error
            enorm = np.linalg.norm(e)
            ee.append(enorm)    # Almacena los errores
            # Condicion de termino
            if (enorm < epsilon):
                print("Error en la iteracion ", i, ": ", np.round(enorm, 4))
                break
            if (i == max_iter-1 and enorm > epsilon):
                print("Iteracion se repite")
                print("Error en la iteracion ", i, ": ", enorm)
            restart = True
    return q,ee 


# -----------------------------------


#============ gradiente
def ik_gradient(xdes, q0):
    """
    Calcular la cinematica inversa
    Emplear el metodo gradiente
    """
    epsilon  = 0.001 # Máximo error admisible (condición para el término)
    max_iter = 1000 # Máximo número de iteraciones
    delta    = 0.5
    # Almacenamiento del error
    ee = []
    eenormal = []

    # Almacenamiento de iteraciones
    itera = []




    q  = copy(q0)
    for i in range(max_iter):
        itera.append(i)

        # Main loop
        # Jacobiano del robot
        J = jacobian_position(q)
        JT =J.T

        # Cinemática directa del robot

        TFF = fkine(q)
        f = TFF[0:3,3]
        # Error
        e = xdes-f


        enorm = np.linalg.norm(e)



        eenormal.append(enorm)
        ee.append(e)
        # # Actualización de q (método de gradiente)
        q = q + delta*np.dot(J.T, e)
        # q = q + delta*np.dot(JT,e)

        # Condición de término
        # if (np.linalg.norm(e)<epsilon):
        #     break


        pass
    
    return q,ee,itera,eenormal



# -------------------------------

# Jacobiano de posicion y orientacion como cuaternion
def jacobian_pose(q, delta=1e-4):
    """
    Jacobiano analitico para la posicion y orientacion (usando un
    cuaternion). Retorna una matriz de 7x6 y toma como entrada el vector de
    configuracion articular q=[q1, q2, q3, q4, q5, q6]
    
    """
 
    J = np.zeros((7,6))
    # Implementar este Jacobiano aqui
    To = fkine(q)
    x0 = TF2xyzquat(To)
 
    # Iteracion para la derivada de cada columna
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i]+delta
        # Transformacion homogenea luego del incremento (q+delta)
        T = fkine(dq)
        x = TF2xyzquat(T)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        Jq = 1/delta*(x-x0)
 
        J[0, i:i+1] = Jq[0]
        J[1, i:i+1] = Jq[1]
        J[2, i:i+1] = Jq[2]
        J[3, i:i+1] = Jq[3]
        J[4, i:i+1] = Jq[4]
        J[5, i:i+1] = Jq[5]
        J[6, i:i+1] = Jq[6]
 
    return J

# Conversion de matriz de rotacion a cuaternion
def rot2quat(R):
    if(1+R[0,0]+R[1,1]+R[2,2]>0):
        w = 1/2*np.sqrt(1+R[0,0]+R[1,1]+R[2,2])
        ex = 1/(4*w)*(R[2,1]-R[1,2])
        ey = 1/(4*w)*(R[0,2]-R[2,0])
        ez = 1/(4*w)*(R[1,0]-R[0,1])
    elif(R[0,0]>=R[1,1] and R[0,0]>=R[2,2]):
            ex = 1/2*np.sqrt(1+R[0,0]-R[1,1]-R[2,2])
            w = 1/(4*ex)*(R[2,1]-R[1,2])
            ey = 1/(4*ex)*(R[1,0]+R[0,1])
            ez = 1/(4*ex)*(R[0,2]+R[2,0])
    elif(R[1,1]>=R[0,0] and R[1,1]>=R[2,2]):
            ey = 1/2*np.sqrt(1+R[1,1]-R[0,0]-R[2,2])
            w = 1/(4*ey)*(R[0,2]-R[2,0])
            ex = 1/(4*ey)*(R[1,0]+R[0,1])
            ez = 1/(4*ey)*(R[1,2]+R[2,1])
    elif(R[2,2]>=R[0,0] and R[2,2]>=R[1,1]):
            ez = 1/2*np.sqrt(1+R[2,2]-R[1,1]-R[2,2])
            w = 1/(4*ez)*(R[1,0]-R[0,1])
            ex = 1/(4*ez)*(R[2,0]+R[0,2])
            ey = 1/(4*ez)*(R[1,2]+R[2,1])
    return np.array([w,ex,ey,ez])

# Conversion de matriz T a vector con possicion y cuaternion
def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)

# Conversion de matriz antrisimetrica a de rotacion
def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R
 
# Matriz de rotacion R a partir de Q
def Q_R(Q):
    w = Q[0]; ex = Q[1]; ey = Q[2]; ez = Q[3]
    R = np.array([
        [2*(w**2+ex**2)-1,   2*(ex*ey-w*ez),    2*(ex*ez+w*ey)],
        [  2*(ex*ey+w*ez), 2*(w**2+ey**2)-1,    2*(ey*ez-w*ex)],
        [  2*(ex*ez-w*ey),   2*(ey*ez+w*ex),    2*(w**2+ez**2)-1]
    ])
    return R
