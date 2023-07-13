#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

# Cargar los datos de los archivos
fxcurrent = np.loadtxt("/tmp/x.txt")                
fxdesired = np.loadtxt("/tmp/xd.txt")
fq = np.loadtxt("/tmp/q.txt")
x = fxcurrent[:,0]
y = fxcurrent[:,1]
z = fxcurrent[:,2]

xd = fxdesired[:,0]
yd = fxdesired[:,1]
zd = fxdesired[:,2]

q1 = fq[:,0]
q2 = fq[:,1]
q3 = fq[:,2]
q4 = fq[:,3]
q5 = fq[:,4]
q6 = fq[:,5]

dt = 1/200
t = np.linspace(0,(len(x)-1)*dt,len(x))

plt.subplot(3,3,1)
plt.plot(t,x,label='x')
plt.plot(t,xd,label='xd')
plt.xlabel('Tiempo (s)')
plt.ylabel('x (m)')
plt.title('Posición en x')
plt.legend()

plt.subplot(3,3,2)
plt.plot(t,y,label='y')
plt.plot(t,yd,label='yd')
plt.xlabel('Tiempo (s)')
plt.ylabel('y (m)')
plt.title('Posición en y')
plt.legend()

plt.subplot(3,3,3)
plt.plot(t,z,label='z')
plt.plot(t,zd,label='zd')
plt.xlabel('Tiempo (s)')
plt.ylabel('z (m)')
plt.title('Posición en z')
plt.legend()

plt.subplot(3,3,4)
plt.plot(t,q1,label='q_1')
plt.xlabel('Tiempo (s)')
plt.ylabel('q_1 (rad)')
plt.title('Posición de q_1')
plt.legend()

plt.subplot(3,3,5)
plt.plot(t,q2,label='q_2')
plt.xlabel('Tiempo (s)')
plt.ylabel('q_2 (rad)')
plt.title('Posición de q_2')
plt.legend()

plt.subplot(3,3,6)
plt.plot(t,q3,label='q_3')
plt.xlabel('Tiempo (s)')
plt.ylabel('q_3 (rad)')
plt.title('Posición de q_3')
plt.legend()

plt.subplot(3,3,7)
plt.plot(t,q4,label='q_4')
plt.xlabel('Tiempo (s)')
plt.ylabel('q_4 (rad)')
plt.title('Posición de q_4')
plt.legend()

plt.subplot(3,3,8)
plt.plot(t,q5,label='q_5')
plt.xlabel('Tiempo (s)')
plt.ylabel('q_5 (rad)')
plt.title('Posición de q_5')
plt.legend()

plt.subplot(3,3,9)
plt.plot(t,q6,label='q_6')
plt.xlabel('Tiempo (s)')
plt.ylabel('q_6 (rad)')
plt.title('Posición de q_6')
plt.legend()


plt.tight_layout()
plt.show()
