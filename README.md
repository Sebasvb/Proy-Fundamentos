# Proy-Fundamentos


Dentro de la subcarpeta de src se encuentra los códigos para la implementación del proyecto de fundamentos, los cuales son desde los componentes y el modelo del robot, pasando también por la aplicación de cinemática directa e inversa del robot, control cinemático del robot, y dinámica y control dinámico del robot. As mismo cuenta con los componentes para visualizar el robot en Rviz he simularlo en Gazebo.


Visualización del robot en Rviz:
roslaunch vl display.launch
rosrun vl "nombre de ejecutable"

Simulación del robot en Gazebo:
roslaunch vl gazebo.launch
rosrun vl "nombre de ejecutable" __ns:=arm_controller

--------------------------------------------------Cinemática directa e inversa----------------------------

Para la cinemática directa se utilizó los parámetros de Denavit–Hartenberg.


Para la cinemática inversa se utilizó un método numérico, llamado descenso de gradiente, para el cálculo de la cinemática inversa. 
Se implementó en dos versiones:

Asignación de un punto en el espacio operacional, el cual se observa en el código en Python llamado "test_gazebo_ikine.py"



Asignación de una serie de puntos atreves del teclado en el espacio operacional, el cual se observa en el código en Python llamado "test_gazebo_ikine_one.py"



La validez de la convergencia de los puntos en el espacio operacional se verificó mediante gráficas, donde se plotea la cantidad de iteraciones de la optimización numérica vs el error normalizado, el cual toma en cuenta el error en cada eje cartesiano.


--------------------------------------------------Control Cinemático---------------------------------------


Para el control cinemático se utilizó un método numérico, llamado diferencias finitas, para el control cinemático del jacobiano. 
Se implementó en:

Asignación de un punto en el espacio operacional, el cual se observa en el código en Python llamado "test_diffkine_pose.py"


Además, para la visualización de los resultados se usó un graficador, el cual se observa en el código en Python llamado "plot.py"


--------------------------------------------------Dinámica y Control Dinámico---------------------------------------



En este caso se calculó primero el modelo dinámico del robot y posteriormente se aplicó un control operacional y articular.

Para calcular el modelo dinámico se utilizó el código en Python llamado "modelo_din.py"

Para la aplicación de control en el espacio operacional se utilizó el código en Python llamado "control_operacional.py"

Para la aplicación de control en el espacio articular se utilizó el código en Python llamado "control_articular.py"
