# Hand-Detection-With-ROS---PCL
En la entrega se adjuntan dos zips, el zip con el nombre “paquete_pcl” es donde se
encuentran los códigos con los que se han realizado la práctica. El otro zip contiene un
programa donde se han hecho las pruebas para poder obtener los mejores parámetros y el
mejor método.

Terminal 1:

● roscore

Terminal 2:

● catkin_make

● source devel/setup.bash

● roslaunch paquete_pcl programas.launch

Terminal 3:

● rviz

Terminal 4:

● rosbag play mano.bag

Terminal 5:

● rosrun tf static_transform_publisher 0 0 0 0 0 0 map camera_depth_optical_frame 50
