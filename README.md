# Hand-Detection-With-ROS---PCL
Hand detection with ros and pcl

Esta práctica consiste en analizar y seleccionar el mejor detector de objetos 3D que haga
uso de características locales. El problema básico a resolver es simple, tengo un conjunto
de vistas de objetos ya identificados y una vista de una escena donde pueden aparecer uno
o más de estos objetos.


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

Resultado: https://www.linkedin.com/posts/gonzalo-ferrando-alonso-54b77817a_robaejtica-ros-c-activity-6929749054012723200-CZCA?utm_source=share&utm_medium=member_desktop
