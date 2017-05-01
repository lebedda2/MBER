#!bin/bash
cd ~/Documents/image_transport_ws/
export ROS_MASTER_URI=http://10.200.61.8:11311
export ROS_IP=10.200.138.113

source devel/setup.bash
rosrun image_transport_tutorial my_publisher