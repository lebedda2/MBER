#!bin/bash
cd ~/Documents/image_transport_ws/
export ROS_MASTER_URI=http://192.168.1.15:11311
export ROS_IP=192.168.1.15

killall roscore
roscore &

# Sleep for 5 seconds
sleep 5

source devel/setup.bash
rosrun mber_console user_ctrl
#rosparam set /compressed_listener/image_transport compressed
#rosrun image_transport_tutorial my_subscriber __name:=compressed_listener

# Maybe do this manually?
#rqt_plot /mber_sensors_temperature
#rqt_plot /mber_sensors_radiation
#rqt_plot /mber_sensors_CO2
