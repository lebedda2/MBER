#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mber_bot_sensors");
  ros::NodeHandle n;
  ros::Publisher sensor_publish_CO2 = n.advertise<std_msgs::String>("mber_sensors_CO2", 1000);
  ros::Publisher sensor_publish_radiation = n.advertise<std_msgs::String>("mber_sensors_radiation", 1000);
  ros::Publisher sensor_publish_temperature = n.advertise<std_msgs::String>("mber_sensors_temperature", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok()) {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    //chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}