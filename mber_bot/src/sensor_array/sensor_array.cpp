#include "ros/ros.h"
#include "std_msgs/String.h"
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sstream>

#define TEMP_I2C_POS   0x40
#define CO2_I2C_POS    0x41
#define GEIGER_I2C_POS 0x42
#define I2C_DEV_NAME   "/dev/i2c-2"
#define I2C_FNAME_SIZE 40

int get_TEMP_data(int i2c_file)
{
	int16_t raw;
	float   data;
	char    channel;
	int     i;

	if (ioctl(i2c_file, I2C_SLAVE, TEMP_I2C_POS) < 0) {
		printf("Failed to acquire bus access and/or talk to temperature slave.\n");
		exit(1);
	}

	if (read(i2c_file, &raw, 2) != 2) {
		printf("Failed to read from the i2c bus.\n\n");
	} else {
		raw >>= 2;
		raw *= 0.03125;
	}

	return (int)raw;
}

int get_CO2_data(void)
{
	return 0;
}

int get_GEIGER_data(void)
{
	return 0;
}

int main(int argc, char **argv)
{
	/** ROS Init Stuff **/
	ros::init(argc, argv, "mber_bot_sensors");
	ros::NodeHandle n;
	ros::Publisher sensor_publish_CO2 = n.advertise<std_msgs::String>("mber_sensors_CO2", 1000);
	ros::Publisher sensor_publish_radiation = n.advertise<std_msgs::String>("mber_sensors_radiation", 1000);
	ros::Publisher sensor_publish_temperature = n.advertise<std_msgs::String>("mber_sensors_temperature", 1000);
	ros::Rate loop_rate(10);

	/** I2C Init Stuff **/
	int  i2c_file;
	char i2c_filename[I2C_FNAME_SIZE];
	sprintf(i2c_filename, I2C_DEV_NAME);
	if ((i2c_file = open(i2c_filename, O_RDWR)) < 0) {
		printf("Failed to open the bus.");
		exit(1);
	}

	int count = 0;
	while (ros::ok()) {
		std_msgs::String msg;

		std::stringstream ss;
		ss << get_TEMP_data(i2c_file);
		msg.data = ss.str();

		ROS_INFO("Temperature: %s", msg.data.c_str());

		sensor_publish_temperature.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
