#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
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

float get_TEMP_data(int i2c_file)
{
	uint16_t raw;
	char     tmp[4];
	float    data;
	char     channel;
	int      i;

	if (ioctl(i2c_file, I2C_SLAVE, TEMP_I2C_POS) < 0) {
		printf("Failed to acquire bus access and/or talk to temperature slave.\n");
		exit(1);
	}

	if (read(i2c_file, &tmp, 4) != 4) {
		printf("Failed to read from the i2c bus.\n\n");
	} else {
		raw = (uint16_t)((tmp[0] << 8) + tmp[1]);
		printf("Step 1: byte = %x\n", raw);
		raw >>= 2;
		data = (float)raw;
		data *= 0.03125;
	}

	return data;
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
	ros::NodeHandle n_CO2;
	ros::NodeHandle n_temp;
	ros::NodeHandle n_geiger;
	ros::Publisher sensor_publish_CO2 =
		n_CO2.advertise<std_msgs::String>("mber_sensors_CO2", 1000);
	ros::Publisher sensor_publish_radiation =
		n_geiger.advertise<std_msgs::String>("mber_sensors_radiation", 1000);
	ros::Publisher sensor_publish_temperature =
		n_temp.advertise<std_msgs::Float32>("mber_sensors_temperature", 1000);
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
		std_msgs::Float32 msg;
		float msg_data = get_TEMP_data(i2c_file);

		msg.data = msg_data;

		ROS_INFO("Temperature: %f", msg_data);

		sensor_publish_temperature.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
