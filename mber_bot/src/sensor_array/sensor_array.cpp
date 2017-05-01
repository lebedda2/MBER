#include "ros/ros.h"
#include "std_msgs/String.h"
#include <glib.h>
#include <glib/gprintf.h>
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

#define CO2_I2C_POS 40
#define TEMP_I2C_POS 41
#define GEIGER_I2C_POS 42

void sensors_ADC_init(void)
{
    int file;
    char filename[40];
    const gchar *buffer;
    int addr = 0b00101001;        // The I2C address of the ADC

    sprintf(filename,"/dev/i2c-2");
    if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    if (ioctl(file,I2C_SLAVE,addr) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }

    char buf[10] = {0};
    float data;
    char channel;

    for(int i = 0; i<4; i++) {
        // Using I2C Read
        if (read(file,buf,2) != 2) {
            /* ERROR HANDLING: i2c transaction failed */
            printf("Failed to read from the i2c bus.\n");
            buffer = g_strerror(errno);
            printf(buffer);
            printf("\n\n");
        } else {
            data = (float)((buf[0] & 0b00001111)<<8)+buf[1];
            data = data/4096*5;
            channel = ((buf[0] & 0b00110000)>>4);
            printf("Channel %02d Data:  %04f\n",channel,data);
        }
    }

    //unsigned char reg = 0x10; // Device register to access
    //buf[0] = reg;
    buf[0] = 0b11110000;

    if (write(file,buf,1) != 1) {
        /* ERROR HANDLING: i2c transaction failed */
        printf("Failed to write to the i2c bus.\n");
        buffer = g_strerror(errno);
        printf(buffer);
        printf("\n\n");
    }
}

int get_CO2_data(void)
{
	return 0;
}

int get_TEMP_data(void)
{
	return 0;
}

int get_GEIGER_data(void)
{
	return 0;
}

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
