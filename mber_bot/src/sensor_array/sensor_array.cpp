#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "bio_def.h"
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
#define BIO_POS        0x77
#define I2C_DEV_NAME   "/dev/i2c-2"
#define I2C_FNAME_SIZE 40

#define BME_MODE      0b11
#define BME_SAMPLING  0b101
#define BME_FILTER    0
#define BME_STANDBY   0
#define BME_ROOM_TEMP 26
bme280_calib_data BME_CAL_DATA;

void setup_bio_sensor (int i2c_file) {
	uint8_t msg;
	uint8_t status = 0;
	uint8_t utemp16[2];
	uint8_t * utemp8;

	if (ioctl(i2c_file, I2C_SLAVE, BIO_POS) < 0) {
		printf("Failed to acquire bus access and/or talk to BME slave.\n");
		exit(1);
	}

	msg = 0xB6;
	pwrite(i2c_file, &msg, 1, BME280_REGISTER_SOFTRESET);
	usleep(3000);

	while (!status) {
		printf("BME reading status\n");
		pread(i2c_file, &status, 1, BME280_REGISTER_STATUS);
		usleep(100);
	}

	usleep(10000);

	/* Read calibration data */
	/* Temperature */
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_T1);
	BME_CAL_DATA.dig_T1 = (uint16_t)(utemp16[0] << 8 + utemp16[1]);
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_T2);
	BME_CAL_DATA.dig_T2 =  (int16_t)(utemp16[0] << 8 + utemp16[1]);
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_T3);
	BME_CAL_DATA.dig_T3 =  (int16_t)(utemp16[0] << 8 + utemp16[1]);

	/* Pressure */
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_P1);
	BME_CAL_DATA.dig_P1 = (uint16_t)(utemp16[0] << 8 + utemp16[1]);
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_P2);
	BME_CAL_DATA.dig_P2 =  (int16_t)(utemp16[0] << 8 + utemp16[1]);
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_P3);
	BME_CAL_DATA.dig_P3 =  (int16_t)(utemp16[0] << 8 + utemp16[1]);
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_P4);
	BME_CAL_DATA.dig_P4 =  (int16_t)(utemp16[0] << 8 + utemp16[1]);
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_P5);
	BME_CAL_DATA.dig_P5 =  (int16_t)(utemp16[0] << 8 + utemp16[1]);
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_P6);
	BME_CAL_DATA.dig_P6 =  (int16_t)(utemp16[0] << 8 + utemp16[1]);
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_P7);
	BME_CAL_DATA.dig_P7 =  (int16_t)(utemp16[0] << 8 + utemp16[1]);
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_P8);
	BME_CAL_DATA.dig_P8 =  (int16_t)(utemp16[0] << 8 + utemp16[1]);
	pread(i2c_file, utemp16, 2, BME280_REGISTER_DIG_P9);
	BME_CAL_DATA.dig_P9 =  (int16_t)(utemp16[0] << 8 + utemp16[1]);
	
	/* TODO: Humidity */

	msg = BME_SAMPLING;
	pwrite(i2c_file, &msg, 1, BME280_REGISTER_CONTROLHUMID);

	msg = (BME_STANDBY << 5) | (BME_FILTER << 3) | 0;
	pwrite(i2c_file, &msg, 1, BME280_REGISTER_CONFIG);

	msg = (BME_SAMPLING << 5) | (BME_SAMPLING << 3) | BME_MODE;
	pwrite(i2c_file, &msg, 1, BME280_REGISTER_CONTROL);


}

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
		printf("Step 1: byte = %x\n", (uint32_t)*(uint32_t*)tmp);
		raw = (uint16_t)((tmp[0] << 8) + tmp[1]);
		printf("Step 2: byte = %x\n", raw);
		raw &= 0x07ff; /* Temp Sensor MSB 5-bits burned out */
		raw >>= 2;
		data = (float)raw;
		data *= 0.03125;
	}

	return data;
}

float get_BME_data(int i2c_file)
{
	uint32_t raw;
	char     tmp[4];
	char     channel;
	int      i;
	int64_t  var1, var2, p;

	if (ioctl(i2c_file, I2C_SLAVE, BIO_POS) < 0) {
		printf("Failed to acquire bus access and/or talk to BME slave.\n");
		exit(1);
	}

	printf("D1 = %x\n", BME_CAL_DATA.dig_P1);
	printf("D1 = %x\n", BME_CAL_DATA.dig_P2);
	printf("D1 = %x\n", BME_CAL_DATA.dig_P3);
	printf("D1 = %x\n", BME_CAL_DATA.dig_P4);
	printf("D1 = %x\n", BME_CAL_DATA.dig_P5);
	printf("D1 = %x\n", BME_CAL_DATA.dig_P6);
	printf("D1 = %x\n", BME_CAL_DATA.dig_P7);
	printf("D1 = %x\n", BME_CAL_DATA.dig_P8);
	printf("D1 = %x\n", BME_CAL_DATA.dig_P9);

	if (pread(i2c_file, &tmp, 3, BME280_REGISTER_PRESSUREDATA) != 3) {
		printf("Failed to read from the i2c bus.\n\n");
	} else {
		raw = (uint32_t)*(uint32_t*)tmp;
		printf("BME Step 1: byte = %x\n", raw);
		raw >>= 4;
		printf("BME Step 2: byte = %x\n", raw);

		var1 = 26 - 128000;
		var2 = var1 * var1 * (int64_t)BME_CAL_DATA.dig_P6;
		printf("BME Step 3: var1 = %ld, var2 = %ld, CAL = %x\n", var1, var2, BME_CAL_DATA.dig_P6);

		var2 = var2 + ((var1*(int64_t)BME_CAL_DATA.dig_P5)<<17);
		var2 = var2 + (((int64_t)BME_CAL_DATA.dig_P4)<<35);
		var1 = ((var1 * var1 * (int64_t)BME_CAL_DATA.dig_P3)>>8) +
				((var1 * (int64_t)BME_CAL_DATA.dig_P2)<<12);
		var1 = (((((int64_t)1)<<47)+var1))*((int64_t)BME_CAL_DATA.dig_P1)>>33;

		printf("BME Step 4: var1 = %ld, var2 = %ld\n", var1, var2);

		if (var1 == 0) {
			printf("BME RESET\n");
			return raw;
		}
		p = 1048576 - raw;
		p = (((p<<31) - var2)*3125) / var1;
		var1 = (((int64_t)BME_CAL_DATA.dig_P9) * (p>>13) * (p>>13)) >> 25;
		var2 = (((int64_t)BME_CAL_DATA.dig_P8) * p) >> 19;
		printf("BME Step 5: p = %ld, var1 = %ld, var2 = %ld\n", p, var1, var2);
		p = ((p + var1 + var2) >> 8) + (((int64_t)BME_CAL_DATA.dig_P7)<<4);
	}

	return (float)p/256;
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
	ros::NodeHandle n_bio;
	ros::Publisher sensor_publish_CO2 =
		n_CO2.advertise<std_msgs::String>("mber_sensors_CO2", 1000);
	ros::Publisher sensor_publish_radiation =
		n_geiger.advertise<std_msgs::String>("mber_sensors_radiation", 1000);
	ros::Publisher sensor_publish_temperature =
		n_temp.advertise<std_msgs::Float32>("mber_sensors_temperature", 1000);
	ros::Publisher sensor_publish_bio =
		n_bio.advertise<std_msgs::Float32>("mber_sensors_bio", 1000);
	ros::Rate loop_rate(10);

	/** I2C Init Stuff **/
	int  i2c_file;
	char i2c_filename[I2C_FNAME_SIZE];
	sprintf(i2c_filename, I2C_DEV_NAME);
	if ((i2c_file = open(i2c_filename, O_RDWR)) < 0) {
		printf("Failed to open the bus.");
		exit(1);
	}

	/* Set up BME Sensor */
	setup_bio_sensor(i2c_file);

	int count = 0;
	float pressure = 0;
	while (ros::ok()) {
		std_msgs::Float32 msg;
		float msg_data;

		/* Temperature Block */
		msg_data = get_TEMP_data(i2c_file);
		msg.data = msg_data;
		ROS_INFO("Temperature: %f", msg_data);
		sensor_publish_temperature.publish(msg);

		/* BME Block */
		msg_data = get_BME_data(i2c_file);
		ROS_INFO("Pressure: %f", msg_data);
		if(msg_data > 1500 && msg_data < 4500) {
			pressure = msg_data / BME_ROOM_TEMP;
		}
		msg.data = pressure;
		sensor_publish_bio.publish(msg);
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}
