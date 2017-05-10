#include "ros/ros.h"
#include "std_msgs/String.h"
#include "PWM.h"

const unsigned long CENTER = 1500000;
const unsigned long RANGE = 1000000;

using namespace std;

// Left Motor is P9.14 - PWM1A
// Right Motor is P9.16 - PWM1B

unsigned long convertSpeedToDC(float speed)
{
	// In Range and Coerce
	if (speed >= 1)
		speed = 1;
	else if (speed <= -1)
		speed = -1;

	return (long)((speed * (RANGE / 2)) + CENTER);
}

void updateMotors(const std_msgs::String::ConstPtr& msg)
{
	float leftSpeed;
	float rightSpeed;
	
	if (strcmp("U", msg->data.c_str()) == 0) // Go Forward
	{
		leftSpeed = 0.8;
		rightSpeed = -0.8;
	}
	else if (strcmp("D", msg->data.c_str()) == 0) // Go Backwards
	{
		leftSpeed = -0.8;
		rightSpeed = 0.8;
	}
	else if (strcmp("L", msg->data.c_str()) == 0) // Go Left
	{
		leftSpeed = 0.3;
		rightSpeed = -0.8;
	}
	else if (strcmp("R", msg->data.c_str()) == 0) // Go Right
	{
		leftSpeed = 0.8;
		rightSpeed = -0.3;
	}
	else // Stop
	{
		leftSpeed = 0.0;
		rightSpeed = 0.0;
	}

	setDutyCycle(PWM1A, convertSpeedToDC(leftSpeed));
	setDutyCycle(PWM1B, convertSpeedToDC(leftSpeed));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mber_bot_motor");

	initPwm();
	setEnable(PWM1A, true);
	setEnable(PWM1B, true);
	setPeriod(PWM1A, 20000000);
	setPeriod(PWM1B, 20000000);
	setDutyCycle(PWM1A, CENTER);
	setDutyCycle(PWM1B, CENTER);
  
	ros::NodeHandle n_motor_ctrl;
	ros::Subscriber motor_sub = n_motor_ctrl.subscribe("rc_motor_ctrl", 1000, updateMotors);
	ros::spin();
  
	return 0;
}
