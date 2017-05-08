#include "PWM.h"

const char* PWM1_PATH = "/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/";
const char* PWM1A_FOLDER = "pwm0/";
const char* PWM1B_FOLDER = "pwm1/";
const int MAX_PWM_DIR_NUM = 9;

char pwmDirName[] = "pwmchip#"; // You need to add '/' at the end for this one

int initPwm(void)
{
	system("config-pin P9.14 pwm"); // Setup PWM pins 1A
	system("config-pin P9.16 pwm"); // and 1B

	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir(PWM1_PATH)) != NULL)
	{
		//printf("Looking for PWM directory.\n");
		//rewinddir(dir); // Reset search

		// Scan through directory to find our pwm directory since it randomly changes on boot
		bool scan = true;
		while ((ent = readdir(dir)) != NULL && scan == true)
		{
			// Scan through each number to find the randomly assigned directory
			for (int i = 0; i <= MAX_PWM_DIR_NUM; i++)
			{
				if (scan == true)
				{
					pwmDirName[7] = (char)(48 + i);
				}

				//printf("Test: \"%s\", \"%s\"\n", pwmDirName, ent->d_name);

				if (strcmp(ent->d_name, pwmDirName) == 0 && scan == true)
				{
					scan = false;
					//printf("Found PWM Directory at: %s\n", pwmDirName);
				}
			}
		}
		closedir(dir);

		//printf("Found PWM Directory at: %s\n", pwmDirName);
	}
	else
	{
		// could not open directory
		return ENOENT;
	}

	char init_cmd[128];

	//TODO: change directory path to that of my constants
	strcpy(init_cmd, "echo 0 > ");
	strcat(init_cmd, PWM1_PATH);
	strcat(init_cmd, pwmDirName);
	strcat(init_cmd, "/export");
	system(init_cmd); // The only way I could figure out how to create the PWM directories
	init_cmd[5] = '1';
	system(init_cmd);
}

int controlPwmFile(PWM_CHAN pwm, char *control_file, char *value)
{
	FILE *filePointer = NULL;
	char file_path[128];

	// sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/ + pwmchip#/ + (pwm0 or pwm1)/ + (what we want to control)
	strcpy(file_path, PWM1_PATH);
	strcat(file_path, pwmDirName);
	strcat(file_path, "/");
	if (pwm == PWM1A) // Decide which PWM channel to choose
		strcat(file_path, PWM1A_FOLDER);
	else
		strcat(file_path, PWM1B_FOLDER);
	strcat(file_path, control_file);

	filePointer = fopen(file_path, "w");
	if (filePointer == NULL)
		return ENOENT; // File did not open properly

	fputs(value, filePointer);

	return fclose(filePointer);
}

int setEnable(PWM_CHAN pwm, bool enable)
{
	char *value;

	if (enable == true)
		value = "1";
	else
		value = "0";

	return controlPwmFile(pwm, "enable", value);
}

int setPeriod(PWM_CHAN pwm, unsigned long period)
{
	char buffer[128];
	int check;

	check = snprintf(buffer, 128, "%lu", period);

	if (check < 0 || check >= 128)
		return EOVERFLOW; // The number is too big for our buffer

	return controlPwmFile(pwm, "period", buffer);
}

int setDutyCycle(PWM_CHAN pwm, unsigned long duty_cycle)
{
	char buffer[128];
	int check;

	check = snprintf(buffer, 128, "%lu", duty_cycle);

	if (check < 0 || check >= 128)
		return EOVERFLOW; // The number is too big for our buffer

	return controlPwmFile(pwm, "duty_cycle", buffer);
}

int setPolarity(PWM_CHAN pwm, POLARITY polarity)
{
	char *value;

	if (polarity == NORMAL)
		value = "normal";
	else
		value = "inversed";

	return controlPwmFile(pwm, "enable", value);
}