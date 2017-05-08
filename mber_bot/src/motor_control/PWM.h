#ifndef PWM_H_
#define PWM_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>

enum PWM_CHAN { PWM1A, PWM1B };
enum POLARITY { NORMAL, INVERSED };

int initPwm();

int setEnable(PWM_CHAN pwm, bool enable);
int setPeriod(PWM_CHAN pwm, unsigned long period);
int setDutyCycle(PWM_CHAN pwm, unsigned long dutyCycle);
int setPolarity(PWM_CHAN pwm, POLARITY polarity);

#endif PWM_H_