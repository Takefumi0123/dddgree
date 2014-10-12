#ifndef CAL_MO_O_H

#define CAL_MO_O_H
#define M_PI 					3.14159265
#include "calculate.h"
#include"calculate_motor_output.h"
#include"iodefine.h"
#include "machine.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

float get_motor_output_lf(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_rf(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_lb(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_rb(float motor_output_x,float motor_output_y,float degree_now);

#endif
