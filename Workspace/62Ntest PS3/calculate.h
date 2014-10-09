#ifndef CAL_H

#define CAL_H

#include "calculate.h"
#include"iodefine.h"
#include "machine.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define M_PI 3.14159265

float convert_radian(float degree);
float Limit_ul(float upper,float lower,float figure);
float revision_degree(float degree);
float get_target_degree(float target_x,float target_y,float x_now,float y_now);
float get_distance(float target_x,float target_y,float now_x,float now_y);

#endif