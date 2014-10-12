#ifndef INIT_H

#define INIT_H

#define ON				1
#define PCLK				48
#define BITRATE_0			115200
#define BITRATE_1			115200
#define BITRATE_2			115200
#define PWM_PERIOD			(48000000/1) / 100000
#define PWM_PERIOD_FUN			(48000000/ 64) / 60.5

#include"iodefine.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
void init_clock(void);
void init_CMT0(void);
void init_pwm(void);
void init_all_encoder(void);
void init_Sci_0(void);
void init_Sci_1(void);
void init_Sci_2(void);
void init_Rspi_dualshock(void);

#endif