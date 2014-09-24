#ifndef INIT_H

#include"iodefine.h"

#define INIT_H
#define PCLK		48
#define BITRATE_1	115200
#define BITRATE_2	115200
#define BITRATE_3	115200
#define PWM_PERIOD			(48000000/1) / 100000
#define ON					1

void init_clock(void);

void init_CMT0(void);

void init_pwm(void);

void init_all_encoder(void);

void init_Sci_0(void);

void init_Sci_1(void);

void init_Sci_2(void);

#endif