#ifndef	MOVE_H

#define	MOVE_H
#define ON			1
#define OFF			0
#define RIGHT_FLONT_CW		PORT7.DR.BIT.B0
#define RIGHT_FLONT_CCW		PORT7.DR.BIT.B2
#define LEFT_BACK_CW		PORT7.DR.BIT.B7
#define LEFT_BACK_CCW		PORT7.DR.BIT.B5
#define LEFT_FLONT_CW		PORT7.DR.BIT.B6
#define LEFT_FLONT_CCW		PORT7.DR.BIT.B4
#define RIGHT_BACK_CW		PORT7.DR.BIT.B1
#define RIGHT_BACK_CCW		PORT7.DR.BIT.B3
#define RIGHT_FLONT_DUTY	MTU6.TGRB
#define LEFT_FLONT_DUTY		MTU4.TGRB
#define LEFT_BACK_DUTY		MTU4.TGRD
#define RIGHT_BACK_DUTY		MTU6.TGRD
#define RIGHT_FLONT_MAX_DUTY	MTU4.TGRA
#define LEFT_FLONT_MAX_DUTY	MTU6.TGRA
#define LEFT_BACK_MAX_DUTY	MTU6.TGRC
#define RIGHT_BACK_MAX_DUTY	MTU4.TGRC
#define LIMIT_MOTOR_DUTY_TIRE	95	//モーターの最高出力
#define BRAKE			1000
#define PWM_PERIOD			(48000000/1) / 100000

#include"iodefine.h"
/*
void wait_timer_count(void);
void Move(float	right_flont_duty, float	left_flont_duty,float	left_back_duty,	float	right_back_duty,float limit_duty);
void Deadtime_right_flont_tire(void);
void Move_right_flont_tire(float right_flont_duty,float limit_duty);
void Deadtime_left_flont_tire(void);
void Move_left_flont_tire(float left_flont_duty,float limit_duty);
void Deadtime_left_back_tire(void);
void Move_left_back_tire(float left_back_duty,float limit_duty);
void Deadtime_right_back_tire(void);
void Move_right_back_tire(float right_back_duty,float limit_duty);

#endif*/
