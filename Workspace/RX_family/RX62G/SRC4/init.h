#ifndef INIT_H

#define INIT_H
#define PCLK				48					//PCLK
#define BITREET			38400					//??????
#include"iodefine.h"

void init_clock( void );

void init_cmt0( void );

void init_cmt1( void );

void init_encoder_MTU1( void );

void init_encoder_MTU2( void );

void init_AD( void );

void init_MTU6_pwm( void );

void init_serial( void );

#endif
