#ifndef INIT_DAFALUT

#define INIT_DAFALUT

#define FRONT_LEFT			PORTB.DR.BIT.B1		//左正回転出力許可
#define BACK_LEFT			PORTB.DR.BIT.B3		//左逆回転出力許可
#define FRONT_RIGHT		PORTA.DR.BIT.B2		//右正回転出力許可
#define BACK_RIGHT			PORTB.DR.BIT.B2		//右逆回転出力許可
#include "iodefine.h"

void init_dafault( void );

#endif
