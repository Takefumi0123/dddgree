#ifndef INIT_DAFALUT

#define INIT_DAFALUT

#define FRONT_LEFT			PORTB.DR.BIT.B1		//������]�o�͋���
#define BACK_LEFT			PORTB.DR.BIT.B3		//���t��]�o�͋���
#define FRONT_RIGHT		PORTA.DR.BIT.B2		//�E����]�o�͋���
#define BACK_RIGHT			PORTB.DR.BIT.B2		//�E�t��]�o�͋���
#include "iodefine.h"

void init_dafault( void );

#endif
