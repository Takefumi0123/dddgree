#ifndef MOVE_H

#define MOVE_H
#define FRONT_LEFT			PORTB.DR.BIT.B1		//������]�o�͋���
#define BACK_LEFT			PORTB.DR.BIT.B3		//���t��]�o�͋���
#define FRONT_RIGHT		PORTA.DR.BIT.B2		//�E����]�o�͋���
#define BACK_RIGHT			PORTB.DR.BIT.B2		//�E�t��]�o�͋���
#include "iodefine.h"

void Move( float L_output, float R_output );

#endif
