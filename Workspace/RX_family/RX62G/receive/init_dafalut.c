#include"init_dafalut.h"
#include"iodefine.h"

/******************************************************************************
*	�^�C�g�� �F init_dafault
*	  �֐��� �F init_dafault
*	  �߂�l �F void�^ �����ݒ�
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/07/17
******************************************************************************/

void init_dafault( void )
{	
	//���o�͂̏�����
	PORTA.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B1 = 1;
	PORTB.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B3 = 1;
	PORTB.DDR.BIT.B4 = 0;
	PORT9.DDR.BYTE   = 0x3F;
	
	//��]�����̏�����
        FRONT_LEFT  	= 0;
        BACK_LEFT    	= 0;
        FRONT_RIGHT 	= 0;
        BACK_RIGHT  	= 0;	
}
