#include "typedefine.h"
#include "iodefine.h"
#include "machine.h"
#include <stdlib.h>
#include <math.h>
//#include"init.h"

#define FRONT_LEFT_CW		PORTB.DR.BIT.B1		//������]�o�͋���
#define FRONT_LEFT_CCW		PORTB.DR.BIT.B3		//������]�o�͋���
#define BACK_LEFT_CW		PORTA.DR.BIT.B2		//���t��]�o�͋���
#define BACK_LEFT_CCW		PORTB.DR.BIT.B2		//���t��]�o�͋���
#define FRONT_RIGHT_CW	PORT7.DR.BIT.B4		//�E����]�o�͋���
#define FRONT_RIGHT_CCW	PORT7.DR.BIT.B6		//�E����]�o�͋���
#define BACK_RIGHT_CW		PORT7.DR.BIT.B1		//�E�t��]�o�͋���
#define BACK_RIGHT_CCW		PORT7.DR.BIT.B3		//�E�t��]�o�͋���

void init_clock(void)
{	
	SYSTEM.SCKCR.BIT.PCK = 1; //�~4 ICK�Ƃ͊֌W�Ȃ��ꏊ�̑���
	SYSTEM.SCKCR.BIT.ICK = 0; //�~8 �}�C�R�����̂��̂̑���	
}

void init_MTU6_pwm( void )
{
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;
//	MSTP( MTU6 );
	MTU.TSTRB.BIT.CST6 	      = 0; //�J�E���^����̒�~
	
	MTU6.TCNT 		= 0x00;
	MTU6.TCR.BIT.TPSC 	= 0x00; //�N���b�N���� 1/1
	MTU6.TCR.BIT.CKEG 	= 0x00; //�J�E���g�G�b�W
	MTU6.TCR.BIT.CCLR 	= 0x01; //�J�E���g�N���A�v��
	
	MTU6.TMDR1.BIT.MD = 0x02; //PWM���[�h�ݒ�
	
	MTU6.TIORH.BIT.IOA = 0x02;
	MTU6.TIORH.BIT.IOB = 0x01;
	MTU6.TIORL.BIT.IOC 	= 0x02;
	MTU6.TIORL.BIT.IOD 	= 0x01;
	
	MTU6.TGRA = 480; //�����ݒ�
	MTU6.TGRB = 240;//479; //�����ݒ�
	
	MTU6.TGRC = 480; //�����ݒ�
	MTU6.TGRD = 240;//1; //�����ݒ�
		
	MTU.TOERB.BIT.OE6B = 1;
	MTU.TOERB.BIT.OE6D = 1;
	
	MTU.TSTRB.BIT.CST6 = 1; //�J�E���^����̊J�n	
}
/******************************************************************************
*	�^�C�g�� �F init_MTU4_pwm
*	  �֐��� �F init_MTU4_pwm
*	  �߂�l �F void�^ 
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/08/26
******************************************************************************/

void init_MTU4_pwm( void )
{
//	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;
	MSTP( MTU4 ) = 0;
	MTU.TSTRA.BIT.CTS4 	      = 0; //�J�E���^����̒�~
	
	MTU4.TCNT 		= 0x00;
	MTU4.TCR.BIT.TPSC 	= 0x00; //�N���b�N���� 1/1
	MTU4.TCR.BIT.CKEG 	= 0x00; //�J�E���g�G�b�W
	MTU4.TCR.BIT.CCLR 	= 0x01; //�J�E���g�N���A�v��
	
	MTU4.TMDR1.BIT.MD = 0x02; //PWM���[�h�ݒ�
	
	MTU4.TIORH.BIT.IOA = 0x02;
	MTU4.TIORH.BIT.IOB = 0x01;
	MTU4.TIORL.BIT.IOC 	= 0x02;
	MTU4.TIORL.BIT.IOD 	= 0x01;

	MTU4.TGRA = (48000000/1)/100000; //�����ݒ�
	MTU4.TGRB = MTU4.TGRA/2; //�����ݒ�
	
	MTU4.TGRC = (48000000/1)/100000; //�����ݒ�
	MTU4.TGRD = MTU4.TGRC/2; //�����ݒ�
		
	MTU.TOERA.BIT.OE4A = 1;
	MTU.TOERA.BIT.OE4B = 1;
	MTU.TOERA.BIT.OE4C = 1;
	MTU.TOERA.BIT.OE4D = 1;
	
	
	MTU.TSTRA.BIT.CTS4 = 1; //�J�E���^����̊J�n	
}

int main(void)
{
	init_clock();
		
	init_MTU6_pwm();

	init_MTU4_pwm();
IOPORT.PFCMTU.BIT.TCLKS = 1;

	PORT9.DDR.BIT.B1 = 1;
	PORTB.DDR.BIT.B1 = 1;
	PORTB.DDR.BIT.B3 = 1;
	PORTA.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B2 = 1;
	PORT7.DDR.BIT.B4 = 1;
	PORT7.DDR.BIT.B1 = 1;
	PORT7.DDR.BIT.B6 = 1;
	PORT7.DDR.BIT.B3 = 1;
	PORTA.DDR.BIT.B0 = 1;
	PORTA.DDR.BIT.B1 = 1;	
	PORT7.DDR.BIT.B5 = 1;
	PORT7.DDR.BIT.B2 = 1;
	while(1){
		PORTB.DR.BIT.B1 = 0;
		PORTB.DR.BIT.B3 = 1;
		PORTA.DR.BIT.B2 = 0;
		PORTB.DR.BIT.B2 = 1;
		PORT7.DR.BIT.B4 = 1;
		PORT7.DR.BIT.B1 = 1;
		PORT7.DR.BIT.B6 = 0;
		PORT7.DR.BIT.B3 = 0;
		
		MTU4.TGRB = (MTU4.TGRA * 50) / 100;
		MTU4.TGRD = (MTU4.TGRC * 50) / 100;
		PORT9.DR.BIT.B1 = 1;
	}
	return 0;
}
