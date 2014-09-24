/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "typedefine.h"
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif
#include "typedefine.h"
#include "iodefine.h"
#include "machine.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define PWM_PERIOD			(48000000/1) / 100000
#define ON					1
#define OFF					0
#define NO_DATA				0
#define INTERRUPT_START		CMT.CMSTR0.BIT.STR0 = 1;//�J�E���g�J�n
#define INTERRUPT_TIME 			10
#define STICK_NO_MOVE_RANGE	64.0		//����肪�����Ȃ��X�e�B�b�N�̒l�͈̔�
#define RIGHT_FLONT_CW			PORT7.DR.BIT.B2
#define RIGHT_FLONT_CCW		PORT7.DR.BIT.B0
#define LEFT_BACK_CW			PORT7.DR.BIT.B4
#define LEFT_BACK_CCW			PORT7.DR.BIT.B6
#define LEFT_FLONT_CW			PORT7.DR.BIT.B5
#define LEFT_FLONT_CCW			PORT7.DR.BIT.B7
#define RIGHT_BACK_CW			PORT7.DR.BIT.B3
#define RIGHT_BACK_CCW			PORT7.DR.BIT.B1
#define RIGHT_FLONT_DUTY		MTU6.TGRB
#define LEFT_FLONT_DUTY		MTU4.TGRD
#define LEFT_BACK_DUTY			MTU4.TGRB
#define RIGHT_BACK_DUTY		MTU6.TGRD
#define RIGHT_FLONT_MAX_DUTY	MTU4.TGRA
#define LEFT_FLONT_MAX_DUTY	MTU6.TGRA
#define LEFT_BACK_MAX_DUTY		MTU6.TGRC
#define RIGHT_BACK_MAX_DUTY	MTU4.TGRC
#define M_PI 					3.14159265
#define LIMIT_MOTOR_DUTY_TIRE	95
#define BRAKE					1000
#define BITRATE_1				115200
#define  PCLK					48

	union psdate1{
		unsigned long dword;
		struct{
			unsigned char byte1;
			unsigned char model_number;
			unsigned char byte3;
			unsigned char select_sw:1;
			unsigned char l3_sw:1;
			unsigned char r3_sw:1;
			unsigned char start_sw:1;
			unsigned char up_sw:1;
			unsigned char right_sw:1;
			unsigned char down_sw:1;
			unsigned char left_sw:1;
		}byte;
	};
	union psdate2{
		unsigned long dword;
		struct{
			unsigned char l2_sw:1;
			unsigned char r2_sw:1;
			unsigned char l1_sw:1;
			unsigned char r1_sw:1;
			unsigned char triangle_sw:1;
			unsigned char circle_sw:1;
			unsigned char cross_sw:1;
			unsigned char square_sw:1;
			unsigned char right_stick_wide;
			unsigned char right_stick_high;
			unsigned char left_stick_wide;
		}byte;
	};
	union psdate3{
		unsigned long dword;
		struct{
			unsigned char left_stick_high;
		}byte;
	};
	
float g_interrupt_timer_count = 0.00,
	g_right_flont_motor_timer_count = 0.00,
	g_right_back_motor_timer_count = 0.00,
	g_left_flont_motor_timer_count = 0.00,
	g_left_back_motor_timer_count = 0.00;
volatile unsigned long	g_controller_receive_1st	= 0;	//�R���g���[������A���Ă���f�[�^�̊i�[�t�H���_1st�͕s��ł悢�B
volatile unsigned long	g_controller_receive_2nd	= 0;
volatile unsigned long	g_controller_receive_3rd	= 0;
int 	getdate1 = 0, getdate2 = 0, getdate3 = 0;

void init_clock(void)
{	
	SYSTEM.SCKCR.BIT.PCK = 1; //�~4 ICK�Ƃ͊֌W�Ȃ��ꏊ�̑���
	SYSTEM.SCKCR.BIT.ICK = 0; //�~8 �}�C�R�����̂��̂̑���	
}

/******************************************************************************
*	�^�C�g�� �F �x���̃��W�A���ϊ�
*	  �֐��� �F convert_radian
*	  �߂�l �F float�^
*	    ����1�F float�^ degree 
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
float convert_radian(float degree)
{
	float radian = 0.0;
	radian = degree * (M_PI / 180);
	return (radian);
}

/******************************************************************************
*	�^�C�g�� �F �ݒ肵���͈͓��̒l��Ԃ�
*	  �֐��� �F Limit_ul
*	  �߂�l �F float�^ �o�͒l
*	   ����1 �F float�^ upper  ����̐��l
*	   ����2 �F float�^ lower  �����̐��l
*	   ����3 �F float�^ figure  ��r���鐔�l
*	  �쐬�� �F �s�� �q��
*	  �쐬�� �F 2011/08/31
******************************************************************************/
float Limit_ul(float upper,float lower,float figure)
{
	if(upper < figure){
		return(upper);
	}else if(figure < lower){
		return(lower);
	}else{
		return(figure);
	}
}

/******************************************************************************
*	�^�C�g�� �FD�@U�@A�@L�@S�@H�@O�@C�@K
*	  �֐��� �F init_Rspi_dualshock
*	  �߂�l �F void�^ 
*	   ����1 �F char�^ s[]  
******************************************************************************/
void init_Rspi_dualshock(void)	//(�f���A���V���b�N�p)
{
	MSTP(RSPI1)			= 0;	//RSPI1���W���[���X�g�b�v�̉���
	
	RSPI1.SPCR.BYTE		= 0x00;	//�n�߂�RSPI�ʐM���g�p���邽�߂�0x00�ŒʐM��L����
	
	RSPI1.SPPCR.BIT.SPLP	= 0;	//RSPI���[�v�o�b�N�r���h=�ʏ탂�[�h
	RSPI1.SPPCR.BIT.SPLP2	= 0;	//RSPI2�X�[�v�o�b�N�r���h=�ʏ탂�[�h
	RSPI1.SPPCR.BIT.SPOM	= 0;	//RSPI�o�͒[�q���[�h�r�b�g=CMOS�o��
	RSPI1.SPPCR.BIT.MOIFV	= 1;	//MOSI�A�C�h���Œ�l�r�b�g1
	RSPI1.SPPCR.BIT.MOIFE	= 1;	//MOSI�o�͒l��MOIFV�r�b�g�̐ݒ�l
	
	RSPI1.SPBR			= 75;	//RSPI�r�b�g���[�g���W�X�^=255�Œᑬ�x

	RSPI1.SPDCR.BIT.SPFC	= 0x00;	//SPDR���W�X�^�Ɋi�[�ł���t���[�������P�ɂ���
	RSPI1.SPDCR.BIT.SLSEL	= 0x00;	//SSL�[�q�o�͐ݒ�]�����|�[�g��IO�|�[�g�ɂ���=���ׂďo�͗p
	RSPI1.SPDCR.BIT.SPRDTD	= 0;	//RSPI��M/���M�f�[�^�I���r�b�g=SPDR�͎�M�o�b�t�@��ǂ݂���
	RSPI1.SPDCR.BIT.SPLW	= 1;	//SPDR���W�X�^�ւ̓����O���[�h�A�N�Z�X�B

	RSPI1.SPSCR.BIT.SPSLN	= 0x02;	//RSPI�V�[�P���X���ݒ�r�b�g=�V�[�P���X��3
	
	RSPI1.SPCKD.BIT.SCKDL	= 0x00;	//RSPCK�x���ݒ�r�b�g=1RSPCK 
	
	RSPI1.SSLND.BIT.SLNDL	= 0x00;	//SSL�l�Q�[�g�x���ݒ�r�b�g=1RSPCK
	
	RSPI1.SPND.BIT.SPNDL	= 0x00;	//RSPI���A�N�Z�X�x���ݒ�r�b�g=1RSPCK+2PCLK
	
	RSPI1.SPCR2.BIT.SPPE	= 0;	//�p���e�B�L���r�b�g�A���M�f�[�^�̃p���e�B�r�b�g��t�����Ȃ�
	RSPI1.SPCR2.BIT.SPOE	= 0;	//�p���e�B���[�h�r�b�g=�����p���e�B�C�ő���M
	RSPI1.SPCR2.BIT.SPIIE	= 1;	//�A�C�h�����荞�ݗv���̔���������
	RSPI1.SPCR2.BIT.PTE		= 0;	//�p���e�B��H���Ȑf�f�@�\�͖���
	
	RSPI1.SPCMD0.BIT.CPHA	= 1;	//��G�b�W�Ńf�[�^�ω��A�����G�b�W�Ńf�[�^�T���v��
	RSPI1.SPCMD0.BIT.CPOL	= 1;	//�A�C�h������RSPCK��'1'
	RSPI1.SPCMD0.BIT.BRDV	= 0x03;	//�x�[�X�̃r�b�g���[�g��8������I��
	RSPI1.SPCMD0.BIT.SSLA	= 0x00;	//SSL�M���A�T�[�g�ݒ�r�b�g=SSLO
	RSPI1.SPCMD0.BIT.SSLKP	= 1;	//�]���I���ォ�玟�A�N�Z�X�J�n�܂�SSL�M�����x����ێ�

	RSPI1.SPCMD0.BIT.SPB		= 0x03;	//RSPI�f�[�^���ݒ�r�b�g=32�r�b�g
	RSPI1.SPCMD0.BIT.LSBF		= 1;		//RSPILSB�t�@�[�X�g�r�b�g=LSB�t�@�[�X�g�r�b�g
	RSPI1.SPCMD0.BIT.SPNDEN	= 1;		//���A�N�Z�X�x����RSPI���A�N�Z�X�x�����W�X�^(SPND)�̐ݒ�l
	RSPI1.SPCMD0.BIT.SLNDEN	= 1;		//���A�N�Z�X�x���ݒ苖�r�b�g=SSL�l�Q�[�g�x����RSPI�X���[�u�Z���N�g�l�Q�[�g�x�����W�X�^(SSLND)�̐ݒ�l
	RSPI1.SPCMD0.BIT.SCKDEN	= 1;		//RSPCK�x����RSPCK�x����RSPI�N���b�N�x�����W�X�^(SPCKD)�̐ݒ�l
	
	RSPI1.SPCMD1.WORD = RSPI1.SPCMD0.WORD;	//4�o�C�g���̑��M���ɍs���ݒ���R�s�[������
	RSPI1.SPCMD2.WORD = RSPI1.SPCMD0.WORD;	//1�o�C�g���̑��M���ɍs���ݒ���R�s�[������
	RSPI1.SPCMD2.BIT.SPB	= 0x07;			//RSPI�f�[�^���ݒ�r�b�g=8�r�b�g

	RSPI1.SPCMD2.BIT.SSLKP	= 0;			//���M���I������ۂɏo�͂�High�ɂ��邽��

	//���荞�݃R���g���[���̐ݒ�
	//DMACA�̐ݒ�
	//���o�̓|�[�g�̐ݒ�	(����̓V���O���}�X�^�ݒ�̂��ߓ��o�͂������Ō��肳���)
	PORTE.ICR.BIT.B7 = 1;
	
	IOPORT.PFHSPI.BIT.RSPIS = 1;

	IOPORT.PFHSPI.BIT.RSPCKE	= 1;	//RSPCKB�[�q�L��
	IOPORT.PFHSPI.BIT.MOSIE		= 1;	//MOSIB�[�q�L��
	IOPORT.PFHSPI.BIT.MISOE		= 1;	//MISOB�[�q�L��
	IOPORT.PFHSPI.BIT.SSL0E		= 1;	//SSLB0�[�q�L��
	IOPORT.PFHSPI.BIT.SSL1E		= 1;	//SSLB1�[�q�L��
	IOPORT.PFHSPI.BIT.SSL2E		= 1;	//SSLB2�[�q�L��
	IOPORT.PFHSPI.BIT.SSL3E		= 1;	//SSLB3�[�q�L��
	
	RSPI1.SPCR.BIT.SPMS		= 0;	//RSPI���[�h�I���r�b�g=SPI����(4����)
	RSPI1.SPCR.BIT.TXMD 	= 0;	//�ʐM���샂�[�h�I���r�b�g=�S��d�������V���A���ʐM
	RSPI1.SPCR.BIT.MODFEN	= 0;	//���[�h�t�H���g�G���[���o���֎~
	RSPI1.SPCR.BIT.MSTR		= 1;	//RSPI�}�X�^/�X���[�u���[�h�I��=�}�X�^���[�h
	RSPI1.SPCR.BIT.SPEIE	= 0;	//RSPI�G���[���荞�ݗv���̔������֎~
	RSPI1.SPCR.BIT.SPTIE	= 0;	//RSPI���M���荞�ݗv���̔������֎~
	RSPI1.SPCR.BIT.SPE		= 1;	//RSPI�@�\��L����
	RSPI1.SPCR.BIT.SPRIE	= 1;	//RSPI��M���荞�ݗv���̔���������
	
	RSPI1.SSLP.BIT.SSLP0	= 0;	//SSL0�M����0�A�N�e�B�u
	RSPI1.SSLP.BIT.SSLP1	= 0;	//SSL1�M����0�A�N�e�B�u
	RSPI1.SSLP.BIT.SSLP2	= 0;	//SSL2�M����0�A�N�e�B�u
	RSPI1.SSLP.BIT.SSLP3	= 0;	//SSL3�M����0�A�N�e�B�u
	
	//�ȉ��s�v�̏ꍇ�͍폜�̎�
	RSPI1.SPSR.BIT.OVRF		= 0;	//�I�[�o�����G���[�Ȃ�
	RSPI1.SPSR.BIT.IDLNF	= 0;	//RSPI���A�C�h�����(��ő���Ƃ�����1��������̂��낤��)<���R�̂ł͎g�p����Ă��Ȃ�����>�H
	RSPI1.SPSR.BIT.MODF	= 0;	//���[�h�t�H���g�G���[�Ȃ�
	RSPI1.SPSR.BIT.PERF		= 0;	//�p���e�B�G���[�Ȃ�

	//�ȏ�s�v�̏ꍇ�͍폜�̎�
	RSPI1.SPCR.BYTE;	//SPCR�̃��[�h
	IEN(RSPI1,SPRI1) = 1;
	IPR(RSPI1,SPRI1) = 12;
}

/******************************************************************************
*	�^�C�g�� �F SPI�ʐM�Ŏ�M�������̂�Ԃ�
*	  �֐��� �F Rspi_send_1
*	  �߂�l �Funsigned lond�^
*	    ���� �F unsingned long moji
******************************************************************************/
unsigned long Rspi_send_1(unsigned long moji)
{
	RSPI1.SPDR.LONG = moji;
	while( RSPI1.SPSR.BIT.SPRF == 0 );	//��M�o�b�t�@�ɂȂɂ�����܂ő҂�
	return RSPI1.SPDR.LONG;
}
/******************************************************************************
*	�^�C�g�� �F SPI�ʐM�Ŏ�M�������̂�Ԃ�
*	  �֐��� �F Rspi_send_1
*	  �߂�l �Funsigned lond�^
*	    ���� �F unsingned long moji
******************************************************************************/
unsigned long Rspi_send_short_1(unsigned short int moji)
{
	RSPI1.SPDR.LONG = moji;
	while( RSPI1.SPSR.BIT.SPRF == 0 );	//��M�o�b�t�@�ɂȂɂ�����܂ő҂�
	return RSPI1.SPDR.LONG;
}
/******************************************************************************
*	�^�C�g�� �F �f���A���V���b�N����̑��M�f�[�^���i�[
*	  �֐��� �F Rspi_receive_send_line_dualshock
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
******************************************************************************/
void Rspi_recive_send_line_dualshock(void)	//DualShock�A�i���O�R���g���[��(�A�i���O���[�h��LED)�p���M�v���O����
{
	while( RSPI1.SPSR.BIT.SPRF == 1 ){				//��M�o�b�t�@���t���Ȃ烊�[�h���ăN���A����
		RSPI1.SPDR.LONG;
	}	
	g_controller_receive_1st = Rspi_send_1(0x00004201);
	g_controller_receive_2nd = Rspi_send_1(0x00000000);
	g_controller_receive_3rd = Rspi_send_short_1(0x00);
}

/******************************************************************************
*	�^�C�g�� �F PWM�̐ݒ�
*	  �֐��� �F init_pwm
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/27
******************************************************************************/
void init_pwm(void)
{
	//MTU1,MTU2,,MTU7,MTU8�͈ʑ��v�����[�h�̂���PWM�ݒ�s�v
	
	PORT7.DDR.BIT.B0 = ON;			//MOTOR_OUTPUT
	PORT7.DDR.BIT.B1 = ON;			//MOTOR_OUTPUT
	PORT7.DDR.BIT.B2 = ON;			//MOTOR_OUTPUT
	PORT7.DDR.BIT.B3 = ON;			//MOTOR_OUTPUT
	PORT7.DDR.BIT.B4 = ON;			//MOTOR_OUTPUT
	PORT7.DDR.BIT.B5 = ON;			//MOTOR_OUTPUT
	PORT7.DDR.BIT.B6 = ON;			//MOTOR_OUTPUT
	PORT7.DDR.BIT.B7 = ON;			//MOTOR_OUTPUT
	PORTA.DDR.BIT.B0 = ON;			//MOTOR_PWM
	PORTA.DDR.BIT.B2 = ON;			//MOTOR_PWM
	PORT8.DDR.BIT.B2 = ON;			//MOTOR_PWM
	PORT8.DDR.BIT.B3 = ON;			//MOTOR_PWM
	
	
	MTUA.TSTR.BIT.CST4 = 0;			//�J�E���g��~ 
	MTUB.TSTR.BIT.CST0 = 0; 		//���c�ǉ�12/22
	IOPORT.PFCMTU.BIT.MTUS4 = 1;		//�|�[�g�t�@���N�V�������W�X�^MTIOC4A-B,MTIOC4C-B�[�q��I���@�ǉ�
	SYSTEM.MSTPCRA.BIT.MSTPA8 = 0;		//MTU���j�b�g1�iMTU6�`MTU11�j�̃��W���[���X�g�b�v����
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;		//MTU���j�b�g0�iMTU0�`MTU5�j�̃��W���[���X�g�b�v���� �ǉ����c12/22
	MTU4.TCNT = 0;				//�ǉ����c12/22�@
	MTU6.TCNT = 0;				//�ǉ����c12/22
	MTU4.TCR.BIT.CCLR = 0x01;		//�ǉ����c12/22
	MTU6.TCR.BIT.CCLR = 0x01;		//�ǉ����c12/22
	MTU4.TCR.BIT.CKEG = 0x00;		//�ǉ����c12/22
	MTU6.TCR.BIT.CKEG = 0x00;		//�ǉ����c12/22
	MTU4.TCR.BIT.TPSC = 0x00;//0x03;		//�ǉ����c12/22
	MTU6.TCR.BIT.TPSC = 0x00;//0x03;		//�ǉ����c12/22

	MTU4.TMDR.BIT.MD = 0x02;		//�ǉ����c12/22
	MTU6.TMDR.BIT.MD = 0x02;		//�ǉ����c12/22
//	MTUB.TOER.BIT.OE0B = 1;			//PWM�o�͋��� MTIOC10A
//	MTUB.TOER.BIT.OE0D = 1;			//PWM�o�͋��� MTIOC10C
	MTUA.TOER.BIT.OE4A = 1;
	MTUA.TOER.BIT.OE4C = 1;
	
	MTU4.TIORH.BIT.IOA = 2;			//�ǉ����c12/22
	MTU4.TIORH.BIT.IOB = 1;			//�ǉ����c12/22
	MTU6.TIORH.BIT.IOA = 2;			//�ǉ����c12/22
	MTU6.TIORH.BIT.IOB = 1;			//�ǉ����c12/22

	MTU4.TIORL.BIT.IOC = 2;			//�ǉ����c12/22
	MTU4.TIORL.BIT.IOD = 1;			//�ǉ����c12/22
	MTU6.TIORL.BIT.IOC = 2;			//�ǉ����c12/22
	MTU6.TIORL.BIT.IOD = 1;			//�ǉ����c12/22

	MTU4.TGRA = PWM_PERIOD;		//�ǉ����c12/22
	MTU4.TGRC = PWM_PERIOD;		//�ǉ����c12/22
	MTU6.TGRA = PWM_PERIOD;//_OTHER;		//�ǉ����c12/22
	MTU6.TGRC = PWM_PERIOD;//_OTHER;		//�ǉ����c12/22
	
	MTUA.TSTR.BIT.CST4 = 1;			//���c�ǉ�12/22	//2013.02.18	CST3��CST4	MTU4�̂���3���Ɠ����Ȃ�����
	MTUB.TSTR.BIT.CST0 = 1; 		//���c�ǉ�12/22
}

void init_CMT0(void)
{
	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0;//CMT0�̃��W���[���X�g�b�v��Ԃ�����
	CMT.CMSTR0.BIT.STR0 = 0;//�J�E���g��~
	
	CMT0.CMCR.BIT.CKS = 2;//PCLK/128
	CMT0.CMCOR = 375;//48000000/128/1000 = 375
	CMT0.CMCNT = 0;
	CMT0.CMCR.BIT.CMIE = 1;
	IEN(CMT0,CMI0) = 1;//���荞�݋���
	IPR(CMT0,CMI0) = 14;//���荞�ݗD��x���ő��
}

void wait_interrupt(void)
{
	IR(CMT0,CMI0) = OFF;
	
	g_interrupt_timer_count ++;
	g_right_flont_motor_timer_count ++;
	g_right_back_motor_timer_count ++;
	g_left_flont_motor_timer_count ++;
	g_left_back_motor_timer_count ++;
}

/******************************************************************************
*	�^�C�g�� �F sci�ʐM������
*	  �֐��� �F init_Sci
*	  �߂�l �F void�^ 
*	    ���� �F �Ȃ�
******************************************************************************/
void init_Sci(void)	//SCI1��
{
	
	int bit_count = 0;

	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;	//SCI1���W���[��STOP��Ԃ�����
	
	SCI1.SCR.BYTE		= 0x00;		//�V���A���R���g���[�����W�X�^
	
	PORT3.DDR.BIT.B0	= 0;		//
	
	PORT3.ICR.BIT.B0	= 1;		//
	
	/*SCI1.SMR.BYTE		= 0x00;		//�V���A�����[�h���W�X�^

	SCI1.BRR			= 25;		//�r�b�g���[�g���W�X�^ 
								//9600bps�F�V���A�����[�h���W�X�^��0x01��77
								//115200bps�F25
								//230400bps:21
	
	SCI1.SEMR.BIT.ABCS	= 1;		//����������{�N���b�N���W�T�C�N���̊��Ԃ��P�r�b�g���Ԃ̓]�����[�g�Ƃ���
	*/
	SCI1.SEMR.BIT.ABCS	= 1;		//����������{�N���b�N���W�T�C�N���̊��Ԃ��P�r�b�g���Ԃ̓]�����[�g�Ƃ���
	
	#if BITRATE_1 == 9600
		SCI1.SMR.BYTE		= 0x01;	
	#else
		SCI1.SMR.BYTE		= 0x00;	
	#endif
	
	SCI1.BRR = ((48*1000000)/((64/(1+SCI1.SEMR.BIT.ABCS))*powf(2,2*SCI1.SMR.BIT.CKS-1)*BITRATE_1)-1);	
	for( bit_count = 0; bit_count < 0x800000; bit_count++ ){	//�P�r�b�g�҂���
	}
	
	SCI1.SCR.BYTE		= 0x30;		//����M���������	
}

void All_setup(void)
{
	init_clock();
//	init_Io();
	init_CMT0();
//	init_CMT1();
	init_pwm();
//	init_all_encorder();
//	init_Ad();
	init_Sci();
//	init_Sci_2();
	init_Rspi_dualshock();
	
	INTERRUPT_START//���荞�݃^�C�}�J�n
}


void transmission_string(char *s)
{
	int i = 0;
	while(s[i] != '\0'){	
			
		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			SCI1.TDR = s[i];
			i++;
		}
	}
}
void Transmit_uart_c(char character)
{
	while(SCI1.SSR.BIT.TDRE == 0);//��TDR�̒��Ƀf�[�^���c���Ă���΃t���O=1  �o�Ă��܂ő҂�
	SCI1.TDR=character;//�f�[�^���������݁i���C�g�j
	SCI1.SSR.BIT.TDRE = 0;
	while( SCI1.SSR.BIT.TEND == 0 );
}
void String (char s[])
{
	int i=0;
	while(s[i] != '\0')
	{
		Transmit_uart_c(s[i]);
		i++;
	}
}
/******************************************************************************
*	�^�C�g�� �F ���O�I���j�^�C���̏o�͌���
*	  �֐��� �F get_motor_output_lf
*	  �߂�l �F float�^
*	    ����1 �Ffloat�^ motor_output_x
*	    ����2 �Ffloat�^ motor_output_y
*	    ����3 �Ffloat�^ degree_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/11/21
******************************************************************************/
float get_motor_output_lf(float motor_output_x,float motor_output_y,float degree_now)
{
	float 	motor_output_lf = 0.0,
		degree_reverse_x = 0.0,
		degree_reverse_y = 0.0;
	
	if(motor_output_x < 0.0){
		degree_reverse_x = 180.0;
	}else{
		degree_reverse_x = 0.0;
		}
	if(motor_output_y < 0.0){
		degree_reverse_y = 180.0;
	}else{
		degree_reverse_y = 0.0;
	}
	
	motor_output_lf = fabs(motor_output_x) * sin(convert_radian(degree_now + (120.0 + degree_reverse_x) + 90)) - fabs(motor_output_y) * cos(convert_radian(degree_now + (120.0 + degree_reverse_y) + 90));
	return(motor_output_lf);
}
/******************************************************************************
*	�^�C�g�� �F �E�O�I���j�^�C���̏o�͌���
*	  �֐��� �F get_motor_output_rf
*	  �߂�l �F float�^
*	    ����1 �Ffloat�^ motor_output_x
*	    ����2 �Ffloat�^ motor_output_y
*	    ����3 �Ffloat�^ degree_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/11/21
******************************************************************************/
float get_motor_output_rf(float motor_output_x,float motor_output_y,float degree_now)
{
	float 	motor_output_rf = 0.0,
		degree_reverse_x = 0.0,
		degree_reverse_y = 0.0;
	
	if(motor_output_x < 0.0){
		degree_reverse_x = 180.0;
	}else{
		degree_reverse_x = 0.0;
		}
	if(motor_output_y < 0.0){
		degree_reverse_y = 180.0;
	}else{
		degree_reverse_y = 0.0;
	}
	
	motor_output_rf = fabs(motor_output_x) * sin(convert_radian(degree_now +( 60.0 + degree_reverse_x) + 90)) - fabs(motor_output_y) * cos(convert_radian(degree_now + (60.0 + degree_reverse_y) + 90));
	return(motor_output_rf);
}
/******************************************************************************
*	�^�C�g�� �F ����I���j�^�C���̏o�͌���
*	  �֐��� �F get_motor_output_lb
*	  �߂�l �F float�^
*	    ����1 �Ffloat�^ motor_output_x
*	    ����2 �Ffloat�^ motor_output_y
*	    ����3 �Ffloat�^ degree_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/11/21
******************************************************************************/
float get_motor_output_lb(float motor_output_x,float motor_output_y,float degree_now)
{
	float 	motor_output_lb = 0.0,
		degree_reverse_x = 0.0,
		degree_reverse_y = 0.0;
	
	if(motor_output_x < 0.0){
		degree_reverse_x = 180.0;
	}else{
		degree_reverse_x = 0.0;
		}
	if(motor_output_y < 0.0){
		degree_reverse_y = 180.0;
	}else{
		degree_reverse_y = 0.0;
	}
	
	motor_output_lb = fabs(motor_output_x) * sin(convert_radian(degree_now  + (degree_reverse_x - 120) + 90)) - fabs(motor_output_y) * cos(convert_radian(degree_now +( degree_reverse_y - 120) + 90));
	return(motor_output_lb);
}
/******************************************************************************
*	�^�C�g�� �F �E��I���j�^�C���̏o�͌���
*	  �֐��� �F get_motor_output_rb
*	  �߂�l �F float�^
*	    ����1 �Ffloat�^ motor_output_x
*	    ����2 �Ffloat�^ motor_output_y
*	    ����3 �Ffloat�^ degree_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/11/21
******************************************************************************/
float get_motor_output_rb(float motor_output_x,float motor_output_y,float degree_now)
{
	float 	motor_output_rb = 0.0,
		degree_reverse_x = 0.0,
		degree_reverse_y = 0.0;
	
	if(motor_output_x < 0.0){
		degree_reverse_x = 180.0;
	}else{
		degree_reverse_x = 0.0;
		}
	if(motor_output_y < 0.0){
		degree_reverse_y = 180.0;
	}else{
		degree_reverse_y = 0.0;
	}
	
	motor_output_rb = fabs(motor_output_x) * sin(convert_radian(degree_now  + (degree_reverse_x - 60.0) + 90)) - fabs(motor_output_y) * cos(convert_radian(degree_now +( degree_reverse_y - 60.0) + 90));
	
	return(motor_output_rb);
}

/******************************************************************************
*	�^�C�g�� �F �E�O�^�C���̏o�̓��Z�b�g
*	  �֐��� �F Deadtime_right_flont_tire
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Deadtime_right_flont_tire(void)
{
	RIGHT_FLONT_CW = OFF;
	RIGHT_FLONT_CCW = OFF;
	RIGHT_FLONT_DUTY = OFF;
}
/******************************************************************************
*	�^�C�g�� �F �E�O�^�C���̓���
*	  �֐��� �F Move_right_flont_tire
*	  �߂�l �F void�^
*	    ����1�F float�^ right_duty 
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Move_right_flont_tire(float right_flont_duty)
{
	static int i = 0;
	static float right_flont_duty_old = 0.0;
	char string[100] = {0};

	if(right_flont_duty == BRAKE){
		right_flont_duty = 0.0;
	}
	
	if(((right_flont_duty > 0.0) && (right_flont_duty_old > 0.0)) || ((right_flont_duty < 0.0) && (right_flont_duty_old < 0.0))){
		if(fabs(right_flont_duty - right_flont_duty_old) >= 80.0){
			right_flont_duty = (right_flont_duty + right_flont_duty_old) / 2.000;
		}else{
			right_flont_duty = right_flont_duty;
		}
	}
	
	if(right_flont_duty < 0){
		if(i != 0){
			Deadtime_right_flont_tire();
			if(g_right_flont_motor_timer_count >= 10){
				i = 0;
			}
		}else{
			g_right_flont_motor_timer_count = 0;
			right_flont_duty *= (-1);
			RIGHT_FLONT_CW = OFF;
			RIGHT_FLONT_CCW = ON;
			i = 0;
		}
	}else if(right_flont_duty == BRAKE){
		if(i != 1){
			Deadtime_right_flont_tire();
			if(g_right_flont_motor_timer_count >= 10){
				i = 1;
			}
		}else{
			g_right_flont_motor_timer_count = 0;
			RIGHT_FLONT_CW = ON;
			RIGHT_FLONT_CCW = ON;
			right_flont_duty = 100.0;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_right_flont_tire();
			if(g_right_flont_motor_timer_count >= 10){
				i = 2;
			}
		}else{
			g_right_flont_motor_timer_count = 0;
			RIGHT_FLONT_CW = ON;
			RIGHT_FLONT_CCW = OFF;
			i = 2;
		}
	}
	
	if(right_flont_duty > 5){
		right_flont_duty =	Limit_ul(LIMIT_MOTOR_DUTY_TIRE , 0	, right_flont_duty);
		RIGHT_FLONT_DUTY = ((PWM_PERIOD * right_flont_duty) / 100.0);
		}
	else{
		Deadtime_right_flont_tire();
		}
	
	right_flont_duty_old = right_flont_duty;
	
//	sprintf(string,"%f,%f\n\r",right_flont_duty, RIGHT_FLONT_DUTY);
//	String(string);	
}

/******************************************************************************
*	�^�C�g�� �F ���O�^�C���̏o�̓��Z�b�g
*	  �֐��� �F Deadtime_left_tire
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Deadtime_left_flont_tire(void)
{
	LEFT_FLONT_CW = OFF;
	LEFT_FLONT_CCW = OFF;
	LEFT_FLONT_DUTY = OFF;
}
/******************************************************************************
*	�^�C�g�� �F ���O�^�C������
*	  �֐��� �F Move_left_tire
*	  �߂�l �F void�^
*	    ����1�F float�^ left_duty 
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Move_left_flont_tire(float left_flont_duty)
{
	static int i = 0;
	static float left_flont_duty_old = 0.0;
	char string[100]={0};
	if(left_flont_duty == BRAKE){
		left_flont_duty = 0.0;
	}
	
	if(((left_flont_duty > 0.0) && (left_flont_duty_old > 0.0)) || ((left_flont_duty < 0.0) && (left_flont_duty_old < 0.0))){
		if(fabs(left_flont_duty - left_flont_duty_old) >= 80.0){
			left_flont_duty = (left_flont_duty + left_flont_duty_old) / 2.000;
		}else{
			left_flont_duty = left_flont_duty;
		}
	}
	
	if(left_flont_duty < 0){
		if(i != 0){
			Deadtime_left_flont_tire();
			if(g_left_flont_motor_timer_count >= 10){
				i = 0;
			}
		}else{
			g_left_flont_motor_timer_count = 0;
			left_flont_duty *= (-1);
			LEFT_FLONT_CW = OFF;
			LEFT_FLONT_CCW = ON;
			i = 0;
		}
	}else if(left_flont_duty == BRAKE){
		if(i != 1){
			Deadtime_left_flont_tire();
			if(g_left_flont_motor_timer_count >= 10){
				i = 1;
			}
		}else{
			g_left_flont_motor_timer_count = 0;
			LEFT_FLONT_CW = ON;
			LEFT_FLONT_CCW = ON;
			left_flont_duty = 100;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_left_flont_tire();
			if(g_left_flont_motor_timer_count >= 10){
				i = 2;
			}	
		}else{
			g_left_flont_motor_timer_count = 0;
			LEFT_FLONT_CW = ON;
			LEFT_FLONT_CCW = OFF;
			i = 2;
		}
	}
	
	if(left_flont_duty > 5){
		left_flont_duty = Limit_ul(LIMIT_MOTOR_DUTY_TIRE,0.0,left_flont_duty);
		LEFT_FLONT_DUTY = (PWM_PERIOD * left_flont_duty) / 100;
	}else{
		Deadtime_left_flont_tire();
	}
	
	left_flont_duty_old = left_flont_duty;
	
//	sprintf(string,"%f,%f\n\r",left_flont_duty, LEFT_FLONT_DUTY);
//	String(string);		
}

/******************************************************************************
*	�^�C�g�� �F ����^�C���̏o�̓��Z�b�g
*	  �֐��� �F Deadtime_left_tire
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Deadtime_left_back_tire(void)
{
	LEFT_BACK_CW = OFF;
	LEFT_BACK_CCW = OFF;
	LEFT_BACK_DUTY = OFF;
}
/******************************************************************************
*	�^�C�g�� �F ����^�C������
*	  �֐��� �F Move_left_back_tire
*	  �߂�l �F void�^
*	    ����1�F float�^ left_back_duty 
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Move_left_back_tire(float left_back_duty)
{
	static int i = 0;
	static float left_back_duty_old = 0.0;
	
	if(left_back_duty == BRAKE){
		left_back_duty = 0.0;
	}
	
	if(((left_back_duty > 0.0) && (left_back_duty_old > 0.0)) || ((left_back_duty < 0.0) && (left_back_duty_old < 0.0))){
		if(fabs(left_back_duty - left_back_duty_old) >= 80.0){
			left_back_duty = (left_back_duty + left_back_duty_old) / 2.000;
		}else{
			left_back_duty = left_back_duty;
		}
	}
	
	if(left_back_duty < 0){
		if(i != 0){
			Deadtime_left_back_tire();
			if(g_left_back_motor_timer_count >= 10){
				i = 0;
			}
		}else{
			g_left_back_motor_timer_count = 0;
			left_back_duty *= (-1);
			LEFT_BACK_CW = OFF;
			LEFT_BACK_CCW = ON;
			i = 0;
		}
	}else if(left_back_duty == BRAKE){
		if(i != 1){
			Deadtime_left_back_tire();
			if(g_left_back_motor_timer_count >= 10){
				i = 1;
			}
		}else{
			g_left_back_motor_timer_count = 0;
			LEFT_BACK_CW = ON;
			LEFT_BACK_CCW = ON;
			left_back_duty = 100;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_left_back_tire();
			if(g_left_back_motor_timer_count >= 10){
				i = 2;
			}	
		}else{
			g_left_back_motor_timer_count = 0;
			LEFT_BACK_CW = ON;
			LEFT_BACK_CCW = OFF;
			i = 2;
		}
	}
	
	if(left_back_duty > 5){
		left_back_duty = Limit_ul(LIMIT_MOTOR_DUTY_TIRE,0.0,left_back_duty);
		LEFT_BACK_DUTY = ((PWM_PERIOD * left_back_duty) / 100);
	}else{
		Deadtime_left_back_tire();
	}
	
	left_back_duty_old = left_back_duty;
}
/******************************************************************************
*	�^�C�g�� �F �E��^�C���̏o�̓��Z�b�g
*	  �֐��� �F Deadtime_right_back_tire
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Deadtime_right_back_tire(void)
{
	RIGHT_BACK_CW = OFF;
	RIGHT_BACK_CCW = OFF;
	RIGHT_BACK_DUTY = OFF;
}
/******************************************************************************
*	�^�C�g�� �F �E��^�C���̓���
*	  �֐��� �F Move_right_back_tire
*	  �߂�l �F void�^
*	    ����1�F float�^ right_back_duty 
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Move_right_back_tire(float right_back_duty)
{
	static int i = 0;
	static float right_back_duty_old = 0.0;

	if(right_back_duty == BRAKE){
		right_back_duty = 0.0;
	}
	
	if(((right_back_duty > 0.0) && (right_back_duty_old > 0.0)) || ((right_back_duty < 0.0) && (right_back_duty_old < 0.0))){
		if(fabs(right_back_duty - right_back_duty_old) >= 80.0){
			right_back_duty = (right_back_duty + right_back_duty_old) / 2.000;
		}else{
			right_back_duty = right_back_duty;
		}
	}
	
	//right_duty = (int)right_duty_row;
	
	if(right_back_duty < 0){
		if(i != 0){
			Deadtime_right_back_tire();
			if(g_right_back_motor_timer_count >= 10){
				i = 0;
			}
		}else{
			g_right_back_motor_timer_count = 0;
			right_back_duty *= (-1);
			RIGHT_BACK_CW = OFF;
			RIGHT_BACK_CCW = ON;
			i = 0;
		}
	}else if(right_back_duty == BRAKE){
		if(i != 1){
			Deadtime_right_back_tire();
			if(g_right_back_motor_timer_count >= 10){
				i = 1;
			}
		}else{
			g_right_back_motor_timer_count = 0;
			RIGHT_BACK_CW = ON;
			RIGHT_BACK_CCW = ON;
			right_back_duty = 100.0;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_right_back_tire();
			if(g_right_back_motor_timer_count >= 10){
				i = 2;
			}
		}else{
			g_right_back_motor_timer_count = 0;
			RIGHT_BACK_CW = ON;
			RIGHT_BACK_CCW = OFF;
			i = 2;
		}
	}
	
	if(right_back_duty > 5){
		right_back_duty =	Limit_ul(LIMIT_MOTOR_DUTY_TIRE , 0	, right_back_duty);
		RIGHT_BACK_DUTY = ((PWM_PERIOD * right_back_duty) / 100.0);
		}
	else{
		Deadtime_right_back_tire();
		}
	
	right_back_duty_old = right_back_duty;
}
/******************************************************************************
*	�^�C�g�� �F ���[�^�֐�
*	  �֐��� �F Move
*	  �߂�l �F void�^
*	    ����1�F float�^ left_duty
*	    ����2�F float�^ right_duty
*	    ����3�F float�^ back_duty
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Move(float right_flont_duty,float left_flont_duty,float left_back_duty,float right_back_duty)
{
	Move_left_flont_tire(left_flont_duty);
	Move_right_flont_tire(right_flont_duty);
	Move_left_back_tire(left_back_duty);
	Move_right_back_tire(right_back_duty);
}

int main(void)
{
	float	Motor_output_x = 0,
		Motor_output_y = 0,
		Turn_output = 0,
		nutral_x = 127,
		nutral_y = 127,
		pwm_percent = 80*1.3,
		turn_pwm_percent = 10;
		
	float	manual_motor_output_rf = 0.00,
		manual_motor_output_lf = 0.00,
		manual_motor_output_lb = 0.00,
		manual_motor_output_rb = 0.00;
		
	char string[100] = { 0 };	
	
	union psdate1 getdate1;
	union psdate2 getdate2;
	union psdate3 getdate3;
	
	All_setup();
/*	MTU4.TGRB = PWM_PERIOD*45/100;		//�ǉ����c12/22
	MTU4.TGRD = PWM_PERIOD*45/100;		//�ǉ����c12/22
	MTU6.TGRB = PWM_PERIOD*45/100;		//�ǉ����c12/22
	MTU6.TGRD = PWM_PERIOD*45/100;		//�ǉ����c12/22
	
	LEFT_FLONT_CW = 1;
	LEFT_FLONT_CCW = 0;
	LEFT_BACK_CW = 1;
	LEFT_BACK_CCW = 0;
	RIGHT_FLONT_CW = 1;
	RIGHT_FLONT_CCW = 0;
	RIGHT_BACK_CW = 1;
	RIGHT_BACK_CCW = 0;*/

	
	
	while(1){
			if(g_interrupt_timer_count >= INTERRUPT_TIME){
				g_interrupt_timer_count = 0;
				
				//�f���A���V���b�N�̒l�����
				Rspi_recive_send_line_dualshock();
				getdate1.dword = g_controller_receive_1st;
				getdate2.dword = g_controller_receive_2nd;
				getdate3.dword = g_controller_receive_3rd;
				
				//�X�e�B�b�N�ɂ�邘�A�������̏o�͌���
				//x����
				if( getdate3.byte.left_stick_high >=  0xBF){
					Motor_output_x = ( -1 ) * fabs( ( (float)getdate3.byte.left_stick_high - ( nutral_x + STICK_NO_MOVE_RANGE ) ) * ( pwm_percent / ( nutral_x - STICK_NO_MOVE_RANGE + 1) ));
				}else if( getdate3.byte.left_stick_high <=  0x3F){
					Motor_output_x = (( nutral_x - STICK_NO_MOVE_RANGE ) - (float)getdate3.byte.left_stick_high ) * ( pwm_percent / ( nutral_x - STICK_NO_MOVE_RANGE ) );	
				}else{
					Motor_output_x = 0.0;
				}
				//y����
				if( getdate2.byte.left_stick_wide >= 0xBF ){
					Motor_output_y = ( -1 ) * fabs( ( (float)getdate2.byte.left_stick_wide - ( nutral_y + STICK_NO_MOVE_RANGE ) ) * ( pwm_percent / ( nutral_y - STICK_NO_MOVE_RANGE + 1) ) );
				}else if( getdate2.byte.left_stick_wide <= 0x3F ){
					Motor_output_y = ( ( nutral_y - STICK_NO_MOVE_RANGE ) - (float)getdate2.byte.left_stick_wide ) * ( pwm_percent / ( nutral_y - STICK_NO_MOVE_RANGE ) );	
				}else{
					Motor_output_y = 0.0;
				}

				if(getdate2.byte.right_stick_wide >= 0xBF ){
					Turn_output = 3*(-1) * ( (float)getdate2.byte.right_stick_wide - ( nutral_y + STICK_NO_MOVE_RANGE ) ) * ( (turn_pwm_percent) / ( nutral_y - STICK_NO_MOVE_RANGE + 1) );
				}else if(getdate2.byte.right_stick_wide <= 0x3F ){
					Turn_output = 3*( ( nutral_y - STICK_NO_MOVE_RANGE ) - (float)getdate2.byte.right_stick_wide ) * ( (turn_pwm_percent) / ( nutral_y - STICK_NO_MOVE_RANGE ) );	
				}else{
					Turn_output = 0;
				}
				manual_motor_output_lf = get_motor_output_lf( Motor_output_x, Motor_output_y, 0.0 );
				manual_motor_output_rf = get_motor_output_rf( Motor_output_x, Motor_output_y, 0.0 );
				manual_motor_output_lb = get_motor_output_lb( Motor_output_x, Motor_output_y, 0.0 );
				manual_motor_output_rb = get_motor_output_rb( Motor_output_x, Motor_output_y, 0.0 );

				sprintf(string,"%c,%f,%f,%f,%f\n\r",getdate1.byte.model_number,manual_motor_output_rf , manual_motor_output_lf , manual_motor_output_lb, manual_motor_output_rb);
//				sprintf(string,"%f\n\r",Turn_output );
				String(string);	
//				String("aaa");
				
				if(getdate1.byte.model_number == 's'){
					Move( manual_motor_output_rf + Turn_output , manual_motor_output_lf + Turn_output, manual_motor_output_lb + Turn_output, manual_motor_output_rb + Turn_output); 
				}
				if(getdate2.byte.cross_sw == 0 || getdate1.byte.model_number != 's'){
					LEFT_FLONT_CW = 1;
					LEFT_FLONT_CCW = 1;
					LEFT_BACK_CW = 1;
					LEFT_BACK_CCW = 1;
					RIGHT_FLONT_CW = 1;
					RIGHT_FLONT_CCW = 1;
					RIGHT_BACK_CW = 1;
					RIGHT_BACK_CCW = 1;
					RIGHT_FLONT_DUTY = PWM_PERIOD * 0.8;
					LEFT_FLONT_DUTY = PWM_PERIOD * 0.8;
					LEFT_BACK_DUTY = PWM_PERIOD * 0.8;
					RIGHT_BACK_DUTY = PWM_PERIOD * 0.8;
					
				}
			}	
	
	}
	return 0;
}

#ifdef __cplusplus
void abort(void)
{

}
#endif
