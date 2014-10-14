#include"init.h"
#include"iodefine.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/******************************************************************************
*	�^�C�g�� �F PCK ICK�̐ݒ�
*	  �֐��� �F init_clock
*	  �߂�l �F void�^ PCK�@ICK�̐ݒ�
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/06/21
******************************************************************************/
void init_clock(void)
{	
	SYSTEM.SCKCR.BIT.PCK = 1; //�~4 ICK�Ƃ͊֌W�Ȃ��ꏊ�̑���
	SYSTEM.SCKCR.BIT.ICK = 0; //�~8 �}�C�R�����̂��̂̑���	
}

/******************************************************************************
*	�^�C�g�� �F �^�C�}�[�̐ݒ�
*	  �֐��� �F init_cmt0
*	  �߂�l �F void�^ �^�C�}�[�̐ݒ�
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/06/21
******************************************************************************/
void init_CMT0(void)
{
	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0;//CMT0�̃��W���[���X�g�b�v��Ԃ�����
	CMT.CMSTR0.BIT.STR0 = 0;//�J�E���g��~
	
	CMT0.CMCR.BIT.CKS = 2;//�N���b�N�I�� PCLK/128
	CMT0.CMCOR = 375; //CMCOR�̌��� 48000000/128/1000 = 375
	CMT0.CMCNT = 0;//������
	CMT0.CMCR.BIT.CMIE = 1;

	IEN(CMT0,CMI0) = 1;//���荞�ݗv�������W�X�^
	IPR(CMT0,CMI0) = 14;//�D��x
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
	PORT9.DDR.BIT.B0 = ON;			//MOTOR_OUTPUT
	PORT9.DDR.BIT.B1 = ON;			//MOTOR_OUTPUT
	PORT9.DDR.BIT.B2 = ON;			//MOTOR_OUTPUT
	PORT9.DDR.BIT.B3 = ON;			//MOTOR_OUTPUT
	PORTB.DDR.BIT.B2 = ON;			//MOTOR_OUTPUT
	PORTB.DDR.BIT.B3 = ON;			//MOTOR_OUTPUT
	PORTB.DDR.BIT.B6 = ON;			//MOTOR_OUTPUT
	PORTB.DDR.BIT.B7 = ON;			//MOTOR_OUTPUT
	PORTB.DDR.BIT.B0 = ON;			//MOTOR_PWM
	PORTB.DDR.BIT.B1 = ON;			//MOTOR_PWM
	PORTB.DDR.BIT.B4 = ON;			//MOTOR_PWM
	PORTB.DDR.BIT.B5 = ON;			//MOTOR_PWM
	PORTA.DDR.BIT.B0 = ON;			//MOTOR_PWM
	PORTA.DDR.BIT.B2 = ON;			//MOTOR_PWM
	PORT8.DDR.BIT.B2 = ON;			//MOTOR_PWM
	PORT8.DDR.BIT.B3 = ON;			//MOTOR_PWM


	MTUA.TSTR.BIT.CST4 = 0;			//�J�E���g��~
	MTUB.TSTR.BIT.CST4 = 0;			//�J�E���g��~
	MTUA.TSTR.BIT.CST3 = 0;			//���c�ǉ�12/22
	MTUB.TSTR.BIT.CST0 = 0; 		//���c�ǉ�12/22
	MTUB.TSTR.BIT.CST3 = 0; 		//���c�ǉ�12/22
	IOPORT.PFCMTU.BIT.MTUS4 = 1;		//�|�[�g�t�@���N�V�������W�X�^MTIOC4A-B,MTIOC4C-B�[�q��I���@�ǉ�
	SYSTEM.MSTPCRA.BIT.MSTPA8 = 0;		//MTU���j�b�g1�iMTU6�`MTU11�j�̃��W���[���X�g�b�v����
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;		//MTU���j�b�g0�iMTU0�`MTU5�j�̃��W���[���X�g�b�v���� �ǉ����c12/22
	MTU10.TCNT = 0;				//TCNT������������
	MTU4.TCNT = 0;				//�ǉ����c12/22�@
	MTU6.TCNT = 0;				//�ǉ����c12/22
	MTU9.TCNT = 0;				//�ǉ����c12/22
	MTU10.TCR.BIT.CCLR = 0x01;		//�����N���A/������������Ă��鑼�̃`���l���̃J�E���^�N���A��TCNT ���N���A
	MTU4.TCR.BIT.CCLR = 0x01;		//�ǉ����c12/22
	MTU6.TCR.BIT.CCLR = 0x01;		//�ǉ����c12/22
	MTU9.TCR.BIT.CCLR = 0x01;		//�ǉ����c12/22
	MTU10.TCR.BIT.CKEG = 0x00;		//�����オ��G�b�W�ŃJ�E���g
	MTU4.TCR.BIT.CKEG = 0x00;		//�ǉ����c12/22
	MTU6.TCR.BIT.CKEG = 0x00;		//�ǉ����c12/22
	MTU9.TCR.BIT.CKEG = 0x00;		//�ǉ����c12/22
	MTU10.TCR.BIT.TPSC = 0x00;//0x03;		//�����N���b�N�ݒ�FICLK/64
	MTU4.TCR.BIT.TPSC = 0x00;//0x03;		//�ǉ����c12/22
	MTU6.TCR.BIT.TPSC = 0x00;//0x03;		//�ǉ����c12/22
	MTU9.TCR.BIT.TPSC = 0x00;//0x03;		//�ǉ����c12/22

	MTU10.TMDR.BIT.MD = 0x02;		//PWM���[�h1
	MTU4.TMDR.BIT.MD = 0x02;		//�ǉ����c12/22
	MTU6.TMDR.BIT.MD = 0x02;		//�ǉ����c12/22
	MTU9.TMDR.BIT.MD = 0x02;		//�ǉ����c12/22
	
	MTUB.TOER.BIT.OE4A = 1;			//PWM�o�͋��� MTIOC10A
	MTUB.TOER.BIT.OE4C = 1;			//PWM�o�͋��� MTIOC10C
	MTUA.TOER.BIT.OE4A = 1;
	MTUA.TOER.BIT.OE4C = 1;
	
	MTU10.TIORH.BIT.IOA = 2;		//�A�E�g�v�b�g�R���y�A���W�X�^�ɐݒ�A�����l�A�o�͒l�̑I��A		���킩��Ȃ����œK�l�������c�C��12/22 =6; �� =1;
	MTU10.TIORH.BIT.IOB = 1;		//�A�E�g�v�b�g�R���y�A���W�X�^�ɐݒ�A�����l�A�o�͒l�̑I��B		���c�C��12/22 =5; �� =1;
	MTU4.TIORH.BIT.IOA = 2;			//�ǉ����c12/22
	MTU4.TIORH.BIT.IOB = 1;			//�ǉ����c12/22
	MTU6.TIORH.BIT.IOA = 2;			//�ǉ����c12/22
	MTU6.TIORH.BIT.IOB = 1;			//�ǉ����c12/22
	MTU9.TIORH.BIT.IOA = 2;			//�ǉ����c12/22
	MTU9.TIORH.BIT.IOB = 1;			//�ǉ����c12/22
	
	MTU10.TIORL.BIT.IOC = 2;		//�A�E�g�v�b�g�R���y�A���W�X�^�ɐݒ�A�����l�A�o�͒l�̑I��C		���c�C��12/22 =6; �� =2;
	MTU10.TIORL.BIT.IOD = 1;		//�A�E�g�v�b�g�R���y�A���W�X�^�ɐݒ�A�����l�A�o�͒l�̑I��D		���c�C��12/22 =5; �� =1;
	MTU4.TIORL.BIT.IOC = 2;			//�ǉ����c12/22
	MTU4.TIORL.BIT.IOD = 1;			//�ǉ����c12/22
	MTU6.TIORL.BIT.IOC = 2;			//�ǉ����c12/22
	MTU6.TIORL.BIT.IOD = 1;			//�ǉ����c12/22
	MTU9.TIORL.BIT.IOC = 2;			//�ǉ����c12/22
	MTU9.TIORL.BIT.IOD = 1;			//�ǉ����c12/22
	
	MTU10.TGRA = PWM_PERIOD;		//48000 / 4 / 8 * 0.01(���ӃN���b�N/�����N���b�N/�^�C�}�v���X�P�[��*����)[1kHz]
	MTU10.TGRC = PWM_PERIOD;		//48000 / 4 / 8 * 0.01(���ӃN���b�N/�����N���b�N/�^�C�}�v���X�P�[��*����)[1kHz]
	MTU4.TGRA = PWM_PERIOD;		//�ǉ����c12/22
	MTU4.TGRC = PWM_PERIOD;		//�ǉ����c12/22
	MTU6.TGRA = PWM_PERIOD;//_OTHER;		//�ǉ����c12/22
	MTU6.TGRC = PWM_PERIOD;//_OTHER;		//�ǉ����c12/22
	MTU9.TGRA = PWM_PERIOD;		//�ǉ����c12/22
	MTU9.TGRC = PWM_PERIOD;		//�ǉ����c12/22

	MTUA.TSTR.BIT.CST3 = 1;			//���c�ǉ�12/22	//2013.02.18	CST3��CST4	MTU4�̂���3���Ɠ����Ȃ�����
	MTUA.TSTR.BIT.CST4 = 1;			//���c�ǉ�12/22	//2013.02.18	CST3��CST4	MTU4�̂���3���Ɠ����Ȃ�����
	MTUB.TSTR.BIT.CST4 = 1;			//�J�E���g��~
	MTUB.TSTR.BIT.CST0 = 1; 		//���c�ǉ�12/22
	MTUB.TSTR.BIT.CST3 = 1; 		//���c�ǉ�12/22
}

/******************************************************************************
*	�^�C�g�� �F ���ׂẴG���R�[�_�ݒ���s��
*	  �֐��� �F init_all_encoder
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/14
******************************************************************************/
void init_all_encoder(void)
{
	PORTC.ICR.BIT.B6 = 1;	//MTCLKA�̓��̓o�b�t�@��L����
	PORTC.ICR.BIT.B7 = 1;	//MTCLKB�̓��̓o�b�t�@��L����
	PORTC.ICR.BIT.B4 = 1;	//MTCLKC�̓��̓o�b�t�@��L����
	PORTC.ICR.BIT.B5 = 1;	//MTCLKD�̓��̓o�b�t�@��L����
	PORTC.ICR.BIT.B2 = 1;	//MTCLKE�̓��̓o�b�t�@��L����
	PORTC.ICR.BIT.B3 = 1;	//MTCLKF�̓��̓o�b�t�@��L����
	PORTC.ICR.BIT.B0 = 1;	//MTCLKG�̓��̓o�b�t�@��L����
	PORTC.ICR.BIT.B1 = 1;	//MTCLKH�̓��̓o�b�t�@��L����

	IOPORT.PFCMTU.BIT.TCLKS = 1;	//PC6,7,4,5��MTCLKn-B�[�q�Ƃ��Đݒ�
	IOPORT.PFDMTU.BIT.TCLKS = 0;	//PC2,3,0,1��MTCLKn-A�[�q�Ƃ��Đݒ�

	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;
	SYSTEM.MSTPCRA.BIT.MSTPA8 = 0;

	MTUA.TSTR.BIT.CST1 = 0;//�J�E���g�����~
	MTUA.TSTR.BIT.CST2 = 0;//�J�E���g�����~
	MTUB.TSTR.BIT.CST1 = 0;//�J�E���g�����~
	MTUB.TSTR.BIT.CST2 = 0;//�J�E���g�����~

	MTU1.TMDR.BIT.MD = 4;//�ʑ��v�����[�h
	MTU2.TMDR.BIT.MD = 4;//�ʑ��v�����[�h
	MTU7.TMDR.BIT.MD = 4;//�ʑ��v�����[�h
	MTU8.TMDR.BIT.MD = 4;//�ʑ��v�����[�h

	MTU1.TCR.BIT.CCLR = 0;//TCNT�̃N���A�֎~
	MTU2.TCR.BIT.CCLR = 0;//TCNT�̃N���A�֎~
	MTU7.TCR.BIT.CCLR = 0;//TCNT�̃N���A�֎~
	MTU8.TCR.BIT.CCLR = 0;//TCNT�̃N���A�֎~

	MTUA.TSTR.BIT.CST1 = 1;//�J�E���g����J�n
	MTUA.TSTR.BIT.CST2 = 1;//
	MTUB.TSTR.BIT.CST1 = 1;//
	MTUB.TSTR.BIT.CST2 = 1;//

	IPR(MTU1,TCIV1) = 0;			//���荞�݂̗D��x���Œ�(���荞�݋֎~)�ɂ���
	IPR(MTU2,TCIV2) = 0;			//���荞�݂̗D��x���Œ�(���荞�݋֎~)�ɂ���
	IPR(MTU7,TCIV7) = 0;			//���荞�݂̗D��x���Œ�(���荞�݋֎~)�ɂ���
	IPR(MTU8,TCIV8) = 0;			//���荞�݂̗D��x���Œ�(���荞�݋֎~)�ɂ���

	MTU1.TIER.BIT.TCIEV = 1;//�I�[�o�[�t���[���荞�݂�������
	MTU1.TIER.BIT.TCIEU = 1;//�A���_�[�t���[���荞�݂�������
	MTU2.TIER.BIT.TCIEV = 1;//�I�[�o�[�t���[���荞�݂�������
	MTU2.TIER.BIT.TCIEU = 1;//�A���_�[�t���[���荞�݂�������
	MTU7.TIER.BIT.TCIEV = 1;
	MTU7.TIER.BIT.TCIEU = 1;
	MTU8.TIER.BIT.TCIEV = 1;
	MTU8.TIER.BIT.TCIEU = 1;

	IEN(MTU1,TCIV1) = 1;//���荞�ݏ���������
	IEN(MTU1,TCIU1) = 1;//���荞�ݏ���������
	IPR(MTU1,TCIV1) = 15;//���荞�ݗD��x��14

	IEN(MTU2,TCIV2) = 1;//���荞�ݏ���������
	IEN(MTU2,TCIU2) = 1;//���荞�ݏ���������
	IPR(MTU2,TCIV2) = 15;//���荞�ݗD��x��14��

	IEN(MTU7,TCIV7) = 1;//���荞�ݏ���������
	IEN(MTU7,TCIU7) = 1;//���荞�ݏ���������
	IPR(MTU7,TCIV7) = 15;//���荞�ݗD��x��14��

	IEN(MTU8,TCIV8) = 1;//���荞�ݏ���������
	IEN(MTU8,TCIU8) = 1;//���荞�ݏ���������
	IPR(MTU8,TCIV8) = 15;//���荞�ݗD��x��14��
}

/******************************************************************************
*	�^�C�g�� �F sci�ʐM������
*	  �֐��� �F init_Sci
*	  �߂�l �F void�^ 
*	    ���� �F �Ȃ�
******************************************************************************/
void init_Sci_0(void)
{
	int bit_count = 0;
	
	SYSTEM.MSTPCRB.BIT.MSTPB31 = 0;						//SCI0���W���[��STOP��Ԃ�����
	SCI0.SCR.BYTE		= 0x00;										//�V���A���R���g���[�����W�X�^
																				//�͌� 0x01��0x00 �ŒʐM���x���ő�ɁD����1
	PORT2.DDR.BIT.B1	= 0;		//
	PORT2.ICR.BIT.B1	= 1;		//
	PORT2.DDR.BIT.B0 = 0;	//�ǉ�
	PORT2.ICR.BIT.B0 	= 1;	//

	SCI0.SMR.BYTE		= 0x00;		//�V���A�����[�h���W�X�^
	SCI0.SEMR.BIT.ABCS	= 1;		//����������{�N���b�N���W�T�C�N���̊��Ԃ��P�r�b�g���Ԃ̓]�����[�g�Ƃ���
	SCI0.BRR			= ((48*1000000)/((64/(1+SCI0.SEMR.BIT.ABCS))*powf(2,2*SCI0.SMR.BIT.CKS-1)*BITRATE_0)-1);;		//�r�b�g���[�g���W�X�^77  9600bps�Ȃ�0x01��77

	for( bit_count = 0; bit_count < 0x80000; bit_count++ ){	//�P�r�b�g�҂���
	}
	SCI0.SCR.BYTE		= 0x70;		//����M���������
	
	IEN(SCI0,RXI0) = 1;
	IPR(SCI0,RXI0) = 13;
}

void init_Sci_1(void)	//SCI1��
{
	int bit_count = 0;
	
	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;							//�V���A���ʐM1 ���W���[����Ԃ̉���
	
	SCI1.SCR.BIT.TIE 	= 0;												//TXI���荞�ݗv��������
	SCI1.SCR.BIT.RIE 	= 0;												//RXI�����ERI���荞�ݗv��������
	SCI1.SCR.BIT.TE 	= 0;												//�V���A�����M������֎~
	SCI1.SCR.BIT.RE 	= 0;												//�V���A����M������֎~
	SCI1.SCR.BIT.TEIE = 0;												//TEI���荞�ݗv�����֎~
	
	SCI1.SCR.BIT.CKE	= 0;												//�����|�[���[�g�W�F�l���[�^ SCKn�[�q�͓��o�̓|�[�g�Ƃ��Ďg�p�\ p815 
	
	SCI1.SMR.BIT.CKS 	= 0;											//PCLK�N���b�N n=0
	SCI1.SMR.BIT.CHR 	= 0;											//p830
	SCI1.SMR.BIT.PE		= 0;											//p830
	SCI1.SMR.BIT.MP		= 0;											//p830	
	SCI1.SMR.BIT.STOP	= 0;											//p830
	
	SCI1.BRR = 48000000 / ( 64 * 0.5 * BITRATE_1 ) - 1;
	
	for( bit_count = 0; bit_count < 0x80000; bit_count++ ){	//�P�r�b�g�҂���
	}
	
	SCI1.SCR.BIT.TE = 1;													//�V���A�����M���������
	SCI1.SCR.BIT.RE = 1;	
}

void init_Sci_2(void)//�ǉ�
{
	int i= 0;	
	
	SYSTEM.MSTPCRB.BIT.MSTPB29 = 0;	//SCI1���W���[��STOP��Ԃ�����
										//�͌� 0x01��0x00 �ŒʐM���x���ő�ɁD����1
	PORT1.DDR.BIT.B2	= 0;		//
	PORT1.ICR.BIT.B2	= 1;		//
	PORT1.DDR.BIT.B3 	= 0;	//�ǉ�
	PORT1.ICR.BIT.B3 	= 1;	//�ǉ�

	
	SCI2.SCR.BIT.TEIE = 0; //TEIE�����ݗv�����֎~ P815
	SCI2.SCR.BIT.MPIE = 0; //�ʏ�̎�M���� P815
	SCI2.SCR.BIT.RIE = 1; //RXI�����ERI�����ݗv�������� P815
	SCI2.SCR.BIT.TIE = 1; //TXI�����ݗv�������� P815
	SCI2.SCR.BIT.RE = 0; //�V���A����M������֎~ P815
	SCI2.SCR.BIT.TE = 0; //�V���A�����M������֎~ P815
	
	SCI2.SCR.BIT.CKE = 0; //�����|�[���[�g�W�F�l���[�^�BP815
	
	SCI2.SMR.BIT.CM = 0; //�������������[�h P813
	SCI2.SMR.BIT.CHR = 0; //�f�[�^��8�r�b�g�ő���M P813
	SCI2.SMR.BIT.PE = 0; //�p���e�B�r�b�g�Ȃ� P813
	SCI2.SMR.BIT.PM = 0; //�����p���e�B�ő���M P813
	SCI2.SMR.BIT.STOP = 0; //1�X�g�b�v�r�b�g
	SCI2.SMR.BIT.MP = 0; //�����۾���ʐM�@�\���֎~
	SCI2.SMR.BIT.CKS = 0; //PCLK�N���b�N(n=0)
	
	SCI2.BRR = PCLK * 1000000 / ( 64 * 0.5 * BITRATE_2 ) - 1; //BRR���W�X�^�̐ݒ�l P822
	
	for(i=0;i > 80000;i++);
	
	SCI2.SCR.BIT.RE = 1; //�V���A����M��������� P815
	SCI2.SCR.BIT.TE = 1; //�V���A�����M��������� P815
	IEN(SCI2,RXI2) = 1;
	IPR(SCI2,RXI2) = 11;
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

void init_dafalut()
{
	PORT8.DDR.BIT.B0 = 1;
	PORT8.DDR.BIT.B1 = 1;
	PORTD.DDR.BIT.B7 = 1;
	PORT6.DDR.BIT.B0 = 1;
	
	PORT1.DDR.BIT.B4 = ON;			//�d����
	PORT2.DDR.BIT.B5 = ON;			//
	PORT3.DDR.BIT.B3 = ON;			//
	PORT3.DDR.BIT.B2 = ON;			//�d����
	PORT5.DDR.BIT.B6 = ON;			//�d����
	PORT6.DDR.BIT.B0 = ON;			//�d����
}