#include"init.h"
#include"iodefine.h"

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
	
	SYSTEM.MSTPCRB.BIT.MSTPB31 = 0;	//SCI1���W���[��STOP��Ԃ�����
	SCI0.SCR.BYTE		= 0x00;		//�V���A���R���g���[�����W�X�^
										//�͌� 0x01��0x00 �ŒʐM���x���ő�ɁD����1
	PORT2.DDR.BIT.B1	= 0;		//
	PORT2.ICR.BIT.B1	= 1;		//
	PORT2.DDR.BIT.B0 	= 0;	//�ǉ�
	PORT2.ICR.BIT.B0 	= 1;	//�ǉ�
	
	SCI0.SEMR.BIT.ABCS	= 1;		//����������{�N���b�N���W�T�C�N���̊��Ԃ��P�r�b�g���Ԃ̓]�����[�g�Ƃ���
	
	#if BITRATE_0 == 9600
		SCI0.SMR.BYTE		= 0x01;	
	#else
		SCI0.SMR.BYTE		= 0x00;	
	#endif
	
	SCI0.BRR = ((48*1000000)/((64/(1+SCI0.SEMR.BIT.ABCS))*powf(2,2*SCI0.SMR.BIT.CKS-1)*BITRATE_0)-1);
	
	for( bit_count = 0; bit_count < 0x800000; bit_count++ ){	//�P�r�b�g�҂���
	}
	SCI0.SCR.BYTE		= 0x70;		//����M���������
	
	IEN(SCI0,RXI0) = 1;
	IPR(SCI0,RXI0) = 11;
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
	
	SCI1.SMR.BIT.CKS 		= 0;											//PCLK�N���b�N n=0
	SCI1.SMR.BIT.CHR 	= 0;											//p830
	SCI1.SMR.BIT.PE		= 0;											//p830
	SCI1.SMR.BIT.MP		= 0;											//p830	
	SCI1.SMR.BIT.STOP	= 0;											//p830
	
	SCI1.BRR = 48000000 / ( 64 * 0.5 * BITRATE_1 ) - 1;
	
	for( bit_count = 0; bit_count < 0x800000; bit_count++ ){	//�P�r�b�g�҂���
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