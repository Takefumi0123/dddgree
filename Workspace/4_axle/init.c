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


void init_cmt0(void)
{
	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //���W���[���X�g�b�v��Ԃ̉���
	CMT.CMSTR0.BIT.STR0 = 0; //�J�E���g����̒�~
	
	CMT0.CMCR.BIT.CKS = 2;//�N���b�N�I�� 1/128
	CMT0.CMCOR = 375;   //CMCOR�̌��� 48mhz/128/1000
	
	CMT0.CMCNT = 0;//������
	CMT0.CMCR.BIT.CMIE = 1;  //���荞�݋���
	CMT.CMSTR0.BIT.STR0 = 1;
	
	IEN(CMT0,CMI0) = 1; //���荞�ݗv�������W�X�^
	IPR(CMT0,CMI0) = 10; //�D��xMAX	
}

/******************************************************************************
*	�^�C�g�� �F �^�C�}�[�̐ݒ�
*	  �֐��� �F init_cmt1
*	  �߂�l �F void�^ �^�C�}�[�̐ݒ� 1��s
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/06/23
******************************************************************************/


void init_cmt1(void){

	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //���W���[���X�g�b�v��Ԃ̉���
	CMT.CMSTR0.BIT.STR1 = 0; //�J�E���g����̒�~
	
	CMT1.CMCR.BIT.CKS = 0;//�N���b�N�I�� 1/8
	CMT1.CMCOR = 6;   //CMCOR�̌��� 48mhz/8/1000000
	
	CMT1.CMCNT = 0;//������
	CMT1.CMCR.BIT.CMIE = 1;  //���荞�݋���
	CMT.CMSTR0.BIT.STR1 = 1;
	
	IEN(CMT1,CMI1) = 1; //���荞�ݗv�������W�X�^
	IPR(CMT1,CMI1) = 15; //�D��xMAX
	
}

/******************************************************************************
*	�^�C�g�� �F �G���R�[�_�[MTU1
*	  �֐��� �F init_encoder_MTU1
*	  �߂�l �F void�^ �G���R�[�_�[MTU1
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/06/21
******************************************************************************/

void init_encoder_MTU1(void)
{	
	IOPORT.PFCMTU.BIT.TCLKS = 1;  //MTCLKA�[�q�Ƃ��đI��
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //���W���[���X�g�b�v��Ԃ̉���
	
	PORT2.ICR.BIT.B1 = 1; //�|�[�g�I��
	PORT2.ICR.BIT.B0 = 1; //�|�[�g�I��

	MTU.TSTRA.BIT.CST1 = 0; //MTU1�̃J�E���g����̒�~
	
	MTU1.TCR.BIT.CCLR = 0; //TCNT�̃N���A�֎~ P390
	
	MTU1.TMDR1.BIT.MD = 0x04; //�ʑ��v�����[�h�P P393
	
	MTU1.TIER.BIT.TCIEV = 1; //�I�[�o�[�t���[�����ݗv��������
	MTU1.TIER.BIT.TCIEU = 1; //�A���_�[�t���[�����ݗv��������
	
	IEN(MTU1,TCIV1) = 1;
	IEN(MTU1,TCIU1) = 1;	
	
	IPR(MTU1,TCIV1) = 15;
	IPR(MTU1,TCIU1) = 15;

	MTU1.TCNT = 0x00; //������
	MTU.TSTRA.BIT.CST1 = 1; //MTU1�̃J�E���g����̊J�n
}

/******************************************************************************
*	�^�C�g�� �F �G���R�[�_�[MTU2
*	  �֐��� �F init_encoder_MTU2
*	  �߂�l �F void�^ �G���R�[�_�[MTU2
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/06/21
******************************************************************************/

void init_encoder_MTU2(void)
{	
	IOPORT.PFCMTU.BIT.TCLKS = 1;  //MTCLKA�[�q�Ƃ��đI��
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //���W���[���X�g�b�v��Ԃ̉���
		
	PORT1.ICR.BIT.B1 = 1; //�|�[�g�I��
	PORT1.ICR.BIT.B0 = 1; //�|�[�g�I��	

	MTU.TSTRA.BIT.CST2 = 0; //MTU2�̃J�E���g����̒�~
		
	MTU2.TCR.BIT.CCLR = 0; //TCNT�̃N���A�֎~ P390	
	
	MTU2.TMDR1.BIT.MD = 0x04; //�ʑ��v�����[�h�P P393
	
	MTU2.TIER.BIT.TCIEV = 1; //�I�[�o�[�t���[�����ݗv��������
	MTU2.TIER.BIT.TCIEU = 1; //�A���_�[�t���[�����ݗv��������
	
	IEN(MTU2,TCIV2) = 1;
	IEN(MTU2,TCIU2) = 1;	
	
	IPR(MTU2,TCIV2) = 15;
	IPR(MTU2,TCIU2) = 15;

	MTU2.TCNT = 0x00; //������
	MTU.TSTRA.BIT.CST2 = 1; //MTU2�̃J�E���g����̊J�n
}
/******************************************************************************
*	�^�C�g�� �F init_MTU6_pwm
*	  �֐��� �F init_MTU6_pwm
*	  �߂�l �F void�^ 
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/07/16
******************************************************************************/

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
	MSTP( MTU4 );
	MTU.TSTRA.BIT.CTS4 	      = 0; //�J�E���^����̒�~
	
	MTU4.TCNT 		= 0x00;
	MTU4.TCR.BIT.TPSC 	= 2;//0x00; //�N���b�N���� 1/1
	MTU4.TCR.BIT.CKEG 	= 0x00; //�J�E���g�G�b�W
	MTU4.TCR.BIT.CCLR 	= 0x01; //�J�E���g�N���A�v��
	
	MTU4.TMDR1.BIT.MD = 0x02; //PWM���[�h�ݒ�
	
	MTU4.TIORH.BIT.IOA = 0x01;
	MTU4.TIORH.BIT.IOB = 0x02;
	MTU4.TIORL.BIT.IOC 	= 0x01;
	MTU4.TIORL.BIT.IOD 	= 0x02;

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
/******************************************************************************
*	�^�C�g�� �F �V���A���ʐM�̐ݒ�
*	  �֐��� �F init_serial
*	  �߂�l �F void�^ �V���A���ʐM�̐ݒ�
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/06/21
******************************************************************************/

void init_serial(void)
{
	int i;
	
	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;  //���W���[���X�g�b�v��Ԃ̉���
	
	PORTD.ICR.BIT.B5 = 1; //�|�[�g�I��
	
	SCI1.SCR.BIT.TEIE = 0; //TEIE�����ݗv�����֎~ P815
	SCI1.SCR.BIT.MPIE = 0; //�ʏ�̎�M���� P815
	SCI1.SCR.BIT.RIE = 1; //RXI�����ERI�����ݗv�������� P815
	SCI1.SCR.BIT.TIE = 1; //TXI�����ݗv�������� P815
	SCI1.SCR.BIT.RE = 0; //�V���A����M������֎~ P815
	SCI1.SCR.BIT.TE = 0; //�V���A�����M������֎~ P815
	
	SCI1.SCR.BIT.CKE = 0; //�����|�[���[�g�W�F�l���[�^�BP815
	
	SCI1.SMR.BIT.CM = 0; //�������������[�h P813
	SCI1.SMR.BIT.CHR = 0; //�f�[�^��8�r�b�g�ő���M P813
	SCI1.SMR.BIT.PE = 0; //�p���e�B�r�b�g�Ȃ� P813
	SCI1.SMR.BIT.PM = 0; //�����p���e�B�ő���M P813
	SCI1.SMR.BIT.STOP = 0; //1�X�g�b�v�r�b�g
	SCI1.SMR.BIT.MP = 0; //�����۾���ʐM�@�\���֎~
	SCI1.SMR.BIT.CKS = 0; //PCLK�N���b�N(n=0)
	
	SCI1.BRR = PCLK * 1000000 / ( 64 * 0.5 * BITREET ) - 1; //BRR���W�X�^�̐ݒ�l P822
	
	for(i=0;i > 80000;i++);
	
	SCI1.SCR.BIT.RE = 1; //�V���A����M��������� P815
	SCI1.SCR.BIT.TE = 1; //�V���A�����M��������� P815	
}