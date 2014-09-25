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
#include"iodefine.h"

#define PCLK				48
#define BITREET			38400
#define ON				1
#define OFF				0

#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif

int	g_timer_count	=	0;
float g_Angle;
float g_Rate;
float g_Angle_f;
float g_Rate_f;
float g_X_acc;
float g_Y_acc;
float g_Z_acc;

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
*	�^�C�g�� �F �V���A���ʐM�̐ݒ�
*	  �֐��� �F init_serial
*	  �߂�l �F void�^ �V���A���ʐM�̐ݒ�
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/06/21
******************************************************************************/

void init_serial_sci1(void)
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

void init_serial_sci2(void)
{
	int i;
	
	SYSTEM.MSTPCRB.BIT.MSTPB29 = 0;  //���W���[���X�g�b�v��Ԃ̉���
	
	PORTB.ICR.BIT.B5 = 1; //�|�[�g�I��
	
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
	
	SCI2.BRR = PCLK * 1000000 / ( 64 * 0.5 * BITREET ) - 1; //BRR���W�X�^�̐ݒ�l P822
	
	for(i=0;i > 80000;i++);
	
	SCI2.SCR.BIT.RE = 1; //�V���A����M��������� P815
	SCI2.SCR.BIT.TE = 1; //�V���A�����M��������� P815	
}

/******************************************************************************
*	�^�C�g�� �Fp12,p13�œ�����M�f�[�^���i�[���Ԃ�
*	  �֐��� �F Receive_uart_c
*	  �߂�l �F char�^
*	    �����F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2014/01/22
******************************************************************************/
char Receive_uart_c(void)
{
	while (SCI2.SSR.BIT.RDRF == 0);		//RDRF = 0�FSCRDR �ɗL���Ȏ�M�f�[�^���i�[����Ă��Ȃ����Ƃ�\��
	SCI2.SSR.BIT.RDRF = 0;				//RDRF��ҋ@��ԂɕύX	
	return SCI2.RDR;
}

/******************************************************************************
*	�^�C�g�� �F �V���A���ʐM
*	  �֐��� �F transmission
*	  �߂�l �F void�^ �V���A���ʐM
*	   ����1 �F char�^ *s  
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/06/21
******************************************************************************/

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

/******************************************************************************
*	�^�C�g�� �FR1350N�p��M�֐�
*	  �֐��� �F input_R1350N
*	  �߂�l �F void�^ 
*	   ����1 �F void
*	  �쐬�� �F �L�{��
*	  �쐬�� �F 2014/01/29
******************************************************************************/
void input_R1350N(void)
{
	static int i = 0;
	static unsigned char receive_pac[15] = {0};
	static int read_start = OFF;
	unsigned int angle;
	unsigned int rate;
	unsigned int x_acc;
	unsigned int y_acc;
	unsigned int z_acc;
	unsigned char check_sum;
	char str[20] = {0};
	//unsigned char index;
	//unsigned int reserved;
	receive_pac[i] = Receive_uart_c();//�󂯎��
	

	//HEADER�l����
	if(receive_pac[0] == 0xAA){//�w�b�_�[�lAA
		read_start = ON;
	}else{
		read_start = OFF;
		i = 0;
		sprintf(str,"0xAA not found");
		transmission_string(str);
	}
	
	if(read_start == ON){
		i++;
		//0�`14�܂ł�1�Z�b�g�̕�����
		if(i >= 15){
			i = 0;
			read_start = OFF;

			//�p�P�b�g�̃w�b�_�[�����m�F����
			if(receive_pac[0] != 0xAA){
				sprintf(str, "Heading ERROR");
				transmission_string(str);
			}
			
			//�f�[�^��g�ݗ��Ă�
			//index = receive_pac[2];
			rate = (receive_pac[3] & 0xFF) | ((receive_pac[4] << 8) & 0xFF00);
			angle = (receive_pac[5] & 0xFF) | ((receive_pac[6] << 8) & 0XFF00);
			x_acc = (receive_pac[7] & 0xFF) | ((receive_pac[8] << 8) & 0xFF00);
			y_acc = (receive_pac[9] & 0xFF) | ((receive_pac[10] << 8) & 0XFF00);
			z_acc = (receive_pac[11] & 0xFF) | ((receive_pac[12] << 8) & 0xFF00);
			//reserved = receive_pac[13];
			
			//�`�F�b�N�T���̊m�F
			check_sum = 	receive_pac[2] + receive_pac[3] + receive_pac[4] + receive_pac[5]
					     + receive_pac[6] + receive_pac[7] + receive_pac[8] + receive_pac[9]
					     + receive_pac[10] + receive_pac[11] + receive_pac[12] + receive_pac[13];
			
			if(check_sum != receive_pac[14]){
				sprintf(str, "Check_Sum ERROR");
				transmission_string(str);
			}
			
			//�p�x�Ɗp���x�̒P�ʂ�ʏ�l�i���ɖ߂��f�[�^���L������
			g_Rate = rate / 100;
			g_Angle = angle / 100;
			g_Rate_f = rate / 100.0;
			g_Angle_f = angle / 100.0;
			g_X_acc = x_acc;
			g_Y_acc = y_acc;
			g_Z_acc = z_acc;
			
			if(g_Rate > 180){
				g_Rate = g_Rate - 655;
			}
			
			if(g_Angle > 180){
				g_Angle = g_Angle - 655;
			}
			
			if(g_Rate_f > 180){
				g_Rate_f = g_Rate_f - 655.35;
			}
			
			if(g_Angle_f > 180){
				g_Angle_f = g_Angle_f - 655.35;
			}

		}
		
	}
	
}

//�J�E���g�i1ms�j
void count_plus( void )
{	
	g_timer_count ++;
}

void main(void)
{
	char a[100]={0};
	int b = 1;
	
	init_clock();
	init_cmt0();
	init_serial_sci1();
//	init_serial_sci2();
	
	PORT9.DDR.BIT.B1 = 1;
	
	while(1){
		if( g_timer_count >= 1000 ){
			g_timer_count = 0;
			sprintf(a,"aaa\n\r");
			transmission_string(a);
//			input_R1350N();
			PORT9.DR.BIT.B1 = 1 - PORT9.DR.BIT.B1;
			
		}
	}
}

#ifdef __cplusplus
void abort(void)
{

}
#endif
