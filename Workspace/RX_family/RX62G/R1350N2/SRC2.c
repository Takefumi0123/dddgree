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

#include "iodefine.h"
#include "init.h"
#include <math.h>
#include <stdio.h>

#define ON			1				//�X�C�b�`���
#define OFF			0

int	g_timer_count       	=	0;	//1ms�J�E���g
float g_Angle;
float g_Rate;
float g_Angle_f;
float g_Rate_f;
float g_X_acc;
float g_Y_acc;
float g_Z_acc;



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
	if (SCI2.SSR.BIT.RDRF == 0);		//RDRF = 0�FSCRDR �ɗL���Ȏ�M�f�[�^���i�[����Ă��Ȃ����Ƃ�\��
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
	char str[50] = {0};
	//unsigned char index;
	//unsigned int reserved;
	receive_pac[i] = Receive_uart_c();//�󂯎��
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
			PORT9.DR.BIT.B1 = 1;
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
			
			if(g_X_acc > 10000){
				g_X_acc = g_X_acc - 65535;
			}

			if(g_Y_acc > 10000){
				g_Y_acc = g_Y_acc - 65535;
			}
			
			if(g_Z_acc > 10000){
				g_Z_acc = g_Z_acc - 65535;
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
	char str[100]={0};
	int b = 1;
	
	init_clock();
	init_cmt0();
	init_serial_sci1();
	init_serial_sci2();
	
	PORT9.DDR.BIT.B1 = 1;
	
	while(1){
		if( g_timer_count >= 5 ){
			g_timer_count = 0;
			sprintf(str, "%f  ,%f  ,%f  ,%f  ,%f\n\r",g_Rate,g_Angle,g_X_acc,g_Y_acc,g_Z_acc);
			transmission_string(str);			
		}
	}
}