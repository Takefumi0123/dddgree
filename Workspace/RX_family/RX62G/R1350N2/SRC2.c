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

#define ON			1				//スイッチ状態
#define OFF			0

int	g_timer_count       	=	0;	//1msカウント
float g_Angle;
float g_Rate;
float g_Angle_f;
float g_Rate_f;
float g_X_acc;
float g_Y_acc;
float g_Z_acc;



/******************************************************************************
*	タイトル ：p12,p13で得た受信データを格納し返す
*	  関数名 ： Receive_uart_c
*	  戻り値 ： char型
*	    引数： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2014/01/22
******************************************************************************/
char Receive_uart_c(void)
{
	if (SCI2.SSR.BIT.RDRF == 0);		//RDRF = 0：SCRDR に有効な受信データが格納されていないことを表示
	SCI2.SSR.BIT.RDRF = 0;				//RDRFを待機状態に変更	
	return SCI2.RDR;
}

/******************************************************************************
*	タイトル ： シリアル通信
*	  関数名 ： transmission
*	  戻り値 ： void型 シリアル通信
*	   引数1 ： char型 *s  
*	  作成者 ： 石井
*	  作成日 ： 2014/06/21
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
*	タイトル ：R1350N用受信関数
*	  関数名 ： input_R1350N
*	  戻り値 ： void型 
*	   引数1 ： void
*	  作成者 ： 有本光
*	  作成日 ： 2014/01/29
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
	receive_pac[i] = Receive_uart_c();//受け取る
	if(receive_pac[0] == 0xAA){//ヘッダー値AA
		read_start = ON;
	}else{
		read_start = OFF;
		i = 0;
		sprintf(str,"0xAA not found");
		transmission_string(str);
	}
	if(read_start == ON){
		i++;
		//0～14までで1セットの文字列
		if(i >= 15){
			i = 0;
			read_start = OFF;
			PORT9.DR.BIT.B1 = 1;
			//パケットのヘッダー情報を確認する
			if(receive_pac[0] != 0xAA){
				sprintf(str, "Heading ERROR");
				transmission_string(str);
			}
			
			//データを組み立てる
			//index = receive_pac[2];
			rate = (receive_pac[3] & 0xFF) | ((receive_pac[4] << 8) & 0xFF00);
			angle = (receive_pac[5] & 0xFF) | ((receive_pac[6] << 8) & 0XFF00);
			x_acc = (receive_pac[7] & 0xFF) | ((receive_pac[8] << 8) & 0xFF00);
			y_acc = (receive_pac[9] & 0xFF) | ((receive_pac[10] << 8) & 0XFF00);
			z_acc = (receive_pac[11] & 0xFF) | ((receive_pac[12] << 8) & 0xFF00);
			//reserved = receive_pac[13];
			
			//チェックサムの確認
			check_sum = 	receive_pac[2] + receive_pac[3] + receive_pac[4] + receive_pac[5]
					     + receive_pac[6] + receive_pac[7] + receive_pac[8] + receive_pac[9]
					     + receive_pac[10] + receive_pac[11] + receive_pac[12] + receive_pac[13];
			
			if(check_sum != receive_pac[14]){
				sprintf(str, "Check_Sum ERROR");
				transmission_string(str);
			}
			
			//角度と角速度の単位を通常値（元に戻しデータを記憶する
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


//カウント（1ms）
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