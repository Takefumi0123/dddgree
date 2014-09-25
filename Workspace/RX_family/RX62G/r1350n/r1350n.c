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
*	タイトル ： PCK ICKの設定
*	  関数名 ： init_clock
*	  戻り値 ： void型 PCK　ICKの設定
*	    引数 ： なし
*	  作成者 ： 石井
*	  作成日 ： 2014/06/21
******************************************************************************/

void init_clock(void)
{	
	SYSTEM.SCKCR.BIT.PCK = 1; //×4 ICKとは関係ない場所の速さ
	SYSTEM.SCKCR.BIT.ICK = 0; //×8 マイコンそのものの速さ	
}

/******************************************************************************
*	タイトル ： タイマーの設定
*	  関数名 ： init_cmt0
*	  戻り値 ： void型 タイマーの設定
*	    引数 ： なし
*	  作成者 ： 石井
*	  作成日 ： 2014/06/21
******************************************************************************/


void init_cmt0(void)
{
	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //モジュールストップ状態の解除
	CMT.CMSTR0.BIT.STR0 = 0; //カウント動作の停止
	
	CMT0.CMCR.BIT.CKS = 2;//クロック選択 1/128
	CMT0.CMCOR = 375;   //CMCORの決定 48mhz/128/1000
	
	CMT0.CMCNT = 0;//初期化
	CMT0.CMCR.BIT.CMIE = 1;  //割り込み許可
	CMT.CMSTR0.BIT.STR0 = 1;
	
	IEN(CMT0,CMI0) = 1; //割り込み要求許可レジスタ
	IPR(CMT0,CMI0) = 10; //優先度MAX	
}

/******************************************************************************
*	タイトル ： シリアル通信の設定
*	  関数名 ： init_serial
*	  戻り値 ： void型 シリアル通信の設定
*	    引数 ： なし
*	  作成者 ： 石井
*	  作成日 ： 2014/06/21
******************************************************************************/

void init_serial_sci1(void)
{
	int i;
	
	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;  //モジュールストップ状態の解除
	
	PORTD.ICR.BIT.B5 = 1; //ポート選択
	
	SCI1.SCR.BIT.TEIE = 0; //TEIE割込み要求を禁止 P815
	SCI1.SCR.BIT.MPIE = 0; //通常の受信動作 P815
	SCI1.SCR.BIT.RIE = 1; //RXIおよびERI割込み要求を許可 P815
	SCI1.SCR.BIT.TIE = 1; //TXI割込み要求を許可 P815
	SCI1.SCR.BIT.RE = 0; //シリアル受信動作を禁止 P815
	SCI1.SCR.BIT.TE = 0; //シリアル送信動作を禁止 P815
	
	SCI1.SCR.BIT.CKE = 0; //内臓ポーレートジェネレータ。P815
	
	SCI1.SMR.BIT.CM = 0; //調歩同期式モード P813
	SCI1.SMR.BIT.CHR = 0; //データ長8ビットで送受信 P813
	SCI1.SMR.BIT.PE = 0; //パリティビットなし P813
	SCI1.SMR.BIT.PM = 0; //偶数パリティで送受信 P813
	SCI1.SMR.BIT.STOP = 0; //1ストップビット
	SCI1.SMR.BIT.MP = 0; //ﾏﾙﾁﾌﾟﾛｾｯｻ通信機能を禁止
	SCI1.SMR.BIT.CKS = 0; //PCLKクロック(n=0)
	
	SCI1.BRR = PCLK * 1000000 / ( 64 * 0.5 * BITREET ) - 1; //BRRレジスタの設定値 P822
	
	for(i=0;i > 80000;i++);
	
	SCI1.SCR.BIT.RE = 1; //シリアル受信動作を許可 P815
	SCI1.SCR.BIT.TE = 1; //シリアル送信動作を許可 P815	
}

void init_serial_sci2(void)
{
	int i;
	
	SYSTEM.MSTPCRB.BIT.MSTPB29 = 0;  //モジュールストップ状態の解除
	
	PORTB.ICR.BIT.B5 = 1; //ポート選択
	
	SCI2.SCR.BIT.TEIE = 0; //TEIE割込み要求を禁止 P815
	SCI2.SCR.BIT.MPIE = 0; //通常の受信動作 P815
	SCI2.SCR.BIT.RIE = 1; //RXIおよびERI割込み要求を許可 P815
	SCI2.SCR.BIT.TIE = 1; //TXI割込み要求を許可 P815
	SCI2.SCR.BIT.RE = 0; //シリアル受信動作を禁止 P815
	SCI2.SCR.BIT.TE = 0; //シリアル送信動作を禁止 P815
	
	SCI2.SCR.BIT.CKE = 0; //内臓ポーレートジェネレータ。P815
	
	SCI2.SMR.BIT.CM = 0; //調歩同期式モード P813
	SCI2.SMR.BIT.CHR = 0; //データ長8ビットで送受信 P813
	SCI2.SMR.BIT.PE = 0; //パリティビットなし P813
	SCI2.SMR.BIT.PM = 0; //偶数パリティで送受信 P813
	SCI2.SMR.BIT.STOP = 0; //1ストップビット
	SCI2.SMR.BIT.MP = 0; //ﾏﾙﾁﾌﾟﾛｾｯｻ通信機能を禁止
	SCI2.SMR.BIT.CKS = 0; //PCLKクロック(n=0)
	
	SCI2.BRR = PCLK * 1000000 / ( 64 * 0.5 * BITREET ) - 1; //BRRレジスタの設定値 P822
	
	for(i=0;i > 80000;i++);
	
	SCI2.SCR.BIT.RE = 1; //シリアル受信動作を許可 P815
	SCI2.SCR.BIT.TE = 1; //シリアル送信動作を許可 P815	
}

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
	while (SCI2.SSR.BIT.RDRF == 0);		//RDRF = 0：SCRDR に有効な受信データが格納されていないことを表示
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
	char str[20] = {0};
	//unsigned char index;
	//unsigned int reserved;
	receive_pac[i] = Receive_uart_c();//受け取る
	

	//HEADER値発見
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
