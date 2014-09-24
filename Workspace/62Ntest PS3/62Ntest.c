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
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

#include "typedefine.h"
#include "iodefine.h"
#include "machine.h"
//#include "init_RX62N.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define ON					1
#define OFF					0
#define NO_DATA				0
#define INTERRUPT_START		CMT.CMSTR0.BIT.STR0 = 1;//カウント開始
#define INTERRUPT_TIME 			5
#define STICK_NO_MOVE_RANGE	0.2		//足回りが動かないスティックの値の範囲
#define OPERATE_DEGREE			90
#define RIGHT_FLONT_CW			PORT7.DR.BIT.B0
#define RIGHT_FLONT_CCW			PORT7.DR.BIT.B2
#define LEFT_BACK_CW			PORT7.DR.BIT.B7
#define LEFT_BACK_CCW			PORT7.DR.BIT.B5
#define LEFT_FLONT_CW			PORT7.DR.BIT.B6
#define LEFT_FLONT_CCW			PORT7.DR.BIT.B4
#define RIGHT_BACK_CW			PORT7.DR.BIT.B1
#define RIGHT_BACK_CCW			PORT7.DR.BIT.B3
#define RIGHT_FLONT_DUTY		MTU6.TGRB
#define LEFT_FLONT_DUTY			MTU4.TGRB
#define LEFT_BACK_DUTY			MTU4.TGRD
#define RIGHT_BACK_DUTY			MTU6.TGRD
#define RIGHT_FLONT_MAX_DUTY		MTU4.TGRA
#define LEFT_FLONT_MAX_DUTY		MTU6.TGRA
#define LEFT_BACK_MAX_DUTY		MTU6.TGRC
#define RIGHT_BACK_MAX_DUTY		MTU4.TGRC
#define M_PI 					3.14159265
#define LIMIT_MOTOR_DUTY_TIRE	95
#define BRAKE					1000
#define END '#'						//通信データの終端文字
#define RECEIVE_STR_COLUMN 32		//1データあたりの最大文字数		例: a123# (6文字)
#define VERTICAL_ENCODER			MTU1.TCNT
#define HORIZONTAL_ENCODER			MTU2.TCNT
#define DIAMETER_VERTICAL_WHEEL		51
#define DIAMETER_HORIZONTAL_WHEEL	51
#define PULSE_VERTICAL_ENCODER		500
#define PULSE_HORIZONTAL_ENCODER	500
#define START_X_COORDNATES			0
#define START_Y_COORDNATES			0
#define PI							3.14159265358979
#define PCLK		48
#define BITRATE_1	115200
#define BITRATE_2	115200
#define BITRATE_3	115200
#define PWM_PERIOD			(48000000/1) / 100000
#define TURN_P_GAIN	1.5
#define TURN_D_GAIN	10
#define LEFT_STICK_HIGH	g_AtoZ_value[0]	
#define LEFT_STICK_WIDE	g_atoz_value[0]
#define RIGHT_STICK_WIDE	g_atoz_value[1]
#define PWM_PER		95

//グローバル変数に格納する場合	おばかな例
float g_atoz_value[26] = {127.00, 127.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
float g_AtoZ_value[26] = {127.00, 127.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00};


float g_interrupt_timer_count = 0.00,
	g_interrupt_timer_count2 = 0.00 ,
	g_interrupt_timer_count3 = 0.00 ,
	g_right_flont_motor_timer_count = 0.00,
	g_right_back_motor_timer_count = 0.00,
	g_left_flont_motor_timer_count = 0.00,
	g_left_back_motor_timer_count = 0.00;
volatile unsigned long	g_controller_receive_1st	= 0;	//コントローラから帰ってくるデータの格納フォルダ1stは不定でよい。
volatile unsigned long	g_controller_receive_2nd	= 0;
volatile unsigned long	g_controller_receive_3rd	= 0;
int 	getdate1 = 0, getdate2 = 0, getdate3 = 0;
int	g_over_vertical_count	 = 0, g_under_vertical_count = 0, g_over_horizontal_count = 0, g_under_horizontal_count = 0;
float	g_x_coordnates = START_X_COORDNATES, g_y_coordnates = START_Y_COORDNATES;
float g_Angle;
float g_Rate;
float g_Angle_f;
float g_Rate_f;
float g_X_acc;
float g_Y_acc;
float g_Z_acc;
unsigned char  g_input_r1350n[15] = {0};

void All_setup(void);
void over_flow_MTU1(void);
void under_flow_MTU1(void);
void over_flow_MTU2(void);
void under_flow_MTU2(void);
void wait_interrupt(void);
float convert_radian(float degree);
float Limit_ul(float upper,float lower,float figure);
char Receive_uart_c(void);
char Receive_uart_c_SCI2(void);
float change_float(char *str);
void receive_order_depot(int target_box, char *storage_str, int minus_flag, int after_point_count, int large_size_flag);
void receive_order_c(char character);
void receive_att(void);
void transmission_string(char *s);
float get_motor_output_lf(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_rf(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_lb(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_rb(float motor_output_x,float motor_output_y,float degree_now);
void Deadtime_right_flont_tire(void);
void Deadtime_left_flont_tire(void);
void Deadtime_left_back_tire(void);
void Deadtime_right_back_tire(void);
void Move_right_flont_tire(float right_flont_duty);
void Move_left_flont_tire(float left_flont_duty);
void Move_left_back_tire(float left_back_duty);
void Move_right_back_tire(float right_back_duty);
void Move(float right_flont_duty,float left_flont_duty,float left_back_duty,float right_back_duty);
void input_R1350N(void);
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
void init_CMT0(void)
{
	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0;//CMT0のモジュールストップ状態を解除
	CMT.CMSTR0.BIT.STR0 = 0;//カウント停止
	
	CMT0.CMCR.BIT.CKS = 2;//クロック選択 PCLK/128
	CMT0.CMCOR = 375; //CMCORの決定 48000000/128/1000 = 375
	CMT0.CMCNT = 0;//初期化
	CMT0.CMCR.BIT.CMIE = 1;

	IEN(CMT0,CMI0) = 1;//割り込み要求許可レジスタ
	IPR(CMT0,CMI0) = 14;//優先度
}

/******************************************************************************
*	タイトル ： PWMの設定
*	  関数名 ： init_pwm
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/27
******************************************************************************/
void init_pwm(void)
{
	//MTU1,MTU2,,MTU7,MTU8は位相計数モードのためPWM設定不要
	
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
	
	
	MTUA.TSTR.BIT.CST4 = 0;			//カウント停止 
	MTUB.TSTR.BIT.CST0 = 0; 		//柳田追加12/22
	IOPORT.PFCMTU.BIT.MTUS4 = 1;		//ポートファンクションレジスタMTIOC4A-B,MTIOC4C-B端子を選択　追加
	SYSTEM.MSTPCRA.BIT.MSTPA8 = 0;		//MTUユニット1（MTU6～MTU11）のモジュールストップ解除
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;		//MTUユニット0（MTU0～MTU5）のモジュールストップ解除 追加柳田12/22
	MTU4.TCNT = 0;				//追加柳田12/22　
	MTU6.TCNT = 0;				//追加柳田12/22
	MTU4.TCR.BIT.CCLR = 0x01;		//追加柳田12/22
	MTU6.TCR.BIT.CCLR = 0x01;		//追加柳田12/22
	MTU4.TCR.BIT.CKEG = 0x00;		//追加柳田12/22
	MTU6.TCR.BIT.CKEG = 0x00;		//追加柳田12/22
	MTU4.TCR.BIT.TPSC = 0x00;//0x03;		//追加柳田12/22
	MTU6.TCR.BIT.TPSC = 0x00;//0x03;		//追加柳田12/22

	MTU4.TMDR.BIT.MD = 0x02;		//追加柳田12/22
	MTU6.TMDR.BIT.MD = 0x02;		//追加柳田12/22
//	MTUB.TOER.BIT.OE0B = 1;			//PWM出力許可 MTIOC10A
//	MTUB.TOER.BIT.OE0D = 1;			//PWM出力許可 MTIOC10C
	MTUA.TOER.BIT.OE4A = 1;
	MTUA.TOER.BIT.OE4C = 1;
	
	MTU4.TIORH.BIT.IOA = 2;			//追加柳田12/22
	MTU4.TIORH.BIT.IOB = 1;			//追加柳田12/22
	MTU6.TIORH.BIT.IOA = 2;			//追加柳田12/22
	MTU6.TIORH.BIT.IOB = 1;			//追加柳田12/22

	MTU4.TIORL.BIT.IOC = 2;			//追加柳田12/22
	MTU4.TIORL.BIT.IOD = 1;			//追加柳田12/22
	MTU6.TIORL.BIT.IOC = 2;			//追加柳田12/22
	MTU6.TIORL.BIT.IOD = 1;			//追加柳田12/22

	MTU4.TGRA = PWM_PERIOD;		//追加柳田12/22
	MTU4.TGRC = PWM_PERIOD;		//追加柳田12/22
	MTU6.TGRA = PWM_PERIOD;//_OTHER;		//追加柳田12/22
	MTU6.TGRC = PWM_PERIOD;//_OTHER;		//追加柳田12/22
	
	MTUA.TSTR.BIT.CST4 = 1;			//柳田追加12/22	//2013.02.18	CST3→CST4	MTU4のため3だと動かなかった
	MTUB.TSTR.BIT.CST0 = 1; 		//柳田追加12/22
}

/******************************************************************************
*	タイトル ： すべてのエンコーダ設定を行う
*	  関数名 ： init_all_encoder
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/14
******************************************************************************/
void init_all_encoder(void)
{
	PORTC.ICR.BIT.B6 = 1;	//MTCLKAの入力バッファを有効に
	PORTC.ICR.BIT.B7 = 1;	//MTCLKBの入力バッファを有効に
	PORTC.ICR.BIT.B4 = 1;	//MTCLKCの入力バッファを有効に
	PORTC.ICR.BIT.B5 = 1;	//MTCLKDの入力バッファを有効に
	PORTC.ICR.BIT.B2 = 1;	//MTCLKEの入力バッファを有効に
	PORTC.ICR.BIT.B3 = 1;	//MTCLKFの入力バッファを有効に
	PORTC.ICR.BIT.B0 = 1;	//MTCLKGの入力バッファを有効に
	PORTC.ICR.BIT.B1 = 1;	//MTCLKHの入力バッファを有効に

	IOPORT.PFCMTU.BIT.TCLKS = 1;	//PC6,7,4,5をMTCLKn-B端子として設定
	IOPORT.PFDMTU.BIT.TCLKS = 0;	//PC2,3,0,1をMTCLKn-A端子として設定

	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;
	SYSTEM.MSTPCRA.BIT.MSTPA8 = 0;

	MTUA.TSTR.BIT.CST1 = 0;//カウント動作停止
	MTUA.TSTR.BIT.CST2 = 0;//カウント動作停止
	MTUB.TSTR.BIT.CST1 = 0;//カウント動作停止
	MTUB.TSTR.BIT.CST2 = 0;//カウント動作停止

	MTU1.TMDR.BIT.MD = 4;//位相計数モード
	MTU2.TMDR.BIT.MD = 4;//位相計数モード
	MTU7.TMDR.BIT.MD = 4;//位相計数モード
	MTU8.TMDR.BIT.MD = 4;//位相計数モード

	MTU1.TCR.BIT.CCLR = 0;//TCNTのクリア禁止
	MTU2.TCR.BIT.CCLR = 0;//TCNTのクリア禁止
	MTU7.TCR.BIT.CCLR = 0;//TCNTのクリア禁止
	MTU8.TCR.BIT.CCLR = 0;//TCNTのクリア禁止

	MTUA.TSTR.BIT.CST1 = 1;//カウント動作開始
	MTUA.TSTR.BIT.CST2 = 1;//
	MTUB.TSTR.BIT.CST1 = 1;//
	MTUB.TSTR.BIT.CST2 = 1;//

	IPR(MTU1,TCIV1) = 0;			//割り込みの優先度を最低(割り込み禁止)にする
	IPR(MTU2,TCIV2) = 0;			//割り込みの優先度を最低(割り込み禁止)にする
	IPR(MTU7,TCIV7) = 0;			//割り込みの優先度を最低(割り込み禁止)にする
	IPR(MTU8,TCIV8) = 0;			//割り込みの優先度を最低(割り込み禁止)にする

	MTU1.TIER.BIT.TCIEV = 1;//オーバーフロー割り込みを許可する
	MTU1.TIER.BIT.TCIEU = 1;//アンダーフロー割り込みを許可する
	MTU2.TIER.BIT.TCIEV = 1;//オーバーフロー割り込みを許可する
	MTU2.TIER.BIT.TCIEU = 1;//アンダーフロー割り込みを許可する
	MTU7.TIER.BIT.TCIEV = 1;
	MTU7.TIER.BIT.TCIEU = 1;
	MTU8.TIER.BIT.TCIEV = 1;
	MTU8.TIER.BIT.TCIEU = 1;

	IEN(MTU1,TCIV1) = 1;//割り込み処理を許可
	IEN(MTU1,TCIU1) = 1;//割り込み処理を許可
	IPR(MTU1,TCIV1) = 15;//割り込み優先度を14

	IEN(MTU2,TCIV2) = 1;//割り込み処理を許可
	IEN(MTU2,TCIU2) = 1;//割り込み処理を許可
	IPR(MTU2,TCIV2) = 15;//割り込み優先度を14に

	IEN(MTU7,TCIV7) = 1;//割り込み処理を許可
	IEN(MTU7,TCIU7) = 1;//割り込み処理を許可
	IPR(MTU7,TCIV7) = 15;//割り込み優先度を14に

	IEN(MTU8,TCIV8) = 1;//割り込み処理を許可
	IEN(MTU8,TCIU8) = 1;//割り込み処理を許可
	IPR(MTU8,TCIV8) = 15;//割り込み優先度を14に
}

/******************************************************************************
*	タイトル ： sci通信初期化
*	  関数名 ： init_Sci
*	  戻り値 ： void型 
*	    引数 ： なし
******************************************************************************/
void init_Sci_0(void)
{
	int bit_count = 0;	
	
	SYSTEM.MSTPCRB.BIT.MSTPB31 = 0;	//SCI1モジュールSTOP状態を解除
	SCI0.SCR.BYTE		= 0x00;		//シリアルコントロールレジスタ
										//河原 0x01→0x00 で通信速度を最大に．分周1
	PORT2.DDR.BIT.B1	= 0;		//
	PORT2.ICR.BIT.B1	= 1;		//
	PORT2.DDR.BIT.B0 	= 0;	//追加
	PORT2.ICR.BIT.B0 	= 1;	//追加
	
	SCI0.SEMR.BIT.ABCS	= 1;		//調歩同期基本クロックを８サイクルの期間を１ビット期間の転送レートとする
	
	#if BITRATE_2 == 9600
		SCI0.SMR.BYTE		= 0x01;	
	#else
		SCI0.SMR.BYTE		= 0x00;	
	#endif
	
	SCI0.BRR = ((48*1000000)/((64/(1+SCI0.SEMR.BIT.ABCS))*powf(2,2*SCI0.SMR.BIT.CKS-1)*BITRATE_3)-1);
	
	for( bit_count = 0; bit_count < 0x800000; bit_count++ ){	//１ビット待つため
	}
	SCI0.SCR.BYTE		= 0x70;		//送受信動作を許可
	
	IEN(SCI0,RXI0) = 1;
	IPR(SCI0,RXI0) = 11;
}

void init_Sci_1(void)	//SCI1版
{
	
	int bit_count = 0;

	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;	//SCI1モジュールSTOP状態を解除
	
	SCI1.SCR.BYTE		= 0x00;		//シリアルコントロールレジスタ
	
	PORT3.DDR.BIT.B0	= 0;		//
	
	PORT3.ICR.BIT.B0	= 1;		//
	
	SCI1.SEMR.BIT.ABCS	= 1;		//調歩同期基本クロックを８サイクルの期間を１ビット期間の転送レートとする
	
	#if BITRATE_1 == 9600
		SCI1.SMR.BYTE		= 0x01;	
	#else
		SCI1.SMR.BYTE		= 0x00;	
	#endif
	
	SCI1.BRR = ((48*1000000)/((64/(1+SCI1.SEMR.BIT.ABCS))*powf(2,2*SCI1.SMR.BIT.CKS-1)*BITRATE_1)-1);	
	for( bit_count = 0; bit_count < 0x800000; bit_count++ ){	//１ビット待つため
	}
	
	SCI1.SCR.BYTE		= 0x30;		//送受信動作を許可

}

void init_Sci_2(void)//追加
{
	int bit_count = 0;	
	
	SYSTEM.MSTPCRB.BIT.MSTPB29 = 0;	//SCI1モジュールSTOP状態を解除
	SCI2.SCR.BYTE		= 0x00;		//シリアルコントロールレジスタ
										//河原 0x01→0x00 で通信速度を最大に．分周1
	PORT1.DDR.BIT.B2	= 0;		//
	PORT1.ICR.BIT.B2	= 1;		//
	PORT1.DDR.BIT.B3 	= 0;	//追加
	PORT1.ICR.BIT.B3 	= 1;	//追加
	
	//SCI2.BRR		= 12;		//ビットレートレジスタ77  9600bpsなら0x01の77
									//河原77→12 通信速度を230400bpsに設定
	SCI2.SEMR.BIT.ABCS	= 1;		//調歩同期基本クロックを８サイクルの期間を１ビット期間の転送レートとする
	
	#if BITRATE_2 == 9600
		SCI2.SMR.BYTE		= 0x01;	
	#else
		SCI2.SMR.BYTE		= 0x00;	
	#endif
	
	SCI2.BRR = ((48*1000000)/((64/(1+SCI2.SEMR.BIT.ABCS))*powf(2,2*SCI2.SMR.BIT.CKS-1)*BITRATE_2)-1);
	
	for( bit_count = 0; bit_count < 0x800000; bit_count++ ){	//１ビット待つため
	}
	SCI2.SCR.BYTE		= 0x70;		//送受信動作を許可
	
	IEN(SCI2,RXI2) = 1;
	IPR(SCI2,RXI2) = 11;
}
void All_setup(void)
{
	init_clock();
	init_CMT0();
	init_pwm();
	init_all_encoder();
	init_Sci_0();
	init_Sci_1();
	init_Sci_2();
	
	INTERRUPT_START//割り込みタイマ開始
}

/******************************************************************************
*	タイトル ： flow_count
*	  関数名 ： flow_count
*	  戻り値 ： void型 オーバーフロー　アンダーフローのカウント
*	    引数 ： なし
*	  作成者 ： 石井
*	  作成日 ： 2014/07/17
******************************************************************************/

void over_flow_MTU1(){
	g_over_vertical_count += 1;
}

void under_flow_MTU1(){
	g_under_vertical_count += 1;
}

void over_flow_MTU2(){
	g_over_horizontal_count += 1;
}

void under_flow_MTU2(){
	g_under_horizontal_count += 1;
}

void wait_interrupt(void)
{
	IR(CMT0,CMI0) = OFF;
	
	g_interrupt_timer_count ++;
	g_interrupt_timer_count2 ++;
	g_right_flont_motor_timer_count ++;
	g_right_back_motor_timer_count ++;
	g_left_flont_motor_timer_count ++;
	g_left_back_motor_timer_count ++;
}

/******************************************************************************
*	タイトル ： 度数のラジアン変換
*	  関数名 ： convert_radian
*	  戻り値 ： float型
*	    引数1： float型 degree 
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
float convert_radian(float degree)
{
	float radian = 0.0;
	while(degree > 180){
		degree -= 360;
	}
	while(degree < -180){
		degree += 360;
	}
	radian = degree * (PI / 180);
	return (radian);
}

/******************************************************************************
*	タイトル ： 設定した範囲内の値を返す
*	  関数名 ： Limit_ul
*	  戻り値 ： float型 出力値
*	   引数1 ： float型 upper  上限の数値
*	   引数2 ： float型 lower  下限の数値
*	   引数3 ： float型 figure  比較する数値
*	  作成者 ： 市川 智章
*	  作成日 ： 2011/08/31
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
*	タイトル ：p12,p13で得た受信データを格納し返す
*	  関数名 ： Receive_uart_c
*	  戻り値 ： char型
*	    引数： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2014/01/22
******************************************************************************/
char Receive_uart_c(void)
{
	if (SCI0.SSR.BIT.RDRF == 0);		//RDRF = 0：SCRDR に有効な受信データが格納されていないことを表示
	SCI0.SSR.BIT.RDRF = 0;				//RDRFを待機状態に変更	
	return SCI0.RDR;
}

/******************************************************************************
*	タイトル ：p12,p13で得た受信データを格納し返す
*	  関数名 ： Receive_uart_c
*	  戻り値 ： char型
*	    引数： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2014/01/22
******************************************************************************/
char Receive_uart_c_SCI2(void)
{
	if (SCI2.SSR.BIT.RDRF == 0);		//RDRF = 0：SCRDR に有効な受信データが格納されていないことを表示
	SCI2.SSR.BIT.RDRF = 0;				//RDRFを待機状態に変更	
	return SCI2.RDR;
}

//文字列をfloat型に変換	この関数はロボティクスのマイコンの授業で配られたやつですね
float change_float(char *str)
{
    float n = 0;
    int i = 0;
    while(str[i]!='\0')
    {
        if(str[i]<'0' || str[i]>'9') break;
        n=n*10+str[i]-'0';
        i++;
    }
    return(n);
}

//解析した命令に応じて数値をグローバル変数に格納する関数	下の関数の続きみたいな
void receive_order_depot(int target_box, char *storage_str, int minus_flag, int after_point_count, int large_size_flag)
{    
    float value = 0.000;
    
    value = change_float(storage_str);

    if(minus_flag == 1){
        value *= ( -1.000 );
    }
    
    value = value * pow( 0.100, after_point_count );

    if( (target_box >= 0) && (target_box <= 25) ){
        if(large_size_flag == 0){
            g_atoz_value[target_box] = value;
        }else{
            g_AtoZ_value[target_box] = value;
        }
    }
}

//一文字ごとに解析する関数	いーのくんの力作
void receive_order_c(char character)
{
    static int target_box = 255;						 //格納命令の開始文字(ASCIIコードのa～z,A～Z)
    static char storage_str[RECEIVE_STR_COLUMN] = "";	 //文字の格納用の文字列
    static int storage_num = 0;							 //文字を文字列のどこに格納するか
    static int minus_flag = 0;							 //マイナス値か否か
    static int point_flag = 0;							 //小数点以下が含まれているか否か
    static int after_point_count = 0;					 //小数点以下にどれだけ数があるか
    static int large_size_flag = 0;						 //大文字なのか否か
    int reset = 0;										 //処理のリセットをするか否か
	const char end = END;								 //格納命令の終了文字
    
    if(character == end){
            storage_str[storage_num] = '\0';
            receive_order_depot(target_box, storage_str, minus_flag, after_point_count, large_size_flag);
            reset = 1;
            target_box = 255;
            large_size_flag = 0;
    }else{
        if( (character >= '0') && (character <= '9') ){
            storage_str[storage_num] = character;
            storage_num++;  
            if( point_flag == 1 ){
                after_point_count++;
            }
        }else if( (character >= 'a') && (character <= 'z') ){
            reset = 1;
            target_box = (int)(character - 'a');
            large_size_flag = 0;
        }else if( (character >= 'A') && (character <= 'Z') ){
            reset = 1;
            target_box = (int)(character - 'A');
            large_size_flag = 1;
        }else if( character == '-' ){
            minus_flag = 1;
        }else if( character == '.' ){
            point_flag = 1;
        }
    }
    
    if( reset == 1 ){
       strcpy(storage_str,"");
       storage_num = 0;
       minus_flag = 0;
       point_flag = 0;
       after_point_count = 0;
    }    
}

//通信相手からの受信割り込み	ここはマイコンによって異なる
void receive_att(void)
{    
    char c;
    IR(SCI0,RXI0) = 0;
    PORT8.DR.BIT.B1 = 1;
	c = Receive_uart_c();//受信データ
    receive_order_c(c);
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

/******************************************************************************
*	タイトル ： 左前オムニタイヤの出力決定
*	  関数名 ： get_motor_output_lf
*	  戻り値 ： float型
*	    引数1 ：float型 motor_output_x
*	    引数2 ：float型 motor_output_y
*	    引数3 ：float型 degree_now
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/11/21
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
*	タイトル ： 右前オムニタイヤの出力決定
*	  関数名 ： get_motor_output_rf
*	  戻り値 ： float型
*	    引数1 ：float型 motor_output_x
*	    引数2 ：float型 motor_output_y
*	    引数3 ：float型 degree_now
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/11/21
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
*	タイトル ： 左後オムニタイヤの出力決定
*	  関数名 ： get_motor_output_lb
*	  戻り値 ： float型
*	    引数1 ：float型 motor_output_x
*	    引数2 ：float型 motor_output_y
*	    引数3 ：float型 degree_now
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/11/21
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
*	タイトル ： 右後オムニタイヤの出力決定
*	  関数名 ： get_motor_output_rb
*	  戻り値 ： float型
*	    引数1 ：float型 motor_output_x
*	    引数2 ：float型 motor_output_y
*	    引数3 ：float型 degree_now
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/11/21
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
*	タイトル ： 右前タイヤの出力リセット
*	  関数名 ： Deadtime_right_flont_tire
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Deadtime_right_flont_tire(void)
{
	RIGHT_FLONT_CW = OFF;
	RIGHT_FLONT_CCW = OFF;
	RIGHT_FLONT_DUTY = OFF;
}

/******************************************************************************
*	タイトル ： 右前タイヤの動作
*	  関数名 ： Move_right_flont_tire
*	  戻り値 ： void型
*	    引数1： float型 right_duty 
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Move_right_flont_tire(float right_flont_duty)
{
	static int i = 0;
	static float right_flont_duty_old = 0.0;

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
}

/******************************************************************************
*	タイトル ： 左前タイヤの出力リセット
*	  関数名 ： Deadtime_left_tire
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Deadtime_left_flont_tire(void)
{
	LEFT_FLONT_CW = OFF;
	LEFT_FLONT_CCW = OFF;
	LEFT_FLONT_DUTY = OFF;
}

/******************************************************************************
*	タイトル ： 左前タイヤ動作
*	  関数名 ： Move_left_tire
*	  戻り値 ： void型
*	    引数1： float型 left_duty 
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Move_left_flont_tire(float left_flont_duty)
{
	static int i = 0;
	static float left_flont_duty_old = 0.0;
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
}

/******************************************************************************
*	タイトル ： 左後タイヤの出力リセット
*	  関数名 ： Deadtime_left_tire
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Deadtime_left_back_tire(void)
{
	LEFT_BACK_CW = OFF;
	LEFT_BACK_CCW = OFF;
	LEFT_BACK_DUTY = OFF;
}

/******************************************************************************
*	タイトル ： 左後タイヤ動作
*	  関数名 ： Move_left_back_tire
*	  戻り値 ： void型
*	    引数1： float型 left_back_duty 
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
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
*	タイトル ： 右後タイヤの出力リセット
*	  関数名 ： Deadtime_right_back_tire
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Deadtime_right_back_tire(void)
{
	RIGHT_BACK_CW = OFF;
	RIGHT_BACK_CCW = OFF;
	RIGHT_BACK_DUTY = OFF;
}

/******************************************************************************
*	タイトル ： 右後タイヤの動作
*	  関数名 ： Move_right_back_tire
*	  戻り値 ： void型
*	    引数1： float型 right_back_duty 
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
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
*	タイトル ： モータ関数
*	  関数名 ： Move
*	  戻り値 ： void型
*	    引数1： float型 left_duty
*	    引数2： float型 right_duty
*	    引数3： float型 back_duty
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Move( float right_flont_duty, float left_flont_duty, float left_back_duty, float right_back_duty)
{
	Move_left_flont_tire(left_flont_duty);
	Move_right_flont_tire(right_flont_duty);
	Move_left_back_tire(left_back_duty);
	Move_right_back_tire(right_back_duty);
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
	static int read_start = OFF;
	unsigned char index;
	unsigned int angle;
	float angle_f;
	static float start_Rate	= 0;
	unsigned int rate;
	float rate_f;
	float now_Rate;
	unsigned int x_acc;
	unsigned int y_acc;
	unsigned int z_acc;
	unsigned int reserved;
	unsigned char check_sum;
	char string[20] = {0};
	static int start_flug	= 0;
	
	
	g_input_r1350n[i] = Receive_uart_c_SCI2();	
	
	//HEADER値発見
	if(g_input_r1350n[0] == 0xAA){
		read_start = ON;
	}
	
	if(read_start == ON){
		i++;
		//0～14までで1セットの文字列
		if(i >= 15){
			i = 0;
			read_start = OFF;

			//パケットのヘッダー情報を確認する
			if(g_input_r1350n[0] != 0xAA){
				sprintf(string, "Heading ERROR");
				transmission_string(string);
			}
			
			//データを組み立てる
			index = g_input_r1350n[2];
			rate = (g_input_r1350n[3] & 0xFF) | ((g_input_r1350n[4] << 8) & 0xFF00);
			angle = (g_input_r1350n[5] & 0xFF) | ((g_input_r1350n[6] << 8) & 0XFF00);
			rate_f = (g_input_r1350n[3] & 0xFF) | ((g_input_r1350n[4] << 8) & 0xFF00);
			angle_f = (g_input_r1350n[5] & 0xFF) | ((g_input_r1350n[6] << 8) & 0XFF00);
			x_acc = (g_input_r1350n[7] & 0xFF) | ((g_input_r1350n[8] << 8) & 0xFF00);
			y_acc = (g_input_r1350n[9] & 0xFF) | ((g_input_r1350n[10] << 8) & 0XFF00);
			z_acc = (g_input_r1350n[11] & 0xFF) | ((g_input_r1350n[12] << 8) & 0xFF00);
			reserved = g_input_r1350n[13];
			
			//チェックサムの確認
			check_sum = 	g_input_r1350n[2] + g_input_r1350n[3] + g_input_r1350n[4] + g_input_r1350n[5]
					     + g_input_r1350n[6] + g_input_r1350n[7] + g_input_r1350n[8] + g_input_r1350n[9]
					     + g_input_r1350n[10] + g_input_r1350n[11] + g_input_r1350n[12] + g_input_r1350n[13];
			
			if(check_sum != g_input_r1350n[14]){
				sprintf(string, "Check_Sum ERROR");
				transmission_string(string);
			}
			if( start_flug == 0 ){
				start_flug = 1;
				start_Rate = rate / 100.0;
				if(start_Rate > 179){
					start_Rate = start_Rate - 655.0;
				}
				while(start_Rate > 179){
					start_Rate -= 360;
				}
				while(start_Rate < -179){
					start_Rate += 360;
				}
			}
			
			//角度と角速度の単位を通常値（元に戻しデータを記憶する
			g_Rate = rate / 100;
			g_Angle = angle / 100;
			g_Rate_f = rate / 100.0;
			g_Angle_f = angle / 100.0;
			g_X_acc = x_acc;
			g_Y_acc = y_acc;
			g_Z_acc = z_acc;
			
			if(g_Rate > 179){
				g_Rate = g_Rate - 655;
			}
			
			if(g_Angle > 179){
				g_Angle = g_Angle - 655;
			}
			
			if(g_Rate_f > 179){
				g_Rate_f = g_Rate_f - 655.0;
			}
			while(g_Rate_f > 179){
				g_Rate_f -= 360;
			}
			while(g_Rate_f < -179){
				g_Rate_f += 360;
			}
			
			g_Rate_f -= start_Rate;
			
			if(g_Rate_f > 179){
				g_Rate_f = g_Rate_f - 655.0;
			}
			while(g_Rate_f > 179){
				g_Rate_f -= 360;
			}
			while(g_Rate_f < -179){
				g_Rate_f += 360;
			}
			
			if(g_Angle_f > 179){
				g_Angle_f = g_Angle_f - 655.0;
			}
//				sprintf(string, "%d\n\r",rate);
//				transmission_string(string);
		}
	}
}

float straight_output_x( void ){
	
	float straight_cal_x = 0.00;
	
	straight_cal_x = ( 255.0 - (float)LEFT_STICK_HIGH ) / 255.0;
	straight_cal_x = ( straight_cal_x - 0.5 ) * 2;
	if( fabs( straight_cal_x ) <= STICK_NO_MOVE_RANGE ){
		straight_cal_x = 0;
	}
	straight_cal_x *= PWM_PER;
	
	return(straight_cal_x);
}

float straight_output_y( void ){
	
	float straight_cal_y = 0.00;
	
	straight_cal_y = ( 255.0 - (float)LEFT_STICK_WIDE ) / 255.0;
	straight_cal_y = ( straight_cal_y - 0.5 ) * 2;
	if( fabs( straight_cal_y ) <= STICK_NO_MOVE_RANGE ){
		straight_cal_y = 0;
	}
	straight_cal_y *= PWM_PER;
	
	return(straight_cal_y);
}


float turn_output( void ){
	
	float turn_cal = 0.00;
	
	turn_cal = ( 255.0 - (float)RIGHT_STICK_WIDE ) / 255.0;
	turn_cal = ( turn_cal - 0.5 ) * 2;
	if( fabs( turn_cal ) <= STICK_NO_MOVE_RANGE ){
		turn_cal = 0;
	}
	turn_cal *= ( OPERATE_DEGREE / ( 1.00 / ( INTERRUPT_TIME / 1000.0 ) ));
	
	return(turn_cal);
}

//PD回転制御
float Turn_PD( float target_degree , float now_degree )
{	
	float output = 0.00;
	float difference_degree = 0.00;
	static float old_difference_degree = 0.00;
	
	difference_degree = target_degree - now_degree;

	if ( difference_degree > 180 ){
		difference_degree = -360 + difference_degree;
	}else if ( difference_degree < -180 ){
		difference_degree = difference_degree + 360;
	}
	output = ( TURN_P_GAIN * difference_degree ) + ( TURN_D_GAIN * ( difference_degree - old_difference_degree ));
	
	output = Limit_ul( 50, -50 , output);
	
	old_difference_degree = difference_degree;
	
	return output;
}	


int main(void)
{
	int 	vertical_enc_count = 0,
		horizontal_enc_count = 0,
		old_vertical_enc_count	= 0,
		old_horizontal_enc_count	= 0,
		i = 0;
	float	add_distance_vertical	= 0,
		add_distance_horizontal	= 0,
		old_distance_vertical	= 0,
		old_distance_horizontal = 0,
		add_distance			= 0,
		add_distance_degree			= 0,
		add_degree		= 0,
		degree_reverse_v	= 0,
		degree_reverse_h	= 0,
		velocity[50] = { 0.00 };
	
	float	Motor_output_x = 0,
		Motor_output_y = 0,
		Motor_output_turn = 0.00,
		nutral_x = 127,
		nutral_y = 127,
		turn_pwm_percent = 10,
		old_accel1 = 0,
		old_accel2 = 0,
		old_accel3 = 0,
		old_degree = 0,
		degree = 0,
		target_degree = 0;
		
	float	manual_motor_output_rf = 0.00,
		manual_motor_output_lf = 0.00,
		manual_motor_output_lb = 0.00,
		manual_motor_output_rb = 0.00;
		
	int stop_flug_count = 0;
		
	char string[100] = { 0 },
		string2[300] = { 0 };	
	
	All_setup();

	PORT8.DDR.BIT.B1 = 1;
	PORTD.DDR.BIT.B7 = 1;
	
	while(1){
		if(g_interrupt_timer_count >= INTERRUPT_TIME){
			g_interrupt_timer_count = 0;
			if( fabs( -g_Rate_f - old_degree ) < 30 || fabs( -g_Rate_f - old_degree ) > 330){
				degree = -g_Rate_f;
			}
			vertical_enc_count = VERTICAL_ENCODER  + ( 65536 * g_over_vertical_count)  + ( ( -65536 ) * g_under_vertical_count  ); //垂直エンコーダーの値
			horizontal_enc_count = HORIZONTAL_ENCODER  + ( 65536 * g_over_horizontal_count)  + ( ( -65536 ) * g_under_horizontal_count  ); //水平エンコーダーの値				
			
			add_distance_vertical = ( ( vertical_enc_count - old_vertical_enc_count ) * PI * DIAMETER_VERTICAL_WHEEL ) / ( PULSE_VERTICAL_ENCODER * 4 );
			add_distance_horizontal = ( ( horizontal_enc_count - old_horizontal_enc_count ) * PI * DIAMETER_HORIZONTAL_WHEEL ) / ( PULSE_HORIZONTAL_ENCODER * 4 );
			add_distance = pow(add_distance_vertical * add_distance_vertical + add_distance_horizontal * add_distance_horizontal,0.5);
			
			if(add_distance_horizontal != 0 || add_distance_vertical != 0 ){
				add_distance_degree = atan2( add_distance_horizontal , add_distance_vertical ) * 180 / PI;
			
//				g_x_coordnates += ( fabs( add_distance_vertical ) * cos( convert_radian( -g_Rate_f + degree_reverse_v) ) + fabs( add_distance_horizontal ) * sin( convert_radian( -g_Rate_f + degree_reverse_h) ));
//				g_y_coordnates += ( fabs( add_distance_vertical ) * sin( convert_radian( -g_Rate_f + degree_reverse_v) ) + fabs( add_distance_horizontal ) * cos( convert_radian( -g_Rate_f + degree_reverse_h) ));
//				sprintf(string, "%f  ,%f  ,%f  \n\r",degree,g_x_coordnates,g_y_coordnates);
//				transmission_string(string);
				g_x_coordnates += add_distance * cos( convert_radian( add_distance_degree + degree  )); 
				g_y_coordnates += add_distance * sin( convert_radian( add_distance_degree + degree  ));
			}
			if( g_interrupt_timer_count3 >= INTERRUPT_TIME * 100 ){
				g_interrupt_timer_count3 = 0;
				velocity[i] = add_distance / ( INTERRUPT_TIME / 1000 );
				i++;
			}
			Motor_output_x = straight_output_x();
			Motor_output_y = straight_output_y();
			target_degree += turn_output();
			
			while( target_degree > 180 ){
				target_degree -= 360;
			}
			while( target_degree < -180 ){
				target_degree += 360;
			}
			Motor_output_turn = Turn_PD( target_degree , degree );
			
			manual_motor_output_lf = get_motor_output_lf( Motor_output_x, Motor_output_y, 0.0 );
			manual_motor_output_rf = get_motor_output_rf( Motor_output_x, Motor_output_y, 0.0 );
			manual_motor_output_lb = get_motor_output_lb( Motor_output_x, Motor_output_y, 0.0 );
			manual_motor_output_rb = get_motor_output_rb( Motor_output_x, Motor_output_y, 0.0 );

//				sprintf(string,"%c,%f,%f,%f,%f\n\r",getdate1.byte.model_number,manual_motor_output_rf , manual_motor_output_lf , manual_motor_output_lb, manual_motor_output_rb);
//				sprintf(string,"%f  %f  %f  %f  %f  %f\n\r",Motor_output_x,Motor_output_y,target_degree ,LEFT_STICK_HIGH,LEFT_STICK_WIDE,RIGHT_STICK_WIDE);
//				transmission_string(string);
//				String("aaa");

			if(old_accel1 == g_atoz_value[13] && old_accel2 == g_atoz_value[14] && old_accel3 == g_atoz_value[16]){
				stop_flug_count ++;
			}else{
				stop_flug_count = 0;
			}

			old_accel1 = g_atoz_value[13];
			old_accel2 = g_atoz_value[14];
			old_accel3 = g_atoz_value[16];
			
			old_vertical_enc_count = vertical_enc_count;
			old_horizontal_enc_count = horizontal_enc_count;
			old_degree = degree;
			
			if(g_interrupt_timer_count2 >= INTERRUPT_TIME * 10 ){
				g_interrupt_timer_count2 = 0;
//					sprintf(string, "%f  ,   %f,   %f  ,%f\n\r",degree,add_distance_degree,g_x_coordnates,g_y_coordnates);
//					sprintf(string, "%f  ,%f  ,%f,%d  ,%d \n\r",old_degree,g_x_coordnates,g_y_coordnates,add_distance_vertical,add_distance_horizontal);					
//				sprintf(string,"%d\n\r",horizontal_enc_count);
//				sprintf(string,"%f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f  %f\n\r",g_atoz_value[0],g_AtoZ_value[0],g_atoz_value[1],g_AtoZ_value[1],g_atoz_value[3],g_atoz_value[4],g_atoz_value[5],g_atoz_value[6],g_atoz_value[7],g_atoz_value[8],g_atoz_value[9],g_atoz_value[10],g_atoz_value[11],g_atoz_value[12],g_atoz_value[13],g_atoz_value[14] );
//				sprintf(string,"%f  %f  %f\n\r",target_degree,degree,Motor_output_turn); 
//				sprintf(string,"%f  %f  %f\n\r",Motor_output_x,Motor_output_y,target_degree);
				sprintf(string2,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,\n\r",velocity[0],velocity[1],velocity[2],velocity[3],velocity[4],velocity[5],velocity[6],velocity[7],velocity[8],velocity[9],velocity[10],velocity[11],velocity[12],velocity[13],velocity[14],velocity[15],velocity[16],velocity[17],velocity[18],velocity[19],velocity[20],velocity[21],velocity[22],velocity[23],velocity[24],velocity[25],velocity[26],velocity[27],velocity[28],velocity[29],velocity[30],velocity[31],velocity[32],velocity[33],velocity[34],velocity[35],velocity[36],velocity[37],velocity[38],velocity[39]);
				transmission_string(string2);
			}			

			if(stop_flug_count < 120 ){
				PORTD.DR.BIT.B7 = 0;
				Move( manual_motor_output_rf + Motor_output_turn , manual_motor_output_lf + Motor_output_turn, manual_motor_output_lb + Motor_output_turn, manual_motor_output_rb + Motor_output_turn); 
			}
				
			if(g_atoz_value[8] == 1 || stop_flug_count >= 120){
				if(stop_flug_count >= 120 ){
					PORTD.DR.BIT.B7 = 1;
				}
				LEFT_FLONT_CW = 0;
				LEFT_FLONT_CCW = 0;
				LEFT_BACK_CW = 0;
				LEFT_BACK_CCW = 0;
				RIGHT_FLONT_CW = 0;
				RIGHT_FLONT_CCW = 0;
				RIGHT_BACK_CW = 0;
				RIGHT_BACK_CCW = 0;
				Move( 0,0,0,0);
			}
		}
	}
}