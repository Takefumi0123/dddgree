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
#define INTERRUPT_START		CMT.CMSTR0.BIT.STR0 = 1;//カウント開始
#define INTERRUPT_TIME 			10
#define STICK_NO_MOVE_RANGE	64.0		//足回りが動かないスティックの値の範囲
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
volatile unsigned long	g_controller_receive_1st	= 0;	//コントローラから帰ってくるデータの格納フォルダ1stは不定でよい。
volatile unsigned long	g_controller_receive_2nd	= 0;
volatile unsigned long	g_controller_receive_3rd	= 0;
int 	getdate1 = 0, getdate2 = 0, getdate3 = 0;

void init_clock(void)
{	
	SYSTEM.SCKCR.BIT.PCK = 1; //×4 ICKとは関係ない場所の速さ
	SYSTEM.SCKCR.BIT.ICK = 0; //×8 マイコンそのものの速さ	
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
	radian = degree * (M_PI / 180);
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
*	タイトル ：D　U　A　L　S　H　O　C　K
*	  関数名 ： init_Rspi_dualshock
*	  戻り値 ： void型 
*	   引数1 ： char型 s[]  
******************************************************************************/
void init_Rspi_dualshock(void)	//(デュアルショック用)
{
	MSTP(RSPI1)			= 0;	//RSPI1モジュールストップの解除
	
	RSPI1.SPCR.BYTE		= 0x00;	//始めにRSPI通信を使用するために0x00で通信を有効に
	
	RSPI1.SPPCR.BIT.SPLP	= 0;	//RSPIループバックビルド=通常モード
	RSPI1.SPPCR.BIT.SPLP2	= 0;	//RSPI2スープバックビルド=通常モード
	RSPI1.SPPCR.BIT.SPOM	= 0;	//RSPI出力端子モードビット=CMOS出力
	RSPI1.SPPCR.BIT.MOIFV	= 1;	//MOSIアイドル固定値ビット1
	RSPI1.SPPCR.BIT.MOIFE	= 1;	//MOSI出力値はMOIFVビットの設定値
	
	RSPI1.SPBR			= 75;	//RSPIビットレートレジスタ=255最低速度

	RSPI1.SPDCR.BIT.SPFC	= 0x00;	//SPDRレジスタに格納できるフレーム数を１にする
	RSPI1.SPDCR.BIT.SLSEL	= 0x00;	//SSL端子出力設定余ったポートをIOポートにする=すべて出力用
	RSPI1.SPDCR.BIT.SPRDTD	= 0;	//RSPI受信/送信データ選択ビット=SPDRは受信バッファを読みだす
	RSPI1.SPDCR.BIT.SPLW	= 1;	//SPDRレジスタへはロングワードアクセス。

	RSPI1.SPSCR.BIT.SPSLN	= 0x02;	//RSPIシーケンス長設定ビット=シーケンス長3
	
	RSPI1.SPCKD.BIT.SCKDL	= 0x00;	//RSPCK遅延設定ビット=1RSPCK 
	
	RSPI1.SSLND.BIT.SLNDL	= 0x00;	//SSLネゲート遅延設定ビット=1RSPCK
	
	RSPI1.SPND.BIT.SPNDL	= 0x00;	//RSPI次アクセス遅延設定ビット=1RSPCK+2PCLK
	
	RSPI1.SPCR2.BIT.SPPE	= 0;	//パリティ有効ビット、送信データのパリティビットを付加しない
	RSPI1.SPCR2.BIT.SPOE	= 0;	//パリティモードビット=偶数パリティイで送受信
	RSPI1.SPCR2.BIT.SPIIE	= 1;	//アイドル割り込み要求の発生を許可
	RSPI1.SPCR2.BIT.PTE		= 0;	//パリティ回路自己診断機能は無効
	
	RSPI1.SPCMD0.BIT.CPHA	= 1;	//奇数エッジでデータ変化、偶数エッジでデータサンプル
	RSPI1.SPCMD0.BIT.CPOL	= 1;	//アイドル時のRSPCKが'1'
	RSPI1.SPCMD0.BIT.BRDV	= 0x03;	//ベースのビットレートを8分周を選択
	RSPI1.SPCMD0.BIT.SSLA	= 0x00;	//SSL信号アサート設定ビット=SSLO
	RSPI1.SPCMD0.BIT.SSLKP	= 1;	//転送終了後から次アクセス開始までSSL信号レベルを保持

	RSPI1.SPCMD0.BIT.SPB		= 0x03;	//RSPIデータ長設定ビット=32ビット
	RSPI1.SPCMD0.BIT.LSBF		= 1;		//RSPILSBファーストビット=LSBファーストビット
	RSPI1.SPCMD0.BIT.SPNDEN	= 1;		//次アクセス遅延はRSPI次アクセス遅延レジスタ(SPND)の設定値
	RSPI1.SPCMD0.BIT.SLNDEN	= 1;		//次アクセス遅延設定許可ビット=SSLネゲート遅延はRSPIスレーブセレクトネゲート遅延レジスタ(SSLND)の設定値
	RSPI1.SPCMD0.BIT.SCKDEN	= 1;		//RSPCK遅延はRSPCK遅延はRSPIクロック遅延レジスタ(SPCKD)の設定値
	
	RSPI1.SPCMD1.WORD = RSPI1.SPCMD0.WORD;	//4バイト毎の送信時に行う設定をコピーさせる
	RSPI1.SPCMD2.WORD = RSPI1.SPCMD0.WORD;	//1バイト毎の送信時に行う設定をコピーさせる
	RSPI1.SPCMD2.BIT.SPB	= 0x07;			//RSPIデータ長設定ビット=8ビット

	RSPI1.SPCMD2.BIT.SSLKP	= 0;			//送信が終わった際に出力をHighにするため

	//割り込みコントローラの設定
	//DMACAの設定
	//入出力ポートの設定	(今回はシングルマスタ設定のため入出力が自動で決定される)
	PORTE.ICR.BIT.B7 = 1;
	
	IOPORT.PFHSPI.BIT.RSPIS = 1;

	IOPORT.PFHSPI.BIT.RSPCKE	= 1;	//RSPCKB端子有効
	IOPORT.PFHSPI.BIT.MOSIE		= 1;	//MOSIB端子有効
	IOPORT.PFHSPI.BIT.MISOE		= 1;	//MISOB端子有効
	IOPORT.PFHSPI.BIT.SSL0E		= 1;	//SSLB0端子有効
	IOPORT.PFHSPI.BIT.SSL1E		= 1;	//SSLB1端子有効
	IOPORT.PFHSPI.BIT.SSL2E		= 1;	//SSLB2端子有効
	IOPORT.PFHSPI.BIT.SSL3E		= 1;	//SSLB3端子有効
	
	RSPI1.SPCR.BIT.SPMS		= 0;	//RSPIモード選択ビット=SPI動作(4線式)
	RSPI1.SPCR.BIT.TXMD 	= 0;	//通信動作モード選択ビット=全二重同期式シリアル通信
	RSPI1.SPCR.BIT.MODFEN	= 0;	//モードフォルトエラー検出を禁止
	RSPI1.SPCR.BIT.MSTR		= 1;	//RSPIマスタ/スレーブモード選択=マスタモード
	RSPI1.SPCR.BIT.SPEIE	= 0;	//RSPIエラー割り込み要求の発生を禁止
	RSPI1.SPCR.BIT.SPTIE	= 0;	//RSPI送信割り込み要求の発生を禁止
	RSPI1.SPCR.BIT.SPE		= 1;	//RSPI機能を有効に
	RSPI1.SPCR.BIT.SPRIE	= 1;	//RSPI受信割り込み要求の発生を許可
	
	RSPI1.SSLP.BIT.SSLP0	= 0;	//SSL0信号は0アクティブ
	RSPI1.SSLP.BIT.SSLP1	= 0;	//SSL1信号は0アクティブ
	RSPI1.SSLP.BIT.SSLP2	= 0;	//SSL2信号は0アクティブ
	RSPI1.SSLP.BIT.SSLP3	= 0;	//SSL3信号は0アクティブ
	
	//以下不要の場合は削除の事
	RSPI1.SPSR.BIT.OVRF		= 0;	//オーバランエラーなし
	RSPI1.SPSR.BIT.IDLNF	= 0;	//RSPIがアイドル状態(後で送るときだけ1を代入するのだろうか)<武山のでは使用されていなかった>？
	RSPI1.SPSR.BIT.MODF	= 0;	//モードフォルトエラーなし
	RSPI1.SPSR.BIT.PERF		= 0;	//パリティエラーなし

	//以上不要の場合は削除の事
	RSPI1.SPCR.BYTE;	//SPCRのリード
	IEN(RSPI1,SPRI1) = 1;
	IPR(RSPI1,SPRI1) = 12;
}

/******************************************************************************
*	タイトル ： SPI通信で受信したものを返す
*	  関数名 ： Rspi_send_1
*	  戻り値 ：unsigned lond型
*	    引数 ： unsingned long moji
******************************************************************************/
unsigned long Rspi_send_1(unsigned long moji)
{
	RSPI1.SPDR.LONG = moji;
	while( RSPI1.SPSR.BIT.SPRF == 0 );	//受信バッファになにか来るまで待つ
	return RSPI1.SPDR.LONG;
}
/******************************************************************************
*	タイトル ： SPI通信で受信したものを返す
*	  関数名 ： Rspi_send_1
*	  戻り値 ：unsigned lond型
*	    引数 ： unsingned long moji
******************************************************************************/
unsigned long Rspi_send_short_1(unsigned short int moji)
{
	RSPI1.SPDR.LONG = moji;
	while( RSPI1.SPSR.BIT.SPRF == 0 );	//受信バッファになにか来るまで待つ
	return RSPI1.SPDR.LONG;
}
/******************************************************************************
*	タイトル ： デュアルショックからの送信データを格納
*	  関数名 ： Rspi_receive_send_line_dualshock
*	  戻り値 ： void型
*	    引数 ： なし
******************************************************************************/
void Rspi_recive_send_line_dualshock(void)	//DualShockアナログコントローラ(アナログモード緑LED)用送信プログラム
{
	while( RSPI1.SPSR.BIT.SPRF == 1 ){				//受信バッファがフルならリードしてクリアする
		RSPI1.SPDR.LONG;
	}	
	g_controller_receive_1st = Rspi_send_1(0x00004201);
	g_controller_receive_2nd = Rspi_send_1(0x00000000);
	g_controller_receive_3rd = Rspi_send_short_1(0x00);
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

void init_CMT0(void)
{
	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0;//CMT0のモジュールストップ状態を解除
	CMT.CMSTR0.BIT.STR0 = 0;//カウント停止
	
	CMT0.CMCR.BIT.CKS = 2;//PCLK/128
	CMT0.CMCOR = 375;//48000000/128/1000 = 375
	CMT0.CMCNT = 0;
	CMT0.CMCR.BIT.CMIE = 1;
	IEN(CMT0,CMI0) = 1;//割り込み許可
	IPR(CMT0,CMI0) = 14;//割り込み優先度を最大に
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
*	タイトル ： sci通信初期化
*	  関数名 ： init_Sci
*	  戻り値 ： void型 
*	    引数 ： なし
******************************************************************************/
void init_Sci(void)	//SCI1版
{
	
	int bit_count = 0;

	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;	//SCI1モジュールSTOP状態を解除
	
	SCI1.SCR.BYTE		= 0x00;		//シリアルコントロールレジスタ
	
	PORT3.DDR.BIT.B0	= 0;		//
	
	PORT3.ICR.BIT.B0	= 1;		//
	
	/*SCI1.SMR.BYTE		= 0x00;		//シリアルモードレジスタ

	SCI1.BRR			= 25;		//ビットレートレジスタ 
								//9600bps：シリアルモードレジスタ→0x01で77
								//115200bps：25
								//230400bps:21
	
	SCI1.SEMR.BIT.ABCS	= 1;		//調歩同期基本クロックを８サイクルの期間を１ビット期間の転送レートとする
	*/
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
	
	INTERRUPT_START//割り込みタイマ開始
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
	while(SCI1.SSR.BIT.TDRE == 0);//箱TDRの中にデータが残っていればフラグ=1  出てくまで待つ
	SCI1.TDR=character;//データを書き込み（ライト）
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
/*	MTU4.TGRB = PWM_PERIOD*45/100;		//追加柳田12/22
	MTU4.TGRD = PWM_PERIOD*45/100;		//追加柳田12/22
	MTU6.TGRB = PWM_PERIOD*45/100;		//追加柳田12/22
	MTU6.TGRD = PWM_PERIOD*45/100;		//追加柳田12/22
	
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
				
				//デュアルショックの値を取る
				Rspi_recive_send_line_dualshock();
				getdate1.dword = g_controller_receive_1st;
				getdate2.dword = g_controller_receive_2nd;
				getdate3.dword = g_controller_receive_3rd;
				
				//スティックによるｘ、ｙ方向の出力決め
				//x方向
				if( getdate3.byte.left_stick_high >=  0xBF){
					Motor_output_x = ( -1 ) * fabs( ( (float)getdate3.byte.left_stick_high - ( nutral_x + STICK_NO_MOVE_RANGE ) ) * ( pwm_percent / ( nutral_x - STICK_NO_MOVE_RANGE + 1) ));
				}else if( getdate3.byte.left_stick_high <=  0x3F){
					Motor_output_x = (( nutral_x - STICK_NO_MOVE_RANGE ) - (float)getdate3.byte.left_stick_high ) * ( pwm_percent / ( nutral_x - STICK_NO_MOVE_RANGE ) );	
				}else{
					Motor_output_x = 0.0;
				}
				//y方向
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
