#include"init.h"
#include"iodefine.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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


	MTUA.TSTR.BIT.CST4 = 0;			//カウント停止
	MTUB.TSTR.BIT.CST4 = 0;			//カウント停止
	MTUA.TSTR.BIT.CST3 = 0;			//柳田追加12/22
	MTUB.TSTR.BIT.CST0 = 0; 		//柳田追加12/22
	MTUB.TSTR.BIT.CST3 = 0; 		//柳田追加12/22
	IOPORT.PFCMTU.BIT.MTUS4 = 1;		//ポートファンクションレジスタMTIOC4A-B,MTIOC4C-B端子を選択　追加
	SYSTEM.MSTPCRA.BIT.MSTPA8 = 0;		//MTUユニット1（MTU6～MTU11）のモジュールストップ解除
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;		//MTUユニット0（MTU0～MTU5）のモジュールストップ解除 追加柳田12/22
	MTU10.TCNT = 0;				//TCNTを初期化する
	MTU4.TCNT = 0;				//追加柳田12/22　
	MTU6.TCNT = 0;				//追加柳田12/22
	MTU9.TCNT = 0;				//追加柳田12/22
	MTU10.TCR.BIT.CCLR = 0x01;		//同期クリア/同期動作をしている他のチャネルのカウンタクリアでTCNT をクリア
	MTU4.TCR.BIT.CCLR = 0x01;		//追加柳田12/22
	MTU6.TCR.BIT.CCLR = 0x01;		//追加柳田12/22
	MTU9.TCR.BIT.CCLR = 0x01;		//追加柳田12/22
	MTU10.TCR.BIT.CKEG = 0x00;		//立ち上がりエッジでカウント
	MTU4.TCR.BIT.CKEG = 0x00;		//追加柳田12/22
	MTU6.TCR.BIT.CKEG = 0x00;		//追加柳田12/22
	MTU9.TCR.BIT.CKEG = 0x00;		//追加柳田12/22
	MTU10.TCR.BIT.TPSC = 0x00;//0x03;		//内部クロック設定：ICLK/64
	MTU4.TCR.BIT.TPSC = 0x00;//0x03;		//追加柳田12/22
	MTU6.TCR.BIT.TPSC = 0x00;//0x03;		//追加柳田12/22
	MTU9.TCR.BIT.TPSC = 0x00;//0x03;		//追加柳田12/22

	MTU10.TMDR.BIT.MD = 0x02;		//PWMモード1
	MTU4.TMDR.BIT.MD = 0x02;		//追加柳田12/22
	MTU6.TMDR.BIT.MD = 0x02;		//追加柳田12/22
	MTU9.TMDR.BIT.MD = 0x02;		//追加柳田12/22
	
	MTUB.TOER.BIT.OE4A = 1;			//PWM出力許可 MTIOC10A
	MTUB.TOER.BIT.OE4C = 1;			//PWM出力許可 MTIOC10C
	MTUA.TOER.BIT.OE4A = 1;
	MTUA.TOER.BIT.OE4C = 1;
	
	MTU10.TIORH.BIT.IOA = 2;		//アウトプットコンペアレジスタに設定、初期値、出力値の選択A		←わからない←最適値が←柳田修正12/22 =6; → =1;
	MTU10.TIORH.BIT.IOB = 1;		//アウトプットコンペアレジスタに設定、初期値、出力値の選択B		柳田修正12/22 =5; → =1;
	MTU4.TIORH.BIT.IOA = 2;			//追加柳田12/22
	MTU4.TIORH.BIT.IOB = 1;			//追加柳田12/22
	MTU6.TIORH.BIT.IOA = 2;			//追加柳田12/22
	MTU6.TIORH.BIT.IOB = 1;			//追加柳田12/22
	MTU9.TIORH.BIT.IOA = 2;			//追加柳田12/22
	MTU9.TIORH.BIT.IOB = 1;			//追加柳田12/22
	
	MTU10.TIORL.BIT.IOC = 2;		//アウトプットコンペアレジスタに設定、初期値、出力値の選択C		柳田修正12/22 =6; → =2;
	MTU10.TIORL.BIT.IOD = 1;		//アウトプットコンペアレジスタに設定、初期値、出力値の選択D		柳田修正12/22 =5; → =1;
	MTU4.TIORL.BIT.IOC = 2;			//追加柳田12/22
	MTU4.TIORL.BIT.IOD = 1;			//追加柳田12/22
	MTU6.TIORL.BIT.IOC = 2;			//追加柳田12/22
	MTU6.TIORL.BIT.IOD = 1;			//追加柳田12/22
	MTU9.TIORL.BIT.IOC = 2;			//追加柳田12/22
	MTU9.TIORL.BIT.IOD = 1;			//追加柳田12/22
	
	MTU10.TGRA = PWM_PERIOD;		//48000 / 4 / 8 * 0.01(周辺クロック/内部クロック/タイマプリスケーラ*周期)[1kHz]
	MTU10.TGRC = PWM_PERIOD;		//48000 / 4 / 8 * 0.01(周辺クロック/内部クロック/タイマプリスケーラ*周期)[1kHz]
	MTU4.TGRA = PWM_PERIOD;		//追加柳田12/22
	MTU4.TGRC = PWM_PERIOD;		//追加柳田12/22
	MTU6.TGRA = PWM_PERIOD;//_OTHER;		//追加柳田12/22
	MTU6.TGRC = PWM_PERIOD;//_OTHER;		//追加柳田12/22
	MTU9.TGRA = PWM_PERIOD;		//追加柳田12/22
	MTU9.TGRC = PWM_PERIOD;		//追加柳田12/22

	MTUA.TSTR.BIT.CST3 = 1;			//柳田追加12/22	//2013.02.18	CST3→CST4	MTU4のため3だと動かなかった
	MTUA.TSTR.BIT.CST4 = 1;			//柳田追加12/22	//2013.02.18	CST3→CST4	MTU4のため3だと動かなかった
	MTUB.TSTR.BIT.CST4 = 1;			//カウント停止
	MTUB.TSTR.BIT.CST0 = 1; 		//柳田追加12/22
	MTUB.TSTR.BIT.CST3 = 1; 		//柳田追加12/22
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
	
	SYSTEM.MSTPCRB.BIT.MSTPB31 = 0;						//SCI0モジュールSTOP状態を解除
	SCI0.SCR.BYTE		= 0x00;										//シリアルコントロールレジスタ
																				//河原 0x01→0x00 で通信速度を最大に．分周1
	PORT2.DDR.BIT.B1	= 0;		//
	PORT2.ICR.BIT.B1	= 1;		//
	PORT2.DDR.BIT.B0 = 0;	//追加
	PORT2.ICR.BIT.B0 	= 1;	//

	SCI0.SMR.BYTE		= 0x00;		//シリアルモードレジスタ
	SCI0.SEMR.BIT.ABCS	= 1;		//調歩同期基本クロックを８サイクルの期間を１ビット期間の転送レートとする
	SCI0.BRR			= ((48*1000000)/((64/(1+SCI0.SEMR.BIT.ABCS))*powf(2,2*SCI0.SMR.BIT.CKS-1)*BITRATE_0)-1);;		//ビットレートレジスタ77  9600bpsなら0x01の77

	for( bit_count = 0; bit_count < 0x80000; bit_count++ ){	//１ビット待つため
	}
	SCI0.SCR.BYTE		= 0x70;		//送受信動作を許可
	
	IEN(SCI0,RXI0) = 1;
	IPR(SCI0,RXI0) = 13;
}

void init_Sci_1(void)	//SCI1版
{
	int bit_count = 0;
	
	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;							//シリアル通信1 モジュール状態の解除
	
	SCI1.SCR.BIT.TIE 	= 0;												//TXI割り込み要求を許可
	SCI1.SCR.BIT.RIE 	= 0;												//RXIおよびERI割り込み要求を許可
	SCI1.SCR.BIT.TE 	= 0;												//シリアル送信動作を禁止
	SCI1.SCR.BIT.RE 	= 0;												//シリアル受信動作を禁止
	SCI1.SCR.BIT.TEIE = 0;												//TEI割り込み要求を禁止
	
	SCI1.SCR.BIT.CKE	= 0;												//内臓ポーレートジェネレータ SCKn端子は入出力ポートとして使用可能 p815 
	
	SCI1.SMR.BIT.CKS 	= 0;											//PCLKクロック n=0
	SCI1.SMR.BIT.CHR 	= 0;											//p830
	SCI1.SMR.BIT.PE		= 0;											//p830
	SCI1.SMR.BIT.MP		= 0;											//p830	
	SCI1.SMR.BIT.STOP	= 0;											//p830
	
	SCI1.BRR = 48000000 / ( 64 * 0.5 * BITRATE_1 ) - 1;
	
	for( bit_count = 0; bit_count < 0x80000; bit_count++ ){	//１ビット待つため
	}
	
	SCI1.SCR.BIT.TE = 1;													//シリアル送信動作を許可
	SCI1.SCR.BIT.RE = 1;	
}

void init_Sci_2(void)//追加
{
	int i= 0;	
	
	SYSTEM.MSTPCRB.BIT.MSTPB29 = 0;	//SCI1モジュールSTOP状態を解除
										//河原 0x01→0x00 で通信速度を最大に．分周1
	PORT1.DDR.BIT.B2	= 0;		//
	PORT1.ICR.BIT.B2	= 1;		//
	PORT1.DDR.BIT.B3 	= 0;	//追加
	PORT1.ICR.BIT.B3 	= 1;	//追加

	
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
	
	SCI2.BRR = PCLK * 1000000 / ( 64 * 0.5 * BITRATE_2 ) - 1; //BRRレジスタの設定値 P822
	
	for(i=0;i > 80000;i++);
	
	SCI2.SCR.BIT.RE = 1; //シリアル受信動作を許可 P815
	SCI2.SCR.BIT.TE = 1; //シリアル送信動作を許可 P815
	IEN(SCI2,RXI2) = 1;
	IPR(SCI2,RXI2) = 11;
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

void init_dafalut()
{
	PORT8.DDR.BIT.B0 = 1;
	PORT8.DDR.BIT.B1 = 1;
	PORTD.DDR.BIT.B7 = 1;
	PORT6.DDR.BIT.B0 = 1;
	
	PORT1.DDR.BIT.B4 = ON;			//電磁弁
	PORT2.DDR.BIT.B5 = ON;			//
	PORT3.DDR.BIT.B3 = ON;			//
	PORT3.DDR.BIT.B2 = ON;			//電磁弁
	PORT5.DDR.BIT.B6 = ON;			//電磁弁
	PORT6.DDR.BIT.B0 = ON;			//電磁弁
}