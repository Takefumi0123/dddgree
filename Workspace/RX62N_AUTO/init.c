#include"init.h"
#include"iodefine.h"

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
	
	#if BITRATE_0 == 9600
		SCI0.SMR.BYTE		= 0x01;	
	#else
		SCI0.SMR.BYTE		= 0x00;	
	#endif
	
	SCI0.BRR = ((48*1000000)/((64/(1+SCI0.SEMR.BIT.ABCS))*powf(2,2*SCI0.SMR.BIT.CKS-1)*BITRATE_0)-1);
	
	for( bit_count = 0; bit_count < 0x800000; bit_count++ ){	//１ビット待つため
	}
	SCI0.SCR.BYTE		= 0x70;		//送受信動作を許可
	
	IEN(SCI0,RXI0) = 1;
	IPR(SCI0,RXI0) = 11;
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
	
	SCI1.SMR.BIT.CKS 		= 0;											//PCLKクロック n=0
	SCI1.SMR.BIT.CHR 	= 0;											//p830
	SCI1.SMR.BIT.PE		= 0;											//p830
	SCI1.SMR.BIT.MP		= 0;											//p830	
	SCI1.SMR.BIT.STOP	= 0;											//p830
	
	SCI1.BRR = 48000000 / ( 64 * 0.5 * BITRATE_1 ) - 1;
	
	for( bit_count = 0; bit_count < 0x800000; bit_count++ ){	//１ビット待つため
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