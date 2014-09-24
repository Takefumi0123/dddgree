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
*	タイトル ： タイマーの設定
*	  関数名 ： init_cmt1
*	  戻り値 ： void型 タイマーの設定 1μs
*	    引数 ： なし
*	  作成者 ： 石井
*	  作成日 ： 2014/06/23
******************************************************************************/


void init_cmt1(void){

	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //モジュールストップ状態の解除
	CMT.CMSTR0.BIT.STR1 = 0; //カウント動作の停止
	
	CMT1.CMCR.BIT.CKS = 0;//クロック選択 1/8
	CMT1.CMCOR = 6;   //CMCORの決定 48mhz/8/1000000
	
	CMT1.CMCNT = 0;//初期化
	CMT1.CMCR.BIT.CMIE = 1;  //割り込み許可
	CMT.CMSTR0.BIT.STR1 = 1;
	
	IEN(CMT1,CMI1) = 1; //割り込み要求許可レジスタ
	IPR(CMT1,CMI1) = 15; //優先度MAX
	
}

/******************************************************************************
*	タイトル ： エンコーダーMTU1
*	  関数名 ： init_encoder_MTU1
*	  戻り値 ： void型 エンコーダーMTU1
*	    引数 ： なし
*	  作成者 ： 石井
*	  作成日 ： 2014/06/21
******************************************************************************/

void init_encoder_MTU1(void)
{	
	IOPORT.PFCMTU.BIT.TCLKS = 1;  //MTCLKA端子として選択
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //モジュールストップ状態の解除
	
	PORT2.ICR.BIT.B1 = 1; //ポート選択
	PORT2.ICR.BIT.B0 = 1; //ポート選択

	MTU.TSTRA.BIT.CST1 = 0; //MTU1のカウント動作の停止
	
	MTU1.TCR.BIT.CCLR = 0; //TCNTのクリア禁止 P390
	
	MTU1.TMDR1.BIT.MD = 0x04; //位相計数モード１ P393
	
	MTU1.TIER.BIT.TCIEV = 1; //オーバーフロー割込み要求を許可
	MTU1.TIER.BIT.TCIEU = 1; //アンダーフロー割込み要求を許可
	
	IEN(MTU1,TCIV1) = 1;
	IEN(MTU1,TCIU1) = 1;	
	
	IPR(MTU1,TCIV1) = 15;
	IPR(MTU1,TCIU1) = 15;

	MTU1.TCNT = 0x00; //初期化
	MTU.TSTRA.BIT.CST1 = 1; //MTU1のカウント動作の開始
}

/******************************************************************************
*	タイトル ： エンコーダーMTU2
*	  関数名 ： init_encoder_MTU2
*	  戻り値 ： void型 エンコーダーMTU2
*	    引数 ： なし
*	  作成者 ： 石井
*	  作成日 ： 2014/06/21
******************************************************************************/

void init_encoder_MTU2(void)
{	
	IOPORT.PFCMTU.BIT.TCLKS = 1;  //MTCLKA端子として選択
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //モジュールストップ状態の解除
		
	PORT1.ICR.BIT.B1 = 1; //ポート選択
	PORT1.ICR.BIT.B0 = 1; //ポート選択	

	MTU.TSTRA.BIT.CST2 = 0; //MTU2のカウント動作の停止
		
	MTU2.TCR.BIT.CCLR = 0; //TCNTのクリア禁止 P390	
	
	MTU2.TMDR1.BIT.MD = 0x04; //位相計数モード１ P393
	
	MTU2.TIER.BIT.TCIEV = 1; //オーバーフロー割込み要求を許可
	MTU2.TIER.BIT.TCIEU = 1; //アンダーフロー割込み要求を許可
	
	IEN(MTU2,TCIV2) = 1;
	IEN(MTU2,TCIU2) = 1;	
	
	IPR(MTU2,TCIV2) = 15;
	IPR(MTU2,TCIU2) = 15;

	MTU2.TCNT = 0x00; //初期化
	MTU.TSTRA.BIT.CST2 = 1; //MTU2のカウント動作の開始
}

/******************************************************************************
*	タイトル ： なし
*	  関数名 ： init_AD
*	  戻り値 ： void型 
*	    引数 ： なし
*	  作成者 ： 石井
*	  作成日 ： 2014/08/05
******************************************************************************/

void init_AD(void){
	SYSTEM.MSTPCRA.BIT.MSTPA23 = 0;
	
	AD0.ADCSR.BIT.CH = 3; //シングルモード AN5
	AD0.ADCSR.BIT.ADST = 0; //AD変換停止
	AD0.ADCSR.BIT.ADIE = 0; //AD変換終了によるADI割込み禁止
	
	AD0.ADCR.BIT.MODE = 0; //シングルモード
	AD0.ADCR.BIT.CKS = 1; //クロック選択 PCLK/8
	
	AD0.ADDPR.BIT.DPPRC = 0; //ADデータレジスタに10ビット精度で格納
	AD0.ADDPR.BIT.DPSEL = 0; //データはLSB詰め(下から)
	
	AD0.ADSSTR = 0xFF; //サンプリング時間
	
}


/******************************************************************************
*	タイトル ： init_MTU6_pwm
*	  関数名 ： init_MTU6_pwm
*	  戻り値 ： void型 
*	    引数 ： なし
*	  作成者 ： 石井
*	  作成日 ： 2014/07/16
******************************************************************************/



void init_MTU6_pwm( void )
{
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;
	MTU.TSTRB.BIT.CST6 	      = 0; //カウンタ動作の停止
	
	MTU6.TCNT 		= 0x00;
	MTU6.TCR.BIT.TPSC 	= 0x02; //クロック分周 1/16
	MTU6.TCR.BIT.CKEG 	= 0x00; //カウントエッジ
	MTU6.TCR.BIT.CCLR 	= 0x01; //カウントクリア要因
	
	MTU6.TMDR1.BIT.MD = 0x02; //PWMモード設定
	
	MTU6.TIORH.BIT.IOA = 0x02;
	MTU6.TIORH.BIT.IOB = 0x01;
	MTU6.TIORL.BIT.IOC 	= 0x02;
	MTU6.TIORL.BIT.IOD 	= 0x01;
	
	MTU6.TGRA = 3000; //周期設定
	MTU6.TGRB = 2999; //周期設定
	
	MTU6.TGRC = 3000; //周期設定
	MTU6.TGRD = 2999; //周期設定
		
	MTU.TOERB.BIT.OE6B = 1;
	MTU.TOERB.BIT.OE6D = 1;
	
	MTU.TSTRB.BIT.CST6 = 1; //カウンタ動作の開始	
}

/******************************************************************************
*	タイトル ： シリアル通信の設定
*	  関数名 ： init_serial
*	  戻り値 ： void型 シリアル通信の設定
*	    引数 ： なし
*	  作成者 ： 石井
*	  作成日 ： 2014/06/21
******************************************************************************/

void init_serial(void)
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
