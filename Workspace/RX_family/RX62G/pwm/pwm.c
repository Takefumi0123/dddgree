#include"iodefine.h"

int count = 0;

void init_clock(void){
	
	SYSTEM.SCKCR.BIT.PCK = 1; //×4 ICKとは関係ない場所の速さ
	SYSTEM.SCKCR.BIT.ICK = 0; //×8 マイコンそのものの速さ
	
}

void init_cmt0(void){

	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //モジュールストップ状態の解除
	CMT.CMSTR0.BIT.STR0 = 0; //カウント動作の停止
	
	CMT0.CMCR.BIT.CKS = 2;//クロック選択 1/128
	CMT0.CMCOR = 375;   //CMCORの決定 48mhz/128/1000
	
	CMT0.CMCNT = 0;//初期化
	CMT0.CMCR.BIT.CMIE = 1;  //割り込み許可
	CMT.CMSTR0.BIT.STR0 = 1;

	
}
void init_MTU7A_PWM(void)
{//p480 PWMモード
	MSTP(MTU7)=0;			//p186 0：モジュールストップ状態の解除
	MTU.TSTRB.BIT.CST7=0;		//p426 MTU7.TCNTのカウント動作は停止
	MTU7.TCR.BIT.TPSC=0x02;		//p390 内部クロック:ICLK/16でカウント
	MTU7.TCR.BIT.CKEG=0x00;		//p388 立ち上がりエッジでカウント
	MTU7.TCR.BIT.CCLR=0x01;		//p390 TGRAの今ペアマッチ/インプットキャプチャでTCNTクリア
	MTU7.TMDR1.BIT.MD=0x02;		//p393 PWMモード１
	MTU7.TIORH.BIT.IOA=0x01;	//p408 TGRA 初期出力はLow出力　コンペアマッチでLow出力
	MTU7.TIORH.BIT.IOB=0x02;	//p402 TGRB 初期出力はLow出力　コンペアマッチでHigh出力
	
	MTU7.TCNT=0;
	
	MTU7.TGRA=1023;
	MTU7.TGRB=900;
	
	MTU.TOERB.BIT.OE7A=1;		//p433 マスタ許可MTIOC7Aビット 1:MTU出力許可
	
	MTU.TSTRB.BIT.CST7=1;		//p426 MTU7.TCNTはカウント動作
}
void init_pwm(void){
	

	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;
	MTU.TSTRB.BIT.CST6 = 0; //カウンタ動作の停止
	
	MTU6.TCNT = 0x00;
	MTU6.TCR.BIT.TPSC = 0x02; //クロック分周 1/16
	MTU6.TCR.BIT.CKEG = 0x00; //カウントエッジ
	MTU6.TCR.BIT.CCLR = 0x01; //カウントクリア要因

	
	MTU6.TMDR1.BIT.MD = 0x02; //PWMモード設定
	
	MTU6.TIORH.BIT.IOA = 0x02;
	MTU6.TIORH.BIT.IOB = 0x01;
	MTU6.TIORL.BIT.IOC = 0x02;
	MTU6.TIORL.BIT.IOD = 0x01;
	
	MTU6.TGRA = 3000; //周期設定
	MTU6.TGRB = 2999; //周期設定
	
	MTU6.TGRC = 3000; //周期設定
	MTU6.TGRD = 2999; //周期設定
	
//	MTU.TOERB.BIT.OE6A = 1;	
	MTU.TOERB.BIT.OE6B = 1;
//	MTU.TOERB.BIT.OE6C = 1;
	MTU.TOERB.BIT.OE6D = 1;
	
	MTU.TSTRB.BIT.CST6 = 1; //カウンタ動作の開始
	
}

void pwm( int TGRB , int TGRD ){
	MTU6.TGRB = TGRB;
	MTU6.TGRD = TGRD;
}

void count_plus(void){
	count++;
}

int main(){
	
	int 	TGRB = 0,
		TGRD = 0;
	
	PORT9.DDR.BYTE = 0x3F;
	PORTB.DDR.BYTE = 0x00;
	PORTE.DDR.BYTE = 0x00;
	init_clock();
	init_cmt0();
	init_pwm();
	while(1){
		
		if(count >= 1000){
			pwm(TGRB,TGRD);

			TGRB = TGRB + 500;
			if(TGRB == 3500){
			TGRB = 0;}
			TGRD = 3000 - TGRB;

			count = 0;
		}
	}

}

//p94 MTU7 TGRA TGRB
//p91 MTU7 TGRC TGRD