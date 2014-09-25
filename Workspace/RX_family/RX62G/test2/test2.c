#include"iodefine.h"


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


void pwm(int TGRB,int TGRD){
	
	MSTP(MTU7) = 0;
	MTU.TSTRB.BIT.CST7 = 0; //カウンタ動作の停止
	
	MTU7.TCNT = 0x00;
	MTU7.TCR.BIT.TPSC = 0x02; //クロック分周 1/16
	MTU7.TCR.BIT.CKEG = 0x00; //カウントエッジ
	MTU7.TCR.BIT.CCLR = 0x02; //カウントクリア要因

	
	MTU7.TMDR1.BIT.MD = 0x02; //PWMモード設定
	
	MTU7.TIORH.BIT.IOA = 0x02;
	MTU7.TIORH.BIT.IOB = 0x01;
	MTU7.TIORL.BIT.IOC = 0x02;
	MTU7.TIORL.BIT.IOD = 0x01;
	
	MTU7.TGRA = TGRB; //周期設定
	MTU7.TGRB = 3000; //周期設定
	
	MTU7.TGRC = TGRD; //周期設定
	MTU7.TGRD = 3000; //周期設定
	
	MTU.TOERB.BIT.OE7A = 1;	
	MTU.TOERB.BIT.OE7B = 1;
	MTU.TOERB.BIT.OE7C = 1;
	MTU.TOERB.BIT.OE7D = 1;
	
	MTU.TSTRB.BIT.CST7 = 1; //カウンタ動作の開始
	
}

int main(){
	
	int count = 0,
		TGRB = 0,
		TGRD = 0;
	

	init_clock();
	init_cmt0();
	
	PORT9.DDR.BYTE = 0x3F;
	PORTB.DDR.BYTE = 0x00;
	PORTE.DDR.BYTE = 0x00;
	PORT8.DDR.BIT.B0 = 1;
	
	while(1){
		
		if(IR(CMT0,CMI0) == 1){
			count++;
			IR(CMT0,CMI0) = 0;
		}
		
		if(count >= 3000){
			count = 0;
			PORT8.DR.BIT.B0 = 1;
			PORT9.DR.BIT.B1 = 1;
		}
	}

}

