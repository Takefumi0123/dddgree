#include"iodefine.h"
#include<stdio.h>

#define PCLK	48
#define bitreet	9600

void init_clock(void){
	
	SYSTEM.SCKCR.BIT.PCK = 1; //×4 ICKとは関係ない場所の速さ
	SYSTEM.SCKCR.BIT.ICK = 0; //×8 マイコンそのものの速さ
	
}

void serial(void){
	
	int i;
	
	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;
	
	PORTD.ICR.BIT.B5 = 1;
	
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
	
	SCI1.BRR = PCLK * 1000000 / ( 64 * 0.5 * bitreet ) - 1; //BRRレジスタの設定値 P822
	
	for(i=0;i > 80000;i++);
	
	SCI1.SCR.BIT.RE = 1; //シリアル受信動作を許可 P815
	SCI1.SCR.BIT.TE = 1; //シリアル送信動作を許可 P815
	
}
void ad(void){
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
/*void send(int  a){
	int i;
	char s[50]={0};
	
		sprintf(s,"%d\n\r",a);
		

		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			
			if(s[i] == '\0'){
				i=0;
			}
			SCI1.TDR = s[i];
			i++;
		}

}*/

int main(void){
	
	int a;
		int i;
	char s[50]={0};
	init_clock();
	serial();
	ad();
	
	while(1){
		AD0.ADCSR.BIT.ADST = 1; //AD変換開始
		
		while(AD0.ADCSR.BIT.ADST == 1);
		
		a = AD0.ADDRD;

	//	send(a);
		
	
		sprintf(s,"%d\n\r",a);
		

		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			
			if(s[i] == '\0'){
				i=0;
			}
			SCI1.TDR = s[i];
			i++;
		}

	}
}