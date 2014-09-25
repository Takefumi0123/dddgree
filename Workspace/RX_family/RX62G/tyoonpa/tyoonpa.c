#include"iodefine.h"
#include<stdio.h>
#include<math.h>

#define PCLK	48
#define bitreet	9600	//ビットレート

int count = 0;

void warikomi(void){
	
	count++;
	
}

void init_clock(void){
	
	SYSTEM.SCKCR.BIT.PCK = 1; //×4 ICKとは関係ない場所の速さ
	SYSTEM.SCKCR.BIT.ICK = 0; //×8 マイコンそのものの速さ
	
}

void init_cmt0(void){

	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //モジュールストップ状態の解除
	CMT.CMSTR0.BIT.STR0 = 0; //カウント動作の停止
	
	CMT0.CMCR.BIT.CKS = 0;//クロック選択 1/8
	CMT0.CMCOR = 6;   //CMCORの決定 48mhz/8/1000000
	
	CMT0.CMCNT = 0;//初期化
	CMT0.CMCR.BIT.CMIE = 1;  //割り込み許可
	CMT.CMSTR0.BIT.STR0 = 1;

	
	IEN(CMT0,CMI0) = 1; //割り込み要求許可レジスタ
	IPR(CMT0,CMI0) = 15; //優先度MAX
}

void serial(void){
	
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
	
	SCI1.BRR = PCLK * 1000000 / ( 64 * 0.5 * bitreet ) - 1; //BRRレジスタの設定値 P822
	
	for(i=0;i > 80000;i++);
	
	SCI1.SCR.BIT.RE = 1; //シリアル受信動作を許可 P815
	SCI1.SCR.BIT.TE = 1; //シリアル送信動作を許可 P815
	
}
void tyoonpa(void){

	PORT2.DDR.BIT.B4 = 1; //トリガー出力
	PORT2.DDR.BIT.B3 = 0; //エコー入力
	
}

void send(char *s){
	int i = 0;
	while(s[i] != '\0'){	
			
		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			SCI1.TDR = s[i];
			i++;
		}
	}
}



int main(void){
	
	int phase =  1;
	double    kyori;
	char s[50] = { 0 };
	
	PORT9.DDR.BYTE = 0x3F;
	PORT9.DR.BYTE = 0x00;
	
	 init_clock();
	 init_cmt0();
	 serial();
	 tyoonpa();

	while ( 1 ){

		/*
		if(IR(CMT0,CMI0) == 1){
			IR(CMT0,CMI0) = 0;
			count ++;
		}*/
		
		if(phase == 1){
			PORT2.DR.BIT.B4 = 1;
			if( count >= 10){
				PORT2.DR.BIT.B4 = 0;
				phase ++;
			}
			PORT9.DR.BYTE |= 0x01;
		}
		else if( phase == 2){
			if( PORT2.PORT.BIT.B3 == 1){
				phase ++;
				count = 0;
			}
			PORT9.DR.BYTE |= 0x02;
		}
		else if( phase == 3){
			PORT9.DR.BYTE |= 0x04;
			if(PORT2.PORT.BIT.B3 == 0){
				PORT9.DR.BYTE |= 0x04;
				phase = 1;
				kyori = count * 0.017;
				
				sprintf(s,"%.3f\n\r",kyori);
				send(s);
				count = 0;
			}
		}
	
	}
}