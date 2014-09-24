#include"iodefine.h"

long count = 0;

void sample(){
	//IR(CMT0,CMI0) = 0;
	count++;
}

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
	
	IEN(CMT0,CMI0) = 1;
	IPR(CMT0,CMI0) = 15;
	
	
	
}


int chata_down (int swt2){

	static int j = 0,
		   LED2 = 0,
		   y = 0;
	
	if(swt2 == 1){
		j++;
	}else if(swt2 == 0){
		j = 0;
	}
	
	if(y == 0 && j >= 4){
		LED2 = 1 - LED2;
		y = 1;
	}
	if(swt2 == 0){
		y = 0;
	}
	
	return LED2;
	
}

int chata_up (int swt1){

	static int i = 0,
		  LED1 = 0,
		   x = 0;
	
	if(swt1 == 1){
		i++;
	}else if(swt1 == 0){
		i = 0;
	}
	
	if(x == 0 && i >= 4){
		LED1 = 1 - LED1;
		x = 1;
	}
	if(i == 0){
		x = 0;
	}
	
	return LED1;
	
}


void main(void)
{	
	int LED1 = 0,
	    LED2 = 0;
	//long count = 0;
	
	init_clock();
	init_cmt0();
	
	PORT9.DDR.BYTE = 0x3F;
	PORTB.DDR.BYTE = 0x00;
	PORTE.DDR.BYTE = 0x00;
	
	while(1){
	
		/*if(IR(CMT0,CMI0) == 1){
			count++;
			IR(CMT0,CMI0) = 0;
		}*/
		
		if(count >= 5){
			LED1 = chata_up(1-(PORTB.PORT.BIT.B4));
			LED2 = chata_down(PORTE.PORT.BIT.B3);
			count = 0;
		}
		
		if(LED1 == 1){
			PORT9.DR.BYTE |= 0x01;
		}else{
			PORT9.DR.BYTE &= 0xfe;
		}
		
		if(LED2 == 1){
			PORT9.DR.BYTE |= 0x02;
		}else{
			PORT9.DR.BYTE &= 0xfd;
		}
	}


}
