#include"iodefine.h"

#define ON				0 //スイッチのON,OFF
#define OFF				1
#define PLUS_SWITCH			PORTB.PORT.BIT.B4
#define MINUS_SWITCH 			PORTE.PORT.BIT.B3
#define START_SWITCH			PORTE.PORT.BIT.B4

int count = 0,
    timer_on = OFF,
    timer_count = 0;

void warikomi(void){
	
	count++;
	if(timer_on == ON){
		timer_count++;
	}
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
	
	IEN(CMT0,CMI0) = 1; //割り込み要求許可レジスタ
	IPR(CMT0,CMI0) = 15; //優先度MAX
	
}
//ダウンエッジ　チャタリング関数
int chata_down (int swt2){

	static int j = 0,
		   b = 0,
		   y = 0;
	
	if(swt2 == 1){
		j++;
	}else if(swt2 == 0){
		j = 0;
	}
	
	if(y == 0 && j >= 4){
		b = 1 - b;
		y = 1;
	}
	if(swt2 == 0){
		y = 0;
	}
	
	return b;
	
}

//アップエッジ　チャタリング関数。5回条件を満たすとONを返す
int chata_up (int swt1, int i2){

	static int i[3] = { 0, 0, 0},
		   kekka[3] = { 0, 0, 0},
		   zyoutai[3] = { 0, 0, 0};
	
	if(swt1 == 1){
		i[i2]++;
	}else if(swt1 == 0){
		i[i2] = 0;
	}
	
	if(zyoutai[i2] == 0 && i[i2] >= 4){
		kekka[i2] = 1 - kekka[i2];
		zyoutai[i2] = 1;
		
	}
	if(i[i2] == 0){
		zyoutai[i2] = 0;
	}
	
//	return kekka[i2]; //継続状態
	return zyoutai[i2]; //状態変化
	
}


void main(void)
{	
	int swt1 = 0,
	    swt2 = 0,
	    swt3 = OFF,
	//    timer_count = 0,
	    timer_max = 0,
	//    timer_on = 0,
	    swt1_x = 0,
	    swt2_x = 0;
	//long count = 0;
	
	init_clock();
	init_cmt0();
	
	PORT9.DDR.BYTE = 0x3F;
	PORTB.DDR.BYTE = 0x00;
	PORTE.DDR.BYTE = 0x00;
	
	PORT9.DR.BYTE = 0x00;
	
	while(1){

		
		if(count >= 5){
			swt1 = chata_up(1-(PLUS_SWITCH) , 0);
			swt2 = chata_up(1-(MINUS_SWITCH) , 1);
			swt3 = chata_up(1-(START_SWITCH) , 2);
			count = 0;
			
			if(timer_count % 1000 == 0 && timer_on == 1){
				PORT9.DR.BYTE |= 0x01;
			}
			if(timer_count % 2000 == 0 && timer_on == 1){
				PORT9.DR.BYTE &= 0x3e;
			}
			
			if(swt1 == 1 && swt1_x ==0){
				timer_max++;
				swt1_x = 1;
				//PORT9.DR.BYTE |= 0x01;
			}
			if(swt1 == 0){
				swt1_x = 0;
				//PORT9.DR.BYTE &= 0x3e;
			}
			
			if(swt2 == 1 && timer_max > 0 && swt2_x ==0){
				timer_max--;
				swt2_x = 1;
			}
			if(swt2 == 0){
				swt2_x = 0;
			}	
			
			if(swt3 == 1){
				timer_on = ON;
			}
		}
		

		
		if(timer_count >= (timer_max * 1000) && timer_on == ON){
			PORT9.DR.BYTE = 0x3F;
		}
	}


}
