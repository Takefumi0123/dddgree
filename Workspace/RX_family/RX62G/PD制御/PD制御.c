#include"iodefine.h"
#include <stdio.h>
#include <math.h>

#define PCLK	48
#define BITREET	9600	//ビットレート
#define PULSE_LEFT	500	//エンコーダーのパルス
#define PULSE_RIGHT	500	//エンコーダーのパルス
#define M_PI	3.141592653589793	//円周率
#define DIAMETER_LEFT	56	//直径		
#define DIAMETER_RIGHT	56	//直径	
#define FLANK		200	//両輪の幅
#define PGAIN		0.4
#define DGAIN		1.0
#define FRONT_LEFT	PORTB.DR.BIT.B1
#define BACK_LEFT	PORTB.DR.BIT.B3
#define FRONT_RIGHT	PORTA.DR.BIT.B2
#define BACK_RIGHT	PORTB.DR.BIT.B2
#define PLUS_SWITCH	PORTB.PORT.BIT.B4

int g_over_LEFT = 0,
    g_under_LEFT = 0,
    g_over_RIGHT = 0,
    g_under_RIGHT = 0,
    count = 0;

void count_plus(void){
	count ++;
}
     
    
//オーバーフローした時、変数g_overに1足す
void over_flow_LEFT(void){
	g_over_LEFT++;
}

//アンダーフローした時、変数g_underに１引く
void under_flow_LEFT(void){
	g_under_LEFT++;
}

//オーバーフローした時、変数g_overに1足す
void over_flow_RIGHT(void){
	g_over_RIGHT++;
}

//アンダーフローした時、変数g_underに１引く
void under_flow_RIGHT(void){
	g_under_RIGHT++;
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
	MTU6.TGRB = 3000; //周期設定
	
	MTU6.TGRC = 3000; //周期設定
	MTU6.TGRD = 3000; //周期設定
	
//	MTU.TOERB.BIT.OE6A = 1;	
	MTU.TOERB.BIT.OE6B = 1;
//	MTU.TOERB.BIT.OE6C = 1;
	MTU.TOERB.BIT.OE6D = 1;
	
	MTU.TSTRB.BIT.CST6 = 1; //カウンタ動作の開始
	
}

void init_serial(void){
	
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

void init_counter_MTU2(void){
	
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


void init_counter_MTU1(void){
	
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

void pwm2(double TGRB,double TGRD){
	MTU6.TGRB = 30 * ( 100 - TGRB );
	MTU6.TGRD = 30 * (  100 - TGRD );
	
}

void Move(double L_output,double R_output)
{
	int straight = 0;
	//L_output = L_output  + straight;
	//R_output = -1 * R_output  + straight;
	
    if (L_output >= 100){
        L_output = 99;
    }
    if (L_output <= -100){
        L_output = -99;
    }

    if (R_output >= 100){
        R_output = 99;
    }
    if (R_output <= -100){
        R_output = -99;
    }    
    
    if (L_output > 0){
        FRONT_LEFT = 1;
        BACK_LEFT = 0;
    }else
    if (L_output < 0){
        FRONT_LEFT = 0;
        BACK_LEFT = 1;
	L_output = -1 * L_output;
    }    

    if (R_output > 0){
        FRONT_RIGHT = 1;
        BACK_RIGHT = 0;
    }else
    if (R_output < 0){
        FRONT_RIGHT = 0;
        BACK_RIGHT = 1;
	R_output = -1 * R_output;
    }    
    
    pwm2(R_output,L_output);
}


void PD( double genzai, double mokuhyou ){
	
	double output = 0.00,
		hensa = 0.00;
	static double old_hensa = 0.00;
	
	hensa = mokuhyou - genzai;
	
	output = ( PGAIN * hensa )/* + ( DGAIN * ( hensa - old_hensa ))*/;
	
	old_hensa = hensa;
	
	Move( output, output );
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


int main(void){
	
	int tcnt_LEFT,
	    tcnt_RIGHT,
	    swt1 = 0,
	    swt1_x = 0;
	    
	double length_LEFT,
	       length_RIGHT,
	       enc_LEFT,
	       enc_RIGHT,
	       old_length_LEFT = 0,
	       old_length_RIGHT = 0,
	       d_length_LEFT = 0,
	       d_length_RIGHT = 0,
	       old_enc_LEFT = 0,
	       old_enc_RIGHT = 0,
	       d_enc_LEFT = 0,
	       d_enc_RIGHT = 0,
	       degree,
	       old_degree = 0,
	       d_degree = 0,
	       rad,
	       old_rad = 0,
	       d_rad = 0,
	       x = 0,
	       y = 0,
	       output = 0,
	       straight = 100,
	       TGRB = 0;
	
	init_clock();
	init_serial();
	init_counter_MTU1();
	init_counter_MTU2();
	init_cmt0();
	init_pwm();
	
        FRONT_LEFT = 0;
        BACK_LEFT = 0;
        FRONT_RIGHT = 0;
        BACK_RIGHT = 0;
	
	PORTA.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B1 = 1;
	PORTB.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B3 = 1;
	PORT9.DDR.BYTE = 0x3F;
	PORTB.DDR.BIT.B4 = 0;

	while(1){

		if( count >= 5 ){
			
			count = 0;
			tcnt_LEFT = MTU1.TCNT; //変数にTCNTの数値を格納(左輪)	
			tcnt_RIGHT = MTU2.TCNT; //変数にTCNTの数値を格納(右輪)
			
			enc_LEFT =  tcnt_LEFT +(65536 * g_over_LEFT) + (-65536 * g_under_LEFT);
			enc_RIGHT =  tcnt_RIGHT +(65536 * g_over_RIGHT) + (-65536 * g_under_RIGHT);
			
			length_LEFT = ( ( tcnt_LEFT +(65536 * g_over_LEFT) + (-65536 * g_under_LEFT) ) * M_PI * DIAMETER_LEFT) / (PULSE_LEFT *4 ); //左輪進行mm
			length_RIGHT = ( ( tcnt_RIGHT +(65536 * g_over_RIGHT) + (-65536 * g_under_RIGHT) ) * M_PI * DIAMETER_RIGHT) / (PULSE_RIGHT *4 ); //右輪進行mm		

			degree = 180 * ( length_RIGHT - length_LEFT ) / ( FLANK * M_PI ); //角度（°）
			rad = ( length_RIGHT - length_LEFT ) / FLANK; //角度（radian)

			d_length_LEFT = length_LEFT - old_length_LEFT; //左輪偏差			
			d_length_RIGHT = length_RIGHT - old_length_RIGHT; //右輪偏差
			
			d_enc_LEFT = enc_LEFT - old_enc_LEFT; //左輪偏差			
			d_enc_RIGHT = enc_RIGHT - old_enc_RIGHT; //右輪偏差
			
			d_degree = degree - old_degree; //角度偏差（°）
			d_rad = rad - old_rad; //角度偏差（radian）
			
//			x =  x+(90 * sin ( rad ) * -1 * tan ( rad / 2)  * ( d_length_LEFT + d_length_RIGHT)) / (M_PI * degree);
//			y =  y+(90 * sin ( rad ) * ( d_length_LEFT + d_length_RIGHT)) / (M_PI * degree);			
			
//			x = x + ( d_length_LEFT + d_length_RIGHT ) * sin( rad ) / 2; //ｘ座標現在位置（横）
//			y = y + ( d_length_LEFT + d_length_RIGHT ) * cos( rad ) / 2; //y座標現在位置（縦）
			
			x = x + (( d_enc_LEFT + d_enc_RIGHT ) * sin( rad ) * M_PI * DIAMETER_RIGHT)/ (2 *  (PULSE_RIGHT *4 )); //ｘ座標現在位置（横）
			y = y + (( d_enc_LEFT + d_enc_RIGHT ) * cos( rad ) * M_PI * DIAMETER_RIGHT)/ (2 *  (PULSE_RIGHT *4 )); //y座標現在位置（縦）
			
			//「左輪偏差　右輪偏差　角度偏差（°）　x座標　y座標」出力
//			sprintf(s,"LEFT : %.4f mm  RIGHT : %.4f mm  degree : %.4f   x : %.4f  y : %.4f\n\r",length_LEFT,length_RIGHT,degree,x,y);
			
			
			//「左輪偏差　右輪偏差　角度偏PU差（°）　x座標　y座標」出力
//			sprintf(s,"%.4f %.4f %.4f\n\r",degree,x,y);
			
			//1つ古い変数に格納
			old_length_LEFT = length_LEFT;
			old_length_RIGHT = length_RIGHT;
			old_enc_LEFT = enc_LEFT;
			old_enc_RIGHT = enc_RIGHT;
			old_degree = degree;
			old_rad = rad;
			
			if (degree > 180 ){
				degree = -360 + degree;
			}
			if (degree < -180 ){
				degree= degree + 360;
			}
			PD( degree , 0);

			swt1 = chata_up(1-(PLUS_SWITCH) , 0);
					
/*			if(swt1 == 1 && swt1_x ==0){
				TGRB = TGRB + 10;
				if(TGRB == 100){
					TGRB = 0;}
				swt1_x = 1;
			}
			if(swt1 == 0){
				swt1_x = 0;
			}*/			
		}

	}
}