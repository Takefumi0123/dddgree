#include "iodefine.h"
#include <math.h>
#include <stdio.h>

#define ON		1
#define OFF		0
#define PCLK		48
#define BAUDREET	9600	//ボーレート
#define PLUS_SWITCH	PORTB.PORT.BIT.B4
#define M_PI		3.141592653589793
#define DIAMETER_LEFT	21.70180004											
#define DIAMETER_RIGHT	22.29819999										
#define PULSE_LEFT	100
#define PULSE_RIGHT	100
#define FLANK		228
#define TURN_PGAIN	110
#define TURN_DGAIN	270
#define STRAIGHT_PGAIN	2.9
#define STRAIGHT_DGAIN	100.0
#define FRONT_LEFT	PORTB.DR.BIT.B3
#define BACK_LEFT	PORTB.DR.BIT.B1
#define FRONT_RIGHT	PORTB.DR.BIT.B2
#define BACK_RIGHT	PORTA.DR.BIT.B2
#define PLUS_SWITCH	PORTB.PORT.BIT.B4

int	g_count       = 0,
	g_count2      = 0,
	g_wave_count  = 0,
	g_over_LEFT   = 0,
	g_under_LEFT  = 0,
	g_over_RIGHT  = 0,
	g_under_RIGHT = 0;
	
//PCK ICK設定	
void init_clock(void){
	
	SYSTEM.SCKCR.BIT.PCK = 1; //×4 ICKとは関係ない場所の速さ
	SYSTEM.SCKCR.BIT.ICK = 0; //×8 マイコンそのものの速さ
	
}
  
//タイマー設定（1ms）
void init_cmt0(void){

	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //モジュールストップ状態の解除
	CMT.CMSTR0.BIT.STR0 = 0; //カウント動作の停止
	
	CMT0.CMCR.BIT.CKS = 2;//クロック選択 1/128
	CMT0.CMCOR = 375;   //CMCORの決定 48mhz/128/1000
	
	CMT0.CMCNT = 0;//初期化
	CMT0.CMCR.BIT.CMIE  = 1;  //割り込み許可
	CMT.CMSTR0.BIT.STR0 = 1;
	
	IEN(CMT0,CMI0) = 1; //割り込み要求許可レジスタ
	IPR(CMT0,CMI0) = 15; //優先度MAX
	
}
  
//タイマー設定（1μs）
void init_cmt1(void){

	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //モジュールストップ状態の解除
	CMT.CMSTR0.BIT.STR1 = 0; //カウント動作の停止
	
	CMT1.CMCR.BIT.CKS = 0;//クロック選択 1/8
	CMT1.CMCOR = 6;   //CMCORの決定 48mhz/8/1000000
	
	CMT1.CMCNT = 0;//初期化
	CMT1.CMCR.BIT.CMIE  = 1;  //割り込み許可
	CMT.CMSTR0.BIT.STR1 = 1;
	
	IEN(CMT1,CMI1) = 1; //割り込み要求許可レジスタ
	IPR(CMT1,CMI1) = 15; //優先度MAX
	
}  
  
//PWM初期設定
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
		
	MTU.TOERB.BIT.OE6B = 1;
	MTU.TOERB.BIT.OE6D = 1;
	
	MTU.TSTRB.BIT.CST6 = 1; //カウンタ動作の開始
	
}

//シリアル通信初期設定
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
	
	SCI1.BRR = PCLK * 1000000 / ( 64 * 0.5 * BAUDREET ) - 1; //BRRレジスタの設定値 P822
	
	for( i = 0 ; i > 80000; i ++ );
	
	SCI1.SCR.BIT.RE = 1; //シリアル受信動作を許可 P815
	SCI1.SCR.BIT.TE = 1; //シリアル送信動作を許可 P815
	
}

//エンコーダーMTU2
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

//エンコーダーMTU1
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
	
//PWM
void pwm(double TGRD,double TGRB){
	
	MTU6.TGRB = 30 * ( 100 - TGRB );
	MTU6.TGRD = 30 * ( 100 - TGRD );
	
}

//範囲内出力
double Limit_ul(double figure,double max,double min)
{
	if(figure > max){
		return ( max );
	}else if(figure < min){
		return ( min );
	}else{
		return ( figure );
	}
}

//モータ関数
void Move(double L_output,double R_output){
		
	L_output = Limit_ul(L_output,100,-100);
	R_output = Limit_ul(R_output,100,-100);
	
	if (L_output > 0){
	        FRONT_LEFT = 1;
	        BACK_LEFT = 0;
	}else
	if (L_output < 0){
	        FRONT_LEFT = 0;
	        BACK_LEFT = 1;
		L_output = -1 * L_output;
	}else 
	if( L_output == 0 ){
		FRONT_LEFT = 0;
		BACK_LEFT  = 0;
	}
	
        if (R_output > 0){
	        FRONT_RIGHT = 1;
	        BACK_RIGHT = 0;
        }else
	if (R_output < 0){
	        FRONT_RIGHT = 0;
	        BACK_RIGHT = 1;
		R_output = -1 * R_output;
	}else
	if( R_output == 0){
		FRONT_RIGHT = 0;
		BACK_RIGHT  = 0;
	}  
	    
	pwm( L_output , R_output );
	    
}	



//PD直進制御
double straight_PD(double difference ){
	
	double output = 0.00;
		
	static double old_difference = 0.00;
	
	output = ( STRAIGHT_PGAIN * difference ) + ( STRAIGHT_DGAIN * ( difference - old_difference ));
	
	output = Limit_ul( output,100,-100 );
	
	old_difference = difference;
	
	return output;
}	

//PD回転制御
double turn_PD( double difference ){
	
	double output = 0.00;
		
	static double old_difference = 0.00;
	
	output = ( TURN_PGAIN * difference ) + ( TURN_DGAIN * ( difference - old_difference ));
	
	output = Limit_ul( output,100,-100 );
	
	old_difference = difference;
	
	return output;
}	
	
//シリアル通信	
void transmission(char *string){
	int i = 0;
	while( string[i] != '\0' ){	
			
		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			SCI1.TDR = string[i];
			i ++;
		}
	}
}

//超音波（1μs）
void sonic_wave(void){

	static int phase = 1;
	double object_distance = 0;
	char string[50] = { 0 };

	PORT2.DDR.BIT.B4 = 1; //トリガー出力
	PORT2.DDR.BIT.B3 = 0; //エコー入力

	g_wave_count ++ ;
	
	if( phase == 1 ){
		PORT2.DR.BIT.B4 = 1;
		if( g_wave_count >= 10){
			PORT2.DR.BIT.B4 = 0;
			phase ++;
		}
	}else 
	if( phase == 2 ){
		if( PORT2.PORT.BIT.B3 == 1){
			phase ++;
			g_wave_count = 0;
		}
	}else 
	if( phase == 3 ){
		if(PORT2.PORT.BIT.B3 == 0){
			phase = 1;
			object_distance = g_wave_count * 0.017;
			
//			sprintf( s, "%.3f\n\r", object_distance );
//			transmission( string );
//			PD( object_distance , 0 );
			g_wave_count = 0;
		}
	}
}

//カウント（1ms）
void count_plus(void){
	
	g_count ++;
	g_count2 ++;

}

void sonic_count_plus(void){
	
	g_wave_count ++;
	
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

//チャタリング（ダウンエッジ）
int chata_down (int swt1, int i2){

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
	
	return kekka[i2]; //継続状態
//	return zyoutai[i2]; //状態変化
	
}

int main(void){

	double	end_x = 2000,
			end_y = 0;
	
	
	int swt1 = 1,
		swt1_x = 1,
		status = ON,
		task = 0;
		
	double	tcnt_LEFT = 0,
		tcnt_RIGHT = 0;
		
	double	distance_LEFT = 0,
		distance_RIGHT = 0,
		add_distance_LEFT = 0,
		add_distance_RIGHT = 0,
		old_distance_LEFT = 0,
		old_distance_RIGHT = 0,
		degree = 0,
		add_degree = 0,
		target_degree = 0,
		difference_degree = 0,
		rad = 0,
		add_rad = 0,
		x = 0,
		y = 0,
		enc_RIGHT = 0,
		enc_LEFT = 0,
		difference_distance = 0,
		straight_output = 0,
		turn_output = 0,
		x_difference_distance = 0,
		y_difference_distance = 0;
		
	char string[50] = { 0 };
		
		
	init_clock();
	init_cmt0();
	init_cmt1();
	init_serial();
	init_counter_MTU1();
	init_counter_MTU2();
	init_pwm();
	
        FRONT_LEFT  = 0;
        BACK_LEFT   = 0;
        FRONT_RIGHT = 0;
        BACK_RIGHT  = 0;
		
	PORTA.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B1 = 1;
	PORTB.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B3 = 1;
	PORTB.DDR.BIT.B4 = 0;
	PORT9.DDR.BYTE   = 0x3F;
	
	while(1){
		
		 // 5ms毎に以下の動作を行う
		if( g_count >= 5 ){
			
			g_count = 0;
			
			tcnt_RIGHT =  MTU2.TCNT;
			tcnt_LEFT  =  MTU1.TCNT;
			
			swt1 = chata_down(( PLUS_SWITCH ), 0); //スイッチの状態の確認
			
			
			if(( swt1 == 0 ) && ( swt1_x == 0 )){
				swt1_x = 1;
				status = ON;
			}
			
			if(swt1 == 1){
				swt1_x = 0;
			}
			
			distance_LEFT 	= -1 * ( ( tcnt_LEFT  +( 65536 * g_over_LEFT  ) + ( -65536 * g_under_LEFT) ) * M_PI * DIAMETER_LEFT)  / (PULSE_LEFT  * 4 ); //左輪進行mm
			distance_RIGHT 	= ( ( tcnt_RIGHT +( 65536 * g_over_RIGHT ) + ( -65536 * g_under_RIGHT) ) * M_PI * DIAMETER_RIGHT) / (PULSE_RIGHT * 4 ); //右輪進行mm
			
			enc_LEFT  =  tcnt_LEFT  + ( 65536 * g_over_LEFT)  + ( ( -65536 ) * g_under_LEFT  ); //左エンコーダーの値
			enc_RIGHT =  tcnt_RIGHT + ( 65536 * g_over_RIGHT) + ( ( -65536 ) * g_under_RIGHT ); //右エンコーダーの値
		
			add_distance_LEFT  = distance_LEFT  - old_distance_LEFT; //左輪偏差			
			add_distance_RIGHT = distance_RIGHT - old_distance_RIGHT; //右輪偏差
			

			rad = ( distance_RIGHT - distance_LEFT ) / FLANK; //角度（radian）
			add_rad +=  ( add_distance_RIGHT - add_distance_LEFT ) / FLANK; //累計角度（radian）
			degree = rad * 180 / M_PI ; //角度（°）
			add_degree = add_degree + 180 * ( add_distance_RIGHT - add_distance_LEFT ) / ( FLANK * M_PI ); //累計角度（°）
			
			x += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * cos( add_rad ) ; //ｘ座標現在位置（縦）
			y += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * sin( add_rad ) ; //y座標現在位置（横）
			
			target_degree = atan2( end_x - x , end_y - y ) * 180 / M_PI; //現在値から見た目的地の角度
			difference_distance = sqrt( ( end_x - x ) * ( end_x - x ) + ( end_y - y ) * ( end_y - y )); //現在値から見た目的地までの距離
		
			if( end_x == x && end_y == y ){
				x_difference_distance = 0;
				y_difference_distance = 0;	
			}
			
			difference_degree = target_degree - degree;							
			
			// -180 < degree < 180
			while (degree > 180 ){
				degree = -360 + degree;
			}
			while (degree < -180 ){
				degree = degree + 360;
			}
			
			if ( difference_degree > 180 ){
				difference_degree = -360 + difference_degree;
			}else
			if ( difference_degree < -180 ){
				difference_degree = difference_degree + 360;
			}
			if ( difference_degree > 90 ){
				difference_degree = 180 - difference_degree;
			}else
			if ( difference_degree < -90 ){
				difference_degree = -180 - difference_degree ;
			}	
			
			x_difference_distance = difference_distance * sin( difference_degree );
			y_difference_distance = difference_distance * cos( difference_degree );			
			
			//1つ前の値に格納
			old_distance_LEFT  = distance_LEFT;
			old_distance_RIGHT = distance_RIGHT;
			
//			sprintf( string,"%.3lf,%.3lf,%.3lf\n\r",x,y,degree);
//			transmission( string );

			if( status == ON ){
				turn_output = turn_PD ( difference_degree ); //回転PD制御
				straight_output = straight_PD ( x_difference_distance ); //直進PD制御
				Move( straight_output + ( -1 * turn_output ),straight_output + ( turn_output ) ); //モータ関数
				
				if( add_distance_LEFT == 0 && add_distance_RIGHT && g_count2 >= 1000 ){
					g_count2 = 0;
					sprintf( string,"%.3lf,%.3lf,%.3lf\n\r",x,y,degree);
					transmission( string );
					task ++;
				}
				if( task == 0 ){
					end_x = 2000;
					end_y = 0;
				}
				if( task == 1 ){
					end_x = 2000;
					end_y = 2000;
				}
				if( task == 2 ){
					end_x = 0;
					end_y = 2000;
				}
				if( task == 3 ){
					end_x = 0;
					end_y = 0;
				}
			}
		}
			
	}
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          