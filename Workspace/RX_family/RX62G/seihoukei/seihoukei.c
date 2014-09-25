/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "typedefine.h"
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif




#ifdef __cplusplus
void abort(void)
{

}
#endif
#include "iodefine.h"
#include <math.h>
#include <stdio.h>

#define ON			1					//スイッチ状態
#define OFF			0
#define PCLK			48					//PCLK
#define BAUDREET		9600					//ボーレート
#define PLUS_SWITCH	PORTB.PORT.BIT.B4		//スイッチポート
#define M_PI			3.141592653589793		//円周率
#define TCNT_RIGHT		MTU2.TCNT			//右エンコーダTCNT
#define TCNT_LEFT		MTU1.TCNT			//左エンコーダTCNT
#define DIAMETER_LEFT	22.0					//左エンコーダ直径
#define DIAMETER_RIGHT	22.0					//右エンコーダ直径
#define PULSE_LEFT		100					//左エンコーダパルス
#define PULSE_RIGHT	100					//右エンコーダパルス
#define FLANK			228					//エンコーダ間距離
#define TURN_PGAIN		100					//回転出力Pゲイン
#define TURN_DGAIN		150					//回転出力Dゲイン
#define STRAIGHT_PGAIN	0.2					//直進出力Pゲイン
#define STRAIGHT_DGAIN	5.0					//直進出力Dゲイン
#define FRONT_LEFT		PORTB.DR.BIT.B3		//左正回転出力許可
#define BACK_LEFT		PORTB.DR.BIT.B1		//左逆回転出力許可
#define FRONT_RIGHT	PORTB.DR.BIT.B2		//右正回転出力許可
#define BACK_RIGHT		PORTA.DR.BIT.B2		//右逆回転出力許可
#define NOW_TARGET_X	COOD_XY[task].X		//目標垂直方向座標
#define NOW_TARGET_Y	COOD_XY[task].Y		//目標水平方向座標
#define END_DEGREE		COOD_XY[task].end_degree//目標地点到着後角度
#define OLD_TARGET_X	COOD_XY[task-1].X		//目標垂直方向座標
#define OLD_TARGET_Y	COOD_XY[task-1].Y		//目標水平方向座標
#define BRAKE			999999				//ブレーキ

int	g_count       =	0,	//1msカウント
	g_count2      =	0,	//1msカウント
	g_wave_count  = 0,	//1μsカウント
	g_over_LEFT   = 0,	//左オーバーフローカウント
	g_under_LEFT  = 0,	//左アンダーフローカウント
	g_over_RIGHT  = 0,	//右オーバーフローカウント
	g_under_RIGHT = 0;	//右アンダーフローカウント

double	g_degree	= 0.00, //機体の現在角度（°　）
		g_rad	= 0.00;//機体の現在角度（rad）
	
//構造体宣言	
struct 	coordinates
{
	double	X;			//垂直方向座標
	double	Y;			//水平方向座標
	double	end_degree;	//目標到着後角度
};
	
//PCK ICK設定	
void init_clock(void)
{
	SYSTEM.SCKCR.BIT.PCK = 1; //×4 ICKとは関係ない場所の速さ
	SYSTEM.SCKCR.BIT.ICK = 0; //×8 マイコンそのものの速さ	
}
  
//タイマー設定（1ms）
void init_cmt0(void)
{
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
void init_cmt1(void)
{
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
void init_pwm(void)
{
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
	
	SCI1.BRR = PCLK * 1000000 / ( 64 * 0.5 * BAUDREET ) - 1; //BRRレジスタの設定値 P822
	
	for( i = 0 ; i > 80000; i ++ );
	
	SCI1.SCR.BIT.RE = 1; //シリアル受信動作を許可 P815
	SCI1.SCR.BIT.TE = 1; //シリアル送信動作を許可 P815	
}

//エンコーダーMTU2
void init_counter_MTU2(void)
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

//エンコーダーMTU1
void init_counter_MTU1(void)
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

//初期設定
void init_dafault(void)
{	
	//入出力の初期化
	PORTA.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B1 = 1;
	PORTB.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B3 = 1;
	PORTB.DDR.BIT.B4 = 0;
	PORT9.DDR.BYTE   = 0x3F;
	
	//回転方向の初期化
        FRONT_LEFT  = 0;
        BACK_LEFT   = 0;
        FRONT_RIGHT = 0;
        BACK_RIGHT  = 0;	
}

//PWM	引数は％
void pwm(double TGRD,double TGRB)
{	
	MTU6.TGRB = 3000 * ( 100 - TGRB ) / 100;
	MTU6.TGRD = 3000 * ( 100 - TGRD ) / 100;	
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
void Move(double L_output,double R_output)
{		
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
		FRONT_LEFT = 1;
		BACK_LEFT  = 1;
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
		FRONT_RIGHT = 1;
		BACK_RIGHT  = 1;
	}  
	    
	pwm( L_output , R_output );
}	

//PD直進制御
double straight_PD(double difference )
{	
	double output = 0.00;
		
	static double	old_difference = 0.00,
			old_output = 0.00;
	
	output = old_output + ( STRAIGHT_PGAIN * difference ) + ( STRAIGHT_DGAIN * ( difference - old_difference ));
	
	output = Limit_ul( output,100,-100 );
	
	old_difference = difference;
	old_output = output;
	
	return output;
}	

//PD回転制御
double turn_PD( double difference )
{	
	double output = 0.00;
		
	static double old_difference = 0.00;
	
	output = ( TURN_PGAIN * difference ) + ( TURN_DGAIN * ( difference - old_difference ));
	
	output = Limit_ul( output,80,-80 );
	
	old_difference = difference;
	
	return output;
}	
	
//シリアル通信	
void transmission(char *string)
{
	int i = 0;
	
	while( string[i] != '\0' ){	
		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			SCI1.TDR = string[i];
			i ++;
		}
	}
}

//カウント（1ms）
void count_plus(void)
{	
	g_count ++;
	g_count2 ++;
}

//カウント（1μs）
void sonic_count_plus(void)
{	
	g_wave_count ++;	
}

//オーバーフローした時、変数g_overに1足す
void over_flow_LEFT(void)
{
	g_over_LEFT++;
}

//アンダーフローした時、変数g_underに１引く
void under_flow_LEFT(void)
{
	g_under_LEFT++;
}

//オーバーフローした時、変数g_overに1足す
void over_flow_RIGHT(void)
{
	g_over_RIGHT++;
}

//アンダーフローした時、変数g_underに１引く
void under_flow_RIGHT(void)
{
	g_under_RIGHT++;
}

//チャタリング（ダウンエッジ）
//スイッチを押し続ける（離し続ける）間切り替わるのが swt_keep
//スイッチを押す（離す）と切り替わるのが swt_change
int chata_down (int swt1, int i)
{
	static int status[3] = { 0, 0, 0},
		   swt_change[3] = { 0, 0, 0},
		   swt_keep[3] = { 0, 0, 0};
	
	if(swt1 == 1){
		status[i]++;
	}else if(swt1 == 0){
		status[i] = 0;
	}
	
	if(swt_keep[i] == 0 && status[i] >= 5){
		swt_change[i] = 1 - swt_change[i];
		swt_keep[i] = 1;
		
	}
	if(status[i] == 0){
		swt_keep[i] = 0;
	}
	
	return swt_change[i]; //継続状態
//	return swt_keep[i]; //状態変化	
}

int main(void)
{	
	int	swt1		= 1,			//スイッチ状態
		swt1_x 	= 1,
		status	= ON,
		task		= 1,			//タスク
		end_to_start = 0,
		task_change = 0;
		
	double	distance_LEFT		= 0.00,	//左輪進行距離
			distance_RIGHT		= 0.00,	//右輪進行距離
			add_distance_LEFT	= 0.00,	//左輪進行距離偏差
			add_distance_RIGHT	= 0.00,	//右輪進行距離偏差
			old_distance_LEFT	= 0.00,	//前回の左輪進行距離
			old_distance_RIGHT	= 0.00,	//前回の右輪進行距離
			add_degree		= 0.00,	//角度総計（°）
			target_degree		= 0.00, //目標座標までの角度（°）
			difference_degree	= 0.00,	//目標までの角度との偏差（°）
			another_difference_degree = 0.00, //目標までの角度との偏差　垂直水平距離の計算に用いる
			add_rad			= 0.00,	//角度総計（rad）
			x				= 0.00,	//垂直方向座標
			y				= 0.00,	//水平方向座標
			enc_RIGHT		= 0.00,	//右エンコーダ値
			enc_LEFT			= 0.00, //左エンコーダ値
			difference_distance	= 0.00,	//目標までの距離
			straight_output		= 0.00,	//直進出力
			turn_output		= 0.00,	//回転出力
			x_difference_distance	= 0.00,	//目標までの垂直距離
			y_difference_distance	= 0.00;	//目標までの水平距離
		
	char string[50] = { 0 };		//文字列宣言
	
	//構造体の初期化
	struct coordinates COOD_XY[10] = {
		{ 0, 0, 0},
		{ 0, 0 , 0},
		{ 500, 0 , 30},
		{ 1515, 865, 0},
		{ 2015, 865 , 0},
		{ BRAKE, BRAKE , BRAKE}
	};
	
	init_clock();
	init_cmt0();
	init_cmt1();
	init_serial();
	init_counter_MTU1();
	init_counter_MTU2();
	init_pwm();
	init_dafault();
	
	while(1){
		 // 5ms毎に以下の動作を行う
		if( g_count >= 5 ){
			g_count = 0;
			//スイッチがOFFのとき以下の動作をおこなう
			if( status == OFF ){
				swt1 = chata_down(( PLUS_SWITCH ), 0); //スイッチの状態の確認
				
				if( swt1 == 0 && swt1_x == 0){
					status = ON;
					swt1_x = 1;
					TCNT_RIGHT = 0;
					TCNT_LEFT = 0;
					x = 0;
					y = 0;
				}else
				if( swt1 == 1){
					swt1_x = 0;
				}
			}else
			//スイッチがONのとき以下の動作をおこなう
			if( status == ON ){
				enc_LEFT  =  TCNT_LEFT  + ( 65536 * g_over_LEFT)  + ( ( -65536 ) * g_under_LEFT  ); //左エンコーダーの値
				enc_RIGHT =  TCNT_RIGHT + ( 65536 * g_over_RIGHT) + ( ( -65536 ) * g_under_RIGHT ); //右エンコーダーの値
				distance_LEFT 	= -1 * ( enc_LEFT * M_PI * DIAMETER_LEFT)  / (PULSE_LEFT  * 4 ); //左輪進行mm
				distance_RIGHT 	= ( enc_RIGHT * M_PI * DIAMETER_RIGHT) / (PULSE_RIGHT * 4 ); //右輪進行mm
							
				add_distance_LEFT  = distance_LEFT  - old_distance_LEFT; //左輪偏差			
				add_distance_RIGHT = distance_RIGHT - old_distance_RIGHT; //右輪偏差
							
				g_rad = ( distance_RIGHT - distance_LEFT ) / FLANK; //角度（radian）
				add_rad +=  ( add_distance_RIGHT - add_distance_LEFT ) / FLANK; //累計角度（radian）
				g_degree = g_rad * 180 / M_PI; //角度（°）
				add_degree = add_degree + 180 * ( add_distance_RIGHT - add_distance_LEFT ) / ( FLANK * M_PI ); //累計角度（°）
				
				x += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * cos( add_rad ) ; //ｘ座標現在位置（縦）
				y += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * sin( add_rad ) ; //y座標現在位置（横）
				
				//atan2エラー回避
				if( NOW_TARGET_X == x && NOW_TARGET_Y == y ){
					target_degree = 0;
				}else{								
					target_degree = atan2( NOW_TARGET_Y - y , NOW_TARGET_X - x ) * 180 / M_PI; //現在値から見た目的地の角度
				}
				
				difference_distance = sqrt( ( NOW_TARGET_X - x ) * ( NOW_TARGET_X - x ) + ( NOW_TARGET_Y - y ) * ( NOW_TARGET_Y - y )); //現在値から見た目的地までの距離
				
				// -180 < degree < 180
				while ( g_degree > 180 ){
					g_degree = -360 + g_degree;
				}
				while ( g_degree < -180 ){
					g_degree = g_degree + 360;
				}
								
				difference_degree = target_degree - g_degree; //角度偏差（°）
				
				// -180 < difference_degree < 180
				if ( difference_degree > 180 ){
					difference_degree = -360 + difference_degree;
				}else
				if ( difference_degree < -180 ){
					difference_degree = difference_degree + 360;
				}
				
				another_difference_degree = difference_degree; //垂直水平距離計算に用いる角度
				
				// -90 < another_difference_degree < 90
				if ( difference_degree > 90 ){
					another_difference_degree = 180 - difference_degree;
					x_difference_distance = -1 * difference_distance * cos( another_difference_degree / 180 * M_PI );//目的地までの垂直距離
				}else
				if ( difference_degree < -90 ){
					another_difference_degree = -180 - difference_degree ;
					x_difference_distance = -1 * difference_distance * cos( another_difference_degree / 180 * M_PI );//目的地までの垂直距離					
				}else{
					x_difference_distance = difference_distance * cos( another_difference_degree / 180 * M_PI );//目的地までの垂直距離
				}
				//1つ前の値に格納
				old_distance_LEFT  = distance_LEFT;
				old_distance_RIGHT = distance_RIGHT;
				
				//1秒毎にシリアル通信を行う
				if(g_count2 >= 1000 ){
					g_count2 = 0;
//					sprintf(string,"%.3lf   %.3lf   %.3lf   %.3lf   %d\n\r",x,y,x_difference_distance,y_difference_distance,task);
//					transmission( string );
				}
				
				//目的地到着判定　垂直距離を見る
				if( x_difference_distance <= 2 && end_to_start == 0){
					end_to_start = 1; //目標地点到着フラグ
//					sprintf( string,"%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%d\n\r",x,y,x_difference_distance,y_difference_distance,g_degree,task);
//					transmission( string );
				}
				//目標地点に着いてから範囲内の角度に入ると以下に入る
				if( x_difference_distance <= 1 && x_difference_distance >= -1 && ( END_DEGREE - g_degree ) <= 1 && ( END_DEGREE - g_degree) >= -1 && end_to_start == 1){
					task_change ++;
				}
				//目標地点に着いてから範囲内の角度に5回入ると以下に入る
				if( task_change >= 500 ){
					task_change = 0;
					end_to_start = 0;
					task ++;
//					sprintf( string,"%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%d\n\r",x,y,x_difference_distance,y_difference_distance,g_degree,task);
//					transmission( string );
				}
				//目標地点に着いてから次の目標地点に向かうまでの制御
				if( end_to_start == 1){
					// -180 < END_DEGREE - g_degree < 180
					if( END_DEGREE - g_degree > 180){
						turn_output = turn_PD( END_DEGREE - g_degree - 360);
					}else
					if( END_DEGREE - g_degree < -180){
						turn_output = turn_PD( END_DEGREE - g_degree + 360);
					}else{
						turn_output = turn_PD( END_DEGREE - g_degree);
					}
					straight_output = straight_PD( x_difference_distance ); //直進PD制御
					Move( straight_output - turn_output ,straight_output + turn_output ); //モータ関数
				}else{
					//通常制御
					turn_output = turn_PD ( difference_degree ); //回転PD制御
					straight_output = straight_PD( x_difference_distance ); //直進PD制御
					Move( straight_output - turn_output ,straight_output + turn_output ); //モータ関数
				}
				//ブレーキ
				if( NOW_TARGET_X == BRAKE && NOW_TARGET_Y == BRAKE ){
					FRONT_LEFT = 1;
					BACK_LEFT = 1;
					FRONT_RIGHT = 1;
					BACK_RIGHT = 1;
					pwm( 99, 99);
				}
			}
		}
	}
}
