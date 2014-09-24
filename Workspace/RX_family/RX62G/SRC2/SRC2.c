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
void abort( void )
{

}
#endif
#include "iodefine.h"
#include "init.h"
#include <math.h>
#include <stdio.h>

#define blue
//#define red
#define ON				1				//スイッチ状態
#define OFF				0
#define INTERRUPT_TIME  	0.005
#define PLUS_SWITCH		PORT8.PORT.BIT.B2		//スイッチポート
#define PI					3.141592653			//円周率
#define TCNT_RIGHT			MTU2.TCNT			//右エンコーダTCNT
#define TCNT_LEFT			MTU1.TCNT			//左エ ンコーダTCNT
#define DIAMETER_LEFT		21.90608333				//左エンコーダ直径
#define DIAMETER_RIGHT		22.09391667				//右エンコーダ直径
#define PULSE_LEFT			100					//左エンコーダパルス
#define PULSE_RIGHT		100					//右エンコーダパルス
#define TREAD				228					//エンコーダ間距離
#define TURN_P_GAIN		20					//回転出力Pゲイン
#define TURN_D_GAIN		30					//回転出力Dゲイン
#define STRAIGHT_P_GAIN	0.8					//直進出力Pゲイン
#define STRAIGHT_D_GAIN	1.2					//直進出力Dゲイン
#define A_KEEP_P_GAIN		20
#define A_KEEP_D_GAIN		25
#define FRONT_LEFT			PORTB.DR.BIT.B1		//左正回転出力許可
#define BACK_LEFT			PORTB.DR.BIT.B3		//左逆回転出力許可
#define FRONT_RIGHT		PORTA.DR.BIT.B2		//右正回転出力許可
#define BACK_RIGHT			PORTB.DR.BIT.B2		//右逆回転出力許可
#define BRAKE				1965
#define V_MAX				450
#define V_ANGULAR_MAX		200
#define PLOT				7

int	g_timer_count       		=	0,	//1msカウント
	g_timer_count2     		=	0,	//1msカウント
	g_wave_count 			= 	0,	//1μsカウント
	g_over_MTU1_count  		= 	0,	//左オーバーフローカウント
	g_under_MTU1_count		= 	0,	//左アンダーフローカウント
	g_over_MTU2_count  		= 	0,	//右オーバーフローカウント
	g_under_MTU2_count 	= 	0;	//右アンダーフローカウント

float	g_degree	= 0.00,
	g_rad	= 0.00;

//構造体宣言	
struct 	coordinates
{
	float	X;
	float	Y;
	char	MODE;
};

//オーバーフローした時、変数g_overに1足す
void over_flow_MTU1( void )
{
	g_over_MTU1_count++;
}

//アンダーフローした時、変数g_underに１引く
void under_flow_MTU1( void )
{
	g_under_MTU1_count++;
}

//オーバーフローした時、変数g_overに1足す
void over_flow_MTU2( void )
{
	g_over_MTU2_count++;
}

//アンダーフローした時、変数g_underに１引く
void under_flow_MTU2( void )
{
	g_under_MTU2_count++;
}
//範囲内出力
float Limit_ul( float figure, float max, float min )
{
	if( figure > max ){
		return ( max );
	}else
	if( figure < min ){
		return ( min );
	}else{
		return ( figure );
	}
}
void pwm( float TGRD, float TGRB )
{	
	MTU6.TGRB = 3000 * ( 100 - TGRB ) / 100 - 0.00001;
	MTU6.TGRD = 3000 * ( 100 - TGRD ) / 100 - 0.00001;	
}
void Move( float L_output, float R_output )
{		
	L_output = Limit_ul( L_output, 99, -99);
	R_output = Limit_ul( R_output, 99, -99);
	
	if ( L_output > 0 ){
	        FRONT_LEFT 	= 1;
	        BACK_LEFT	= 0;
	}else
	if ( L_output < 0 ){
	        FRONT_LEFT 	= 0;
	        BACK_LEFT 	= 1;
		L_output 		= -1 * L_output;
	}else 
	if( L_output == 0 ){
		FRONT_LEFT	= 1;
		BACK_LEFT  	= 1;
	}
	
        if ( R_output > 0 ){
	        FRONT_RIGHT 	= 1;
	        BACK_RIGHT 	= 0;
        }else
	if ( R_output < 0 ){
	        FRONT_RIGHT 	= 0;
	        BACK_RIGHT 	= 1;
		R_output 		= -1 * R_output;
	}else
	if( R_output == 0 ){
		FRONT_RIGHT 	= 1;
		BACK_RIGHT  	= 1;
	}  
	    
	pwm( L_output , R_output );
}	
//PD直進制御
float straight_PD( float target_velocity, float now_velocity )
{	
	float output = 0.00,
		difference	= 0.00;
		
	static float	old_difference 	= 0.00;
	
	difference = target_velocity - now_velocity;	
			
	output =  ( STRAIGHT_P_GAIN * difference ) + ( STRAIGHT_D_GAIN * ( difference - old_difference ));
	
	output = Limit_ul( output, 100, -100 );
	
	old_difference 	= difference;
//	old_output 	= output;
	
	return output;
}	
//PD回転制御
float angle_keep_PD( float target_angular_velocity, float now_angular_velocity )
{	
	float output = 0.00,
		difference = 0.00;
	static float old_difference = 0.00;
	
	difference = target_angular_velocity - now_angular_velocity;	
	
	output = ( A_KEEP_P_GAIN * difference ) + ( A_KEEP_D_GAIN * ( difference - old_difference ));
	
	output = Limit_ul( output, 100, -100 );
	
	old_difference = difference;
	
	return output;
}	
//PD回転制御
float turn_PD( float difference )
{	
	float output = 0.00;
	static float old_difference = 0.00;
	
	output = ( TURN_P_GAIN * difference ) + ( TURN_D_GAIN * ( difference - old_difference ));
	
	output = Limit_ul( output, 100, -100 );
	
	old_difference = difference;
	
	return output;
}	

//カウント（1ms）
void count_plus( void )
{	
	g_timer_count ++;
	g_timer_count2 ++;
}

//カウント（1μs）
void sonic_count_plus(void)
{	
	g_wave_count ++;	
}

float get_average ( int average_number , int number , float now_value )
{
	static float box[5][300] = {{ 0 },{ 0 },{ 0 },{ 0 },{ 0 }};
	float	total_value = 0.00,
		average_value = 0.00;
	int i = 0;
	
	for( i = 0 ; i < ( average_number - 1 ) ; i ++ ){
		box[number][i+1] = box[number][i];
		total_value += box[number][i+1];
	}
	box[number][0] = now_value;
	average_value = ( total_value + now_value ) / average_number;
	
	return average_value;
}

float distance_position_xy( float now_x, float now_y, float target_x, float target_y )
{
	float	target_distance = 0.00;
	
	target_distance = sqrt(( target_x - now_x ) * ( target_x - now_x ) + ( target_y - now_y ) * ( target_y - now_y ));
	
	return	target_distance;
}

float	vertical_via( float now_x, float now_y, float target_x, float target_y )
{
	float	target_degree 	= 0.00,
		gap_degree 	= 0.00,
		vertical_difference_distance = 0.00;
		
	if( now_x == target_x && now_y == target_y ){
		target_degree = 0;
	}else{
		target_degree = atan2( target_y - now_y, target_x - now_x) * 180 / PI;
	}
	gap_degree = target_degree - g_degree;
	
	if( gap_degree > 180 ){
		gap_degree -= 360;
	}else
	if( gap_degree < -180 ){
		gap_degree += 360;
	}
	if( gap_degree > 90 ){
		gap_degree = 180 - gap_degree;
		vertical_difference_distance = -1 * distance_position_xy( now_x, now_y, target_x, target_y) * cos( gap_degree * PI / 180 );
	}else
	if( gap_degree < -90 ){
		gap_degree = -180 - gap_degree;
		vertical_difference_distance = -1 * distance_position_xy( now_x, now_y, target_x, target_y) * cos( gap_degree * PI / 180 );
	}else{
		vertical_difference_distance = distance_position_xy( now_x, now_y, target_x, target_y) * cos( gap_degree * PI / 180 );
	}
	return	vertical_difference_distance;
}

float get_target_angular_velocity(float difference_degree, float a_up,  float a_down,  float max_angular_velocity)
{
	static float target_angular_velocity = 0.0;

	if(difference_degree > 0.5 * max_angular_velocity * max_angular_velocity / a_down ){
			if(target_angular_velocity < max_angular_velocity){
				target_angular_velocity += (a_up * INTERRUPT_TIME);					//v = a * t		時間と																																		１次関数的ではあるが、+=することで加速していく目標速度になる		*/
			}else{
				target_angular_velocity = max_angular_velocity;
			}
	}
	target_angular_velocity =  sqrt(2 * a_down * difference_degree);

	return (target_angular_velocity);
}


float get_target_velocity( float target_x,  float target_y,  float  now_x,  float now_y,  float a_up,  float a_down,  float max_velocity )
{
	float distance_rest = 0.0;
	static float target_velocity = 0.0;

		distance_rest = distance_position_xy ( now_x, now_y, target_x, target_y );/*今と目標地の残りの距離*/

		if( distance_rest > 0.5 * max_velocity * max_velocity / a_down ){				//残りの距離が、設定した減速度・最高速度から計算される距離よりも大きい時																																// Vmax = √(2 * a * l)   →   l =  (Vmax ^ 2) / (2 * a)
				if( target_velocity < max_velocity ){
					target_velocity += ( a_up *  INTERRUPT_TIME );		//v = a * t		時間と加速度から速度を計算する
				}else{
					target_velocity = max_velocity;
				}
		}else{
			target_velocity =  sqrt( 2 * a_down * distance_rest );
		}

	return target_velocity;
}

float Absolute_duty( float max_duty, float target, float now)
{
	float 	output_duty 	= 0.00,
		percentage 	= 0.00;

	if( target != 0.00 ) percentage = fabs( now / target );

	if( percentage >= 1.00 ){
		output_duty = 0.00;
	}else{
		output_duty = max_duty * ( 1.00 - percentage );
	}

	return ( output_duty );
}

int mode_change ( void ){
	static int mode = 0;
	int volume;
	char string[50] = { 0 };
	
	AD0.ADCSR.BIT.ADST = 1; //AD変換開始
	
	while(AD0.ADCSR.BIT.ADST == 1);
		
	volume= AD0.ADDRD;
	
	volume = get_average( 20 , 0 , volume );

	if( volume <= 170 ){
		mode = 0;
		PORT9.DR.BYTE = 1;
	}else if( volume <= 340 ){
		mode = 1;
		PORT9.DR.BYTE = 2;
	}else if( volume <= 510 ){
		mode = 2;
		PORT9.DR.BYTE = 4;
	}else if( volume <= 680 ){
		mode = 3;
		PORT9.DR.BYTE = 8;
	}else if( volume <= 850 ){
		mode = 4;
		PORT9.DR.BYTE = 16;
	}else{
		mode = 5;
		PORT9.DR.BYTE = 32;
	}
//		sprintf( string,"%d\n\r",volume);
//		transmission( string );	
	return mode;
}

int main( void )
{	
	int	swt1 		= 1,			//スイッチ状態
		swt1_x 	= 1,
		status 	= OFF,
		task 		= 1,			//タスク
		end_to_start = 0,
		i		= 0,
		mode	= 0;
		
	float	distance_LEFT		= 0.00,	//左輪進行距離
		distance_RIGHT		= 0.00,	//右輪進行距離
		add_distance_central	= 0.00,	//中心進行距離
		add_distance_LEFT	= 0.00,	//左輪進行距離偏差
		add_distance_RIGHT	= 0.00,	//右輪進行距離偏差
		old_distance_LEFT	= 0.00,	//前回の左輪進行距離
		old_distance_RIGHT	= 0.00,	//前回の右輪進行距離
		vertical_difference_distance = 0.00,	//目標までの垂直距離		
		target_distance		= 0.00;
		
	float	add_degree		= 0.00,	//角度総計（°）
		target_degree		= 0.00, 	//目標座標までの角度（°）
		difference_degree	= 0.00,	//目標までの角度との偏差（°）
		next_degree		= 0.00,
		old_degree		= 0.00,
		add_rad			= 0.00,	//角度総計（rad）
		task_degree		= 0.00,
		start_degree		= 0.00,
		gap_degree		= 0.00;
	
	float	x_coordinates		= 0.00,	//垂直方向座標
		y_coordinates		= 0.00,	//水平方向座標
		target_x			= 0.00,
		target_y			= 0.00;
		
	float	enc_RIGHT		= 0.00,	//右エンコーダ値
		enc_LEFT			= 0.00,	//左エンコーダ値
		old_enc_RIGHT		= 0.00,
		old_enc_LEFT		= 0.00,
		all_enc_RIGHT		= 0.00,
		all_enc_LEFT		= 0.00;
			
	float	straight_output		= 0.00,	//直進出力
		turn_output		= 0.00;	//回転出力
			
	float	velocity			= 0.00,
		target_velocity		= 0.00;
		
	float	angular_velocity			= 0.00,
		target_angular_velocity	= 0.00;		
		
	char string[50] = { 0 };		//文字列宣言
	int count_while = 1,
		task_count = 0;
	
	//構造体の初期化
#ifdef red
	struct coordinates COORD_XY[6][10] = {
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 510, 0 , 's'},
		{ 1530, 1020, 's'},
		{ 1935, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	},
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 510, 0 , 's'},
		{ 1530, 1020, 's'},
		{ 1935, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	},
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 510, 0 , 's'},
		{ 1530, 1020, 's'},
		{ 1935, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	},
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 510, 0 , 's'},
		{ 850 , 340 , 's'},
		{ 1530, 340, 's'},
		{ 1530, 1015, 's'},
		{ 1935, 1015, 's'},
		{ BRAKE, 985 , 'b'}
	},
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 530, 0 , 's'},
		{ 550, 360, 's'},		
		{ 850, 680, 's'},
		{ 850, 1020, 's'},
		{ 1935 , 1080 ,'s'},
		{ BRAKE, 1050 , 'b'}
	},
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 1190, 0, 's'},
		{ 1190, 690, 's'},
		{ 1550, 1050, 's'},
		{ 1935, 1060, 's'},
		{ BRAKE, 1030, 's'}
	}
	};
#endif
#ifdef blue
	struct coordinates COORD_XY[6][10] = {
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 510, 0 , 's'},
		{ 1530, 1020, 's'},
		{ 1935, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	},
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 510, 0 , 's'},
		{ 1530, 1020, 's'},
		{ 1935, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	},
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 510, 0 , 's'},
		{ 1530, 1020, 's'},
		{ 1935, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	},
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 510, 0 , 's'},
		{ 1530 , 510 , 's'},
		{ 1530, 1020, 's'},
		{ 1935, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	},
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 530, 0 , 's'},
		{ 550, 360, 's'},		
		{ 850, 680, 's'},
		{ 850, 1020, 's'},
		{ 1935 , 1120 ,'s'},
		{ BRAKE, 1090 , 'b'}
	},
	{
		{ 153, 0, 's'},
		{ 153, 0, 's'},
		{ 1190, 0, 's'},
		{ 1190, 700, 's'},
		{ 1530, 1040, 's'},
		{ 1935, 1040, 's'},
		{ BRAKE, 1010, 's'}
	}
	};
#endif
	init_clock();
	init_cmt0();
//	init_cmt1();
	init_serial();
	init_encoder_MTU1();
	init_encoder_MTU2();
	init_MTU6_pwm();
	init_AD();
	init_dafault();
	
	while( 1 ){
		count_while++;
		 // 5ms毎に以下の動作を行う
		if( g_timer_count >= 5 ){	
			g_timer_count = 0;
			
			//スイッチがOFFのとき以下の動作をおこなう
			if( status == OFF ){
				swt1 = chata_down(( 1 -PLUS_SWITCH ), 0 ); //スイッチの状態の確認
				
				if( swt1 == 0 && swt1_x == 0 ){
					status = ON;
					swt1_x = 1;
					TCNT_RIGHT = 0;
					TCNT_LEFT = 0;
					x_coordinates = 153;
					y_coordinates = 0;
					g_over_MTU1_count   = 0,	//左オーバーフローカウント
					g_under_MTU1_count  = 0,	//左アンダーフローカウント
					g_over_MTU2_count  = 0,	//右オーバーフローカウント
					g_under_MTU2_count = 0;	//右アンダーフローカウント
				}else
				if( swt1 == 1 ){
					swt1_x = 0;
				}
				mode = mode_change();
			}else
			//スイッチがONのとき以下の動作をおこなう
			if( status == ON ){
//				PORT9.DR.BIT.B1 = 1;
				enc_LEFT  =  TCNT_LEFT  + ( 65536 * g_over_MTU1_count)  + ( ( -65536 ) * g_under_MTU1_count  ); //左エンコーダーの値
				enc_RIGHT =  TCNT_RIGHT + ( 65536 * g_over_MTU2_count) + ( ( -65536 ) * g_under_MTU2_count ); //右エンコーダーの値
			
/*				if( mode == 5 && task == 5 && end_to_start == 0 ){
					all_enc_RIGHT += (( enc_LEFT - old_enc_LEFT )+( enc_RIGHT - old_enc_RIGHT)) / 2;
					all_enc_LEFT += (( enc_LEFT - old_enc_LEFT )+( enc_RIGHT - old_enc_RIGHT)) / 2;
/*					if( ( enc_LEFT - old_enc_LEFT ) >= ( enc_RIGHT - old_enc_RIGHT ) ){
						all_enc_RIGHT += enc_LEFT - old_enc_LEFT;
						all_enc_LEFT += enc_LEFT - old_enc_LEFT;						
					}else if ( (enc_RIGHT - old_enc_RIGHT) > (enc_LEFT - old_enc_LEFT) ){
						all_enc_LEFT += enc_RIGHT - old_enc_RIGHT;
						all_enc_RIGHT += enc_RIGHT - old_enc_RIGHT;
					}*/
//				}else{
					all_enc_LEFT += enc_LEFT - old_enc_LEFT;
					all_enc_RIGHT += enc_RIGHT - old_enc_RIGHT;	
//				}
				distance_LEFT 	= -1 * ( all_enc_LEFT * PI * DIAMETER_LEFT)  / (PULSE_LEFT  * 4 ); //左輪進行mm
				distance_RIGHT 	= ( all_enc_RIGHT * PI * DIAMETER_RIGHT) / (PULSE_RIGHT * 4 ); //右輪進行mm
							
				add_distance_LEFT  = distance_LEFT  - old_distance_LEFT; //左輪偏差			
				add_distance_RIGHT = distance_RIGHT - old_distance_RIGHT; //右輪偏差
				
				add_distance_central = ( add_distance_LEFT + add_distance_RIGHT ) / 2;
				
				g_rad = ( distance_RIGHT - distance_LEFT ) / TREAD; //角度（radian）
				add_rad += ( add_distance_RIGHT - add_distance_LEFT ) / TREAD; //累計角度（radian）
				g_degree = g_rad * 180 / PI; //角度（°）
				add_degree = add_degree + 180 * ( add_distance_RIGHT - add_distance_LEFT ) / ( TREAD * PI ); //累計角度（°）
				
				// -180 < degree < 180
				while ( g_degree > 180 ){
					g_degree = -360 + g_degree;
				}
				while ( g_degree < -180 ){
					g_degree = g_degree + 360;
				}				
				velocity = add_distance_central * 1000 / 5; //速度(mm/s)
				angular_velocity =  ( g_degree - old_degree ) * 1000 / 5; //角速度				
				
				x_coordinates += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * cos( add_rad ) ; //ｘ座標現在位置（縦）
				y_coordinates += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * sin( add_rad ) ; //y座標現在位置（横）
				
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------				
				if( COORD_XY[mode][task].MODE == 's' && end_to_start == 0){
					//atan2エラー回避
					if( COORD_XY[mode][task].X == x_coordinates && COORD_XY[mode][task].Y == y_coordinates ){
						target_degree = 0;
					}else{								
						target_degree = atan2( COORD_XY[mode][task].Y - y_coordinates , COORD_XY[mode][task].X - x_coordinates ) * 180 / PI; //現在値から見た目的地の角度
					}
									
					vertical_difference_distance = vertical_via( x_coordinates, y_coordinates, COORD_XY[mode][task].X, COORD_XY[mode][task].Y );
					
					difference_degree = target_degree - g_degree; //角度偏差（°）

					// -180 < difference_degree < 180
					if ( difference_degree > 180 ){
						difference_degree = -360 + difference_degree;
					}else
					if ( difference_degree < -180 ){
						difference_degree = difference_degree + 360;
					}				
					
					target_velocity = get_target_velocity( COORD_XY[mode][task].X, COORD_XY[mode][task].Y , x_coordinates, y_coordinates, 100000, V_MAX * V_MAX / ( 0.3 * distance_position_xy ( COORD_XY[mode][task - 1].X, COORD_XY[mode][task - 1].Y, COORD_XY[mode][task].X, COORD_XY[mode][task].Y ) * 2 ), V_MAX);
					
					if( vertical_difference_distance <= 0 ){
						target_velocity = ( -1 ) * target_velocity;
					}
					
					if(g_timer_count2 >= 1000 ){
						g_timer_count2 = 0;
						sprintf(string," %.3lf   ,%.3lf ,%.3lf   ,%.3lf  %d\n\r",x_coordinates,y_coordinates ,g_degree, old_degree,task);
						transmission( string );
					}

					//目的地到着判定				//目的地到着判定　垂直距離を見る
					if( vertical_difference_distance <= 5 ){
						if( COORD_XY[mode][task+1].MODE == 'c' ){
							task++;
							start_degree = g_degree;
							if( COORD_XY[mode][task].Y == COORD_XY[mode][task - 1].Y && COORD_XY[mode][task].X == COORD_XY[mode][task - 1].X ){
								task_degree = 0;
							}else{
								task_degree = atan2( COORD_XY[mode][task].Y - COORD_XY[mode][task - 1].Y , COORD_XY[mode][task].X - COORD_XY[mode][task - 1].X ) * 180 / PI;
							}
							
							gap_degree = task_degree - start_degree;
							if( gap_degree > 180 ){
								gap_degree -= 360;
							}else
							if( gap_degree < -180 ){
								gap_degree += 360;
							}
							target_distance = distance_position_xy( COORD_XY[mode][task - 1].X , COORD_XY[mode][task - 1].Y , COORD_XY[mode][task].X , COORD_XY[mode][task].Y );
												
							if( COORD_XY[mode][task + 1].Y == COORD_XY[mode][task].Y && COORD_XY[mode][task + 1].X == COORD_XY[mode][task].X ){
								next_degree = 0;
							}else if( COORD_XY[mode][task+1].X == BRAKE ){
								next_degree = -45;
							}else{
								next_degree = atan2( COORD_XY[mode][task + 1].Y - COORD_XY[mode][task].Y , COORD_XY[mode][task + 1].X - COORD_XY[mode][task].X ) *180 / PI;
							}
						}else{
							end_to_start = 1; //目標地点到着フラグ
						}
						sprintf( string,"%.3lf,%.3lf,%.3lf,%d\n\r",x_coordinates,y_coordinates,g_degree,task);
						transmission( string );
					}
					straight_output = straight_PD( target_velocity , velocity ); //直進PD制御
					turn_output = turn_PD ( difference_degree );
				}else
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
				if( COORD_XY[mode][task].MODE == 'c' && end_to_start == 0){
					target_x = i * target_distance * cos( ( start_degree + ( i * gap_degree) / PLOT ) * PI / 180) / PLOT + COORD_XY[mode][task - 1].X;
					target_y = i * target_distance * cos( ( start_degree + (( i * gap_degree) / PLOT ) - 90) * PI / 180) / PLOT + COORD_XY[mode][task - 1].Y;

					//atan2エラー回避
					if( target_x == x_coordinates && target_y == y_coordinates ){
						target_degree = 0;
					}else{								
						target_degree = atan2( target_y - y_coordinates , target_x - x_coordinates ) * 180 / PI; //現在値から見た目的地の角度
					}
									
					vertical_difference_distance = vertical_via( x_coordinates, y_coordinates, target_x, target_y );
					
					difference_degree = target_degree - g_degree; //角度偏差（°）
					
					// -180 < difference_degree < 180
					if ( difference_degree > 180 ){
						difference_degree = -360 + difference_degree;
					}else
					if ( difference_degree < -180 ){
						difference_degree = difference_degree + 360;
					}				
					target_velocity = get_target_velocity( COORD_XY[mode][4].X , COORD_XY[mode][4].Y , x_coordinates, y_coordinates, 100000, V_MAX * V_MAX / ( 0.2 * distance_position_xy ( COORD_XY[mode][0].X , COORD_XY[mode][0].Y , COORD_XY[mode][4].X , COORD_XY[mode][4].Y ) * 2 ), V_MAX);				
					
					if( vertical_difference_distance <= 0 ){
						target_velocity = ( -1 ) * target_velocity;
					}
					
					if(g_timer_count2 >= 1000 ){
						g_timer_count2 = 0;
						sprintf(string," %.3lf   ,%.3lf ,%.3lf   ,%.3lf  %d  %d\n\r",target_x , target_y ,start_degree + ( i * gap_degree) / 10, gap_degree,task,i);
						transmission( string );
					}

					if( vertical_difference_distance <= 1 && vertical_difference_distance >= -1 && i < PLOT){
						i ++;
					}else			
					if( vertical_difference_distance <= 2 && i >= PLOT ){
						if( COORD_XY[mode][task + 1].MODE == 'c' ){
							task ++;
							start_degree = g_degree;
							i= 0;

							if( COORD_XY[mode][task].Y == COORD_XY[mode][task - 1].Y && COORD_XY[mode][task].X == COORD_XY[mode][task - 1].X ){
								task_degree = 0;
							}else{
								task_degree = atan2( COORD_XY[mode][task].Y - COORD_XY[mode][task - 1].Y , COORD_XY[mode][task].X - COORD_XY[mode][task - 1].X ) * 180 / PI;
							}
							
							gap_degree = task_degree - start_degree;
							if( gap_degree > 180 ){
								gap_degree -= 360;
							}else
							if( gap_degree < -180 ){
								gap_degree += 360;
							}

							target_distance = distance_position_xy( COORD_XY[mode][task - 1].X , COORD_XY[mode][task - 1].Y , COORD_XY[mode][task].X , COORD_XY[mode][task].Y );
						}else{
							end_to_start = 1;
						}
					}
					
					straight_output = straight_PD( target_velocity , velocity ); //直進PD制御
					turn_output = turn_PD ( difference_degree );
				}
				if( COORD_XY[mode][task].MODE == 'b' ){
					FRONT_LEFT = 1;
					BACK_LEFT = 1;
					FRONT_RIGHT = 1;
					BACK_RIGHT = 1;
					pwm( 99, 99 );
				}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
				if( end_to_start == 1){
					PORT9.DR.BIT.B1 = 1;
					vertical_difference_distance = vertical_via( x_coordinates, y_coordinates, COORD_XY[mode][task].X, COORD_XY[mode][task].Y );
					
					difference_degree = next_degree - g_degree;
					
					// -180 < difference_degree < 180
					if ( difference_degree > 180 ){
						difference_degree = -360 + difference_degree;
					}else
					if ( difference_degree < -180 ){
						difference_degree = difference_degree + 360;
					}				
					
					target_velocity = get_target_velocity( COORD_XY[mode][task].X, COORD_XY[mode][task].Y , x_coordinates, y_coordinates, 100000, V_MAX * V_MAX / ( 0.2 * distance_position_xy ( COORD_XY[mode][task - 1].X, COORD_XY[mode][task - 1].Y, COORD_XY[mode][task].X, COORD_XY[mode][task].Y ) * 2 ), V_MAX);
					target_angular_velocity = get_target_angular_velocity(  fabs( difference_degree ), 100000,2000, V_ANGULAR_MAX );
					
					if( vertical_difference_distance <= 0 ){
						target_velocity = ( -1 ) * target_velocity;
					}
					if( difference_degree <= 0 ){
						target_angular_velocity = ( -1 ) * target_angular_velocity;
					}

					//目標地点に着いてから範囲内の角度に入ると以下に入る
					if(vertical_difference_distance <= 1 && vertical_difference_distance >= -1 && difference_degree <= 1 && difference_degree >= -1 ){
						task_count ++;
					}else{
						//task_count = 0;
					}
					if( task_count >=20 && COORD_XY[mode][task].X != BRAKE){
						task_count = 0;
						if( COORD_XY[mode][task+1].X != BRAKE){
							end_to_start = 0;
						}
						task ++;
						if( COORD_XY[mode][task].X == BRAKE ){
//						}else if( COORD_XY[mode][task+1].X == BRAKE ){
//							next_degree = -45;
						}else{
							next_degree = atan2( COORD_XY[mode][task + 1].Y - COORD_XY[mode][task].Y , COORD_XY[mode][task + 1].X - COORD_XY[mode][task].X ) *180 / PI;
						}
					}
					straight_output = straight_PD( target_velocity , velocity ); //直進PD制御
					turn_output = angle_keep_PD ( target_angular_velocity, angular_velocity ); //回転PD制御
				}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
				old_distance_LEFT  = distance_LEFT;
				old_distance_RIGHT = distance_RIGHT;
				old_degree = g_degree;
				old_enc_LEFT = enc_LEFT;
				old_enc_RIGHT = enc_RIGHT;
				
				turn_output = Limit_ul( turn_output , 100 , -100 );
				straight_output =  Limit_ul( straight_output , 100 , -100 );
				Move( straight_output + ( -1 ) * turn_output ,straight_output + turn_output ); //モータ関数	
//				Move( turn_PD( g_degree ) , ( -1 ) * turn_PD( g_degree ));
				
//				sprintf(string," %lf  %lf %lf\n\r",distance_LEFT,distance_RIGHT , g_degree);
//				transmission( string );
				//ブレーキ
				if( COORD_XY[mode][task].X == BRAKE/* && fabs( difference_degree ) < 2 */){
					FRONT_LEFT = 1;
					BACK_LEFT = 1;
					FRONT_RIGHT = 1;
					BACK_RIGHT = 1;
					pwm( 99, 99 );
					break;
				}
			}
//		sprintf( string,"%d\n\r",count_while);
//		transmission( string );
//		count_while = 0;
		}
	}
}