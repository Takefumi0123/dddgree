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
#define ON				1				//�X�C�b�`���
#define OFF				0
#define INTERRUPT_TIME  	0.005
#define PLUS_SWITCH		PORT8.PORT.BIT.B2		//�X�C�b�`�|�[�g
#define PI					3.141592653			//�~����
#define TCNT_RIGHT			MTU2.TCNT			//�E�G���R�[�_TCNT
#define TCNT_LEFT			MTU1.TCNT			//���G ���R�[�_TCNT
#define DIAMETER_LEFT		21.90608333				//���G���R�[�_���a
#define DIAMETER_RIGHT		22.09391667				//�E�G���R�[�_���a
#define PULSE_LEFT			100					//���G���R�[�_�p���X
#define PULSE_RIGHT		100					//�E�G���R�[�_�p���X
#define TREAD				228					//�G���R�[�_�ԋ���
#define TURN_P_GAIN		20					//��]�o��P�Q�C��
#define TURN_D_GAIN		30					//��]�o��D�Q�C��
#define STRAIGHT_P_GAIN	0.8					//���i�o��P�Q�C��
#define STRAIGHT_D_GAIN	1.2					//���i�o��D�Q�C��
#define A_KEEP_P_GAIN		20
#define A_KEEP_D_GAIN		25
#define FRONT_LEFT			PORTB.DR.BIT.B1		//������]�o�͋���
#define BACK_LEFT			PORTB.DR.BIT.B3		//���t��]�o�͋���
#define FRONT_RIGHT		PORTA.DR.BIT.B2		//�E����]�o�͋���
#define BACK_RIGHT			PORTB.DR.BIT.B2		//�E�t��]�o�͋���
#define BRAKE				1965
#define V_MAX				450
#define V_ANGULAR_MAX		200
#define PLOT				7

int	g_timer_count       		=	0,	//1ms�J�E���g
	g_timer_count2     		=	0,	//1ms�J�E���g
	g_wave_count 			= 	0,	//1��s�J�E���g
	g_over_MTU1_count  		= 	0,	//���I�[�o�[�t���[�J�E���g
	g_under_MTU1_count		= 	0,	//���A���_�[�t���[�J�E���g
	g_over_MTU2_count  		= 	0,	//�E�I�[�o�[�t���[�J�E���g
	g_under_MTU2_count 	= 	0;	//�E�A���_�[�t���[�J�E���g

float	g_degree	= 0.00,
	g_rad	= 0.00;

//�\���̐錾	
struct 	coordinates
{
	float	X;
	float	Y;
	char	MODE;
};

//�I�[�o�[�t���[�������A�ϐ�g_over��1����
void over_flow_MTU1( void )
{
	g_over_MTU1_count++;
}

//�A���_�[�t���[�������A�ϐ�g_under�ɂP����
void under_flow_MTU1( void )
{
	g_under_MTU1_count++;
}

//�I�[�o�[�t���[�������A�ϐ�g_over��1����
void over_flow_MTU2( void )
{
	g_over_MTU2_count++;
}

//�A���_�[�t���[�������A�ϐ�g_under�ɂP����
void under_flow_MTU2( void )
{
	g_under_MTU2_count++;
}
//�͈͓��o��
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
//PD���i����
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
//PD��]����
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
//PD��]����
float turn_PD( float difference )
{	
	float output = 0.00;
	static float old_difference = 0.00;
	
	output = ( TURN_P_GAIN * difference ) + ( TURN_D_GAIN * ( difference - old_difference ));
	
	output = Limit_ul( output, 100, -100 );
	
	old_difference = difference;
	
	return output;
}	

//�J�E���g�i1ms�j
void count_plus( void )
{	
	g_timer_count ++;
	g_timer_count2 ++;
}

//�J�E���g�i1��s�j
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
				target_angular_velocity += (a_up * INTERRUPT_TIME);					//v = a * t		���Ԃ�																																		�P���֐��I�ł͂��邪�A+=���邱�Ƃŉ������Ă����ڕW���x�ɂȂ�		*/
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

		distance_rest = distance_position_xy ( now_x, now_y, target_x, target_y );/*���ƖڕW�n�̎c��̋���*/

		if( distance_rest > 0.5 * max_velocity * max_velocity / a_down ){				//�c��̋������A�ݒ肵�������x�E�ō����x����v�Z����鋗�������傫����																																// Vmax = ��(2 * a * l)   ��   l =  (Vmax ^ 2) / (2 * a)
				if( target_velocity < max_velocity ){
					target_velocity += ( a_up *  INTERRUPT_TIME );		//v = a * t		���ԂƉ����x���瑬�x���v�Z����
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
	
	AD0.ADCSR.BIT.ADST = 1; //AD�ϊ��J�n
	
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
	int	swt1 		= 1,			//�X�C�b�`���
		swt1_x 	= 1,
		status 	= OFF,
		task 		= 1,			//�^�X�N
		end_to_start = 0,
		i		= 0,
		mode	= 0;
		
	float	distance_LEFT		= 0.00,	//���֐i�s����
		distance_RIGHT		= 0.00,	//�E�֐i�s����
		add_distance_central	= 0.00,	//���S�i�s����
		add_distance_LEFT	= 0.00,	//���֐i�s�����΍�
		add_distance_RIGHT	= 0.00,	//�E�֐i�s�����΍�
		old_distance_LEFT	= 0.00,	//�O��̍��֐i�s����
		old_distance_RIGHT	= 0.00,	//�O��̉E�֐i�s����
		vertical_difference_distance = 0.00,	//�ڕW�܂ł̐�������		
		target_distance		= 0.00;
		
	float	add_degree		= 0.00,	//�p�x���v�i���j
		target_degree		= 0.00, 	//�ڕW���W�܂ł̊p�x�i���j
		difference_degree	= 0.00,	//�ڕW�܂ł̊p�x�Ƃ̕΍��i���j
		next_degree		= 0.00,
		old_degree		= 0.00,
		add_rad			= 0.00,	//�p�x���v�irad�j
		task_degree		= 0.00,
		start_degree		= 0.00,
		gap_degree		= 0.00;
	
	float	x_coordinates		= 0.00,	//�����������W
		y_coordinates		= 0.00,	//�����������W
		target_x			= 0.00,
		target_y			= 0.00;
		
	float	enc_RIGHT		= 0.00,	//�E�G���R�[�_�l
		enc_LEFT			= 0.00,	//���G���R�[�_�l
		old_enc_RIGHT		= 0.00,
		old_enc_LEFT		= 0.00,
		all_enc_RIGHT		= 0.00,
		all_enc_LEFT		= 0.00;
			
	float	straight_output		= 0.00,	//���i�o��
		turn_output		= 0.00;	//��]�o��
			
	float	velocity			= 0.00,
		target_velocity		= 0.00;
		
	float	angular_velocity			= 0.00,
		target_angular_velocity	= 0.00;		
		
	char string[50] = { 0 };		//������錾
	int count_while = 1,
		task_count = 0;
	
	//�\���̂̏�����
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
		 // 5ms���Ɉȉ��̓�����s��
		if( g_timer_count >= 5 ){	
			g_timer_count = 0;
			
			//�X�C�b�`��OFF�̂Ƃ��ȉ��̓���������Ȃ�
			if( status == OFF ){
				swt1 = chata_down(( 1 -PLUS_SWITCH ), 0 ); //�X�C�b�`�̏�Ԃ̊m�F
				
				if( swt1 == 0 && swt1_x == 0 ){
					status = ON;
					swt1_x = 1;
					TCNT_RIGHT = 0;
					TCNT_LEFT = 0;
					x_coordinates = 153;
					y_coordinates = 0;
					g_over_MTU1_count   = 0,	//���I�[�o�[�t���[�J�E���g
					g_under_MTU1_count  = 0,	//���A���_�[�t���[�J�E���g
					g_over_MTU2_count  = 0,	//�E�I�[�o�[�t���[�J�E���g
					g_under_MTU2_count = 0;	//�E�A���_�[�t���[�J�E���g
				}else
				if( swt1 == 1 ){
					swt1_x = 0;
				}
				mode = mode_change();
			}else
			//�X�C�b�`��ON�̂Ƃ��ȉ��̓���������Ȃ�
			if( status == ON ){
//				PORT9.DR.BIT.B1 = 1;
				enc_LEFT  =  TCNT_LEFT  + ( 65536 * g_over_MTU1_count)  + ( ( -65536 ) * g_under_MTU1_count  ); //���G���R�[�_�[�̒l
				enc_RIGHT =  TCNT_RIGHT + ( 65536 * g_over_MTU2_count) + ( ( -65536 ) * g_under_MTU2_count ); //�E�G���R�[�_�[�̒l
			
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
				distance_LEFT 	= -1 * ( all_enc_LEFT * PI * DIAMETER_LEFT)  / (PULSE_LEFT  * 4 ); //���֐i�smm
				distance_RIGHT 	= ( all_enc_RIGHT * PI * DIAMETER_RIGHT) / (PULSE_RIGHT * 4 ); //�E�֐i�smm
							
				add_distance_LEFT  = distance_LEFT  - old_distance_LEFT; //���֕΍�			
				add_distance_RIGHT = distance_RIGHT - old_distance_RIGHT; //�E�֕΍�
				
				add_distance_central = ( add_distance_LEFT + add_distance_RIGHT ) / 2;
				
				g_rad = ( distance_RIGHT - distance_LEFT ) / TREAD; //�p�x�iradian�j
				add_rad += ( add_distance_RIGHT - add_distance_LEFT ) / TREAD; //�݌v�p�x�iradian�j
				g_degree = g_rad * 180 / PI; //�p�x�i���j
				add_degree = add_degree + 180 * ( add_distance_RIGHT - add_distance_LEFT ) / ( TREAD * PI ); //�݌v�p�x�i���j
				
				// -180 < degree < 180
				while ( g_degree > 180 ){
					g_degree = -360 + g_degree;
				}
				while ( g_degree < -180 ){
					g_degree = g_degree + 360;
				}				
				velocity = add_distance_central * 1000 / 5; //���x(mm/s)
				angular_velocity =  ( g_degree - old_degree ) * 1000 / 5; //�p���x				
				
				x_coordinates += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * cos( add_rad ) ; //�����W���݈ʒu�i�c�j
				y_coordinates += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * sin( add_rad ) ; //y���W���݈ʒu�i���j
				
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------				
				if( COORD_XY[mode][task].MODE == 's' && end_to_start == 0){
					//atan2�G���[���
					if( COORD_XY[mode][task].X == x_coordinates && COORD_XY[mode][task].Y == y_coordinates ){
						target_degree = 0;
					}else{								
						target_degree = atan2( COORD_XY[mode][task].Y - y_coordinates , COORD_XY[mode][task].X - x_coordinates ) * 180 / PI; //���ݒl���猩���ړI�n�̊p�x
					}
									
					vertical_difference_distance = vertical_via( x_coordinates, y_coordinates, COORD_XY[mode][task].X, COORD_XY[mode][task].Y );
					
					difference_degree = target_degree - g_degree; //�p�x�΍��i���j

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

					//�ړI�n��������				//�ړI�n��������@��������������
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
							end_to_start = 1; //�ڕW�n�_�����t���O
						}
						sprintf( string,"%.3lf,%.3lf,%.3lf,%d\n\r",x_coordinates,y_coordinates,g_degree,task);
						transmission( string );
					}
					straight_output = straight_PD( target_velocity , velocity ); //���iPD����
					turn_output = turn_PD ( difference_degree );
				}else
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
				if( COORD_XY[mode][task].MODE == 'c' && end_to_start == 0){
					target_x = i * target_distance * cos( ( start_degree + ( i * gap_degree) / PLOT ) * PI / 180) / PLOT + COORD_XY[mode][task - 1].X;
					target_y = i * target_distance * cos( ( start_degree + (( i * gap_degree) / PLOT ) - 90) * PI / 180) / PLOT + COORD_XY[mode][task - 1].Y;

					//atan2�G���[���
					if( target_x == x_coordinates && target_y == y_coordinates ){
						target_degree = 0;
					}else{								
						target_degree = atan2( target_y - y_coordinates , target_x - x_coordinates ) * 180 / PI; //���ݒl���猩���ړI�n�̊p�x
					}
									
					vertical_difference_distance = vertical_via( x_coordinates, y_coordinates, target_x, target_y );
					
					difference_degree = target_degree - g_degree; //�p�x�΍��i���j
					
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
					
					straight_output = straight_PD( target_velocity , velocity ); //���iPD����
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

					//�ڕW�n�_�ɒ����Ă���͈͓��̊p�x�ɓ���ƈȉ��ɓ���
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
					straight_output = straight_PD( target_velocity , velocity ); //���iPD����
					turn_output = angle_keep_PD ( target_angular_velocity, angular_velocity ); //��]PD����
				}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
				old_distance_LEFT  = distance_LEFT;
				old_distance_RIGHT = distance_RIGHT;
				old_degree = g_degree;
				old_enc_LEFT = enc_LEFT;
				old_enc_RIGHT = enc_RIGHT;
				
				turn_output = Limit_ul( turn_output , 100 , -100 );
				straight_output =  Limit_ul( straight_output , 100 , -100 );
				Move( straight_output + ( -1 ) * turn_output ,straight_output + turn_output ); //���[�^�֐�	
//				Move( turn_PD( g_degree ) , ( -1 ) * turn_PD( g_degree ));
				
//				sprintf(string," %lf  %lf %lf\n\r",distance_LEFT,distance_RIGHT , g_degree);
//				transmission( string );
				//�u���[�L
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