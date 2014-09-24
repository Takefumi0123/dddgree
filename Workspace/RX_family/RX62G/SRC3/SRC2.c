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

#define PATURN6
#define ON			1				//�X�C�b�`���
#define OFF			0
#define INTERRUPT_TIME  	0.005
#define PLUS_SWITCH		PORT8.PORT.BIT.B2		//�X�C�b�`�|�[�g
#define PI			3.141592653			//�~����
#define TCNT_RIGHT		MTU2.TCNT			//�E�G���R�[�_TCNT
#define TCNT_LEFT		MTU1.TCNT			//���G���R�[�_TCNT
#define DIAMETER_LEFT		22.343				//���G���R�[�_���a
#define DIAMETER_RIGHT		22.389				//�E�G���R�[�_���a
#define PULSE_LEFT		100					//���G���R�[�_�p���X
#define PULSE_RIGHT		100					//�E�G���R�[�_�p���X
#define TREAD			228					//�G���R�[�_�ԋ���
#define TURN_P_GAIN		4					//��]�o��P�Q�C��
#define TURN_D_GAIN		5					//��]�o��D�Q�C��
#define STRAIGHT_P_GAIN		0.8					//���i�o��P�Q�C��
#define STRAIGHT_D_GAIN		2					//���i�o��D�Q�C��
#define A_KEEP_P_GAIN		15
#define A_KEEP_D_GAIN		25
#define FRONT_LEFT		PORTB.DR.BIT.B1		//������]�o�͋���
#define BACK_LEFT		PORTB.DR.BIT.B3		//���t��]�o�͋���
#define FRONT_RIGHT		PORTA.DR.BIT.B2		//�E����]�o�͋���
#define BACK_RIGHT		PORTB.DR.BIT.B2		//�E�t��]�o�͋���
#define BRAKE			3000
#define V_MAX			500
#define V_ANGULAR_MAX		200
#define PLOT			7
#define TARGET_DISTANCE		300

int	g_timer_count       	=	0,	//1ms�J�E���g
	g_timer_count2     	=	0,	//1ms�J�E���g
	g_wave_count 		= 	0,	//1��s�J�E���g
	g_over_MTU1_count  	= 	0,	//���I�[�o�[�t���[�J�E���g
	g_under_MTU1_count	= 	0,	//���A���_�[�t���[�J�E���g
	g_over_MTU2_count  	= 	0,	//�E�I�[�o�[�t���[�J�E���g
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

int main( void )
{	
	int	swt1 		= 1,			//�X�C�b�`���
		swt1_x 	= 1,
		status 	= OFF,
		task 		= 1;			//�^�X�N
		
	float	distance_LEFT		= 0.00,	//���֐i�s����
		distance_RIGHT		= 0.00,	//�E�֐i�s����
		add_distance_central	= 0.00,	//���S�i�s����
		add_distance_LEFT	= 0.00,	//���֐i�s�����΍�
		add_distance_RIGHT	= 0.00,	//�E�֐i�s�����΍�
		old_distance_LEFT	= 0.00,	//�O��̍��֐i�s����
		old_distance_RIGHT	= 0.00,	//�O��̉E�֐i�s����
		vertical_difference_distance = 0.00,	//�ڕW�܂ł̐�������		
		difference_distance = 0.00,
		halfway_distance = 0.00;
		
	float	add_degree		= 0.00,	//�p�x���v�i���j
		target_degree		= 0.00, 	//�ڕW���W�܂ł̊p�x�i���j
		difference_degree	= 0.00,	//�ڕW�܂ł̊p�x�Ƃ̕΍��i���j
		add_rad			= 0.00;	//�p�x���v�irad�j

	
	float	x_coordinates		= 0.00,	//�����������W
		y_coordinates		= 0.00,	//�����������W
		target_x			= 0.00,
		target_y			= 0.00;
		
	float	enc_RIGHT		= 0.00,	//�E�G���R�[�_�l
		enc_LEFT			= 0.00;	//���G���R�[�_�l
			
	float	straight_output		= 0.00,	//���i�o��
		turn_output		= 0.00;	//��]�o��	
		
	char string[50] = { 0 };		//������錾
	int count_while = 1;
	
	//�\���̂̏�����
// PATURN1 2 3

#ifdef  PATURN1 
	struct coordinates COOD_XY[10] = {
		{ 36, 0, 's'},
		{ 36, 0, 's'},
		{ 510, 0 , 's'},
		{ 1530, 1020, 's'},
		{ 1905, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	};
#endif
#ifdef  PATURN2 
	struct coordinates COOD_XY[10] = {
		{ 36, 0, 's'},
		{ 36, 0, 's'},
		{ 510, 0 , 's'},
		{ 1530, 1020, 's'},
		{ 1905, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	};
#endif
#ifdef  PATURN3 
	struct coordinates COOD_XY[10] = {
		{ 36, 0, 's'},
		{ 36, 0, 's'},
		{ 510, 0 , 's'},
		{ 1530, 1020, 's'},
		{ 1905, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	};
#endif
// PATURN 4
#ifdef PATURN4
	struct coordinates COOD_XY[10] = {
		{ 36, 0, 's'},
		{ 36, 0, 's'},
		{ 510, 0 , 's'},
		{ 1190 , 340 , 's'},
		{ 1530 , 510 , 's'},		
		{ 1530, 960, 's'},
		{ 1905, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	};
#endif

// PATURN5
#ifdef PATURN5
	struct coordinates COOD_XY[10] = {
		{ 36, 0, 's'},
		{ 36, 0, 's'},
		{ 850, 0, 's'},
		{ 1190, 310, 's'},
		{ 1190, 920, 's'},
		{ 1905 , 1020 ,'s'},
		{ BRAKE, 1020 , 'b'}
	};
#endif

// PATURN6
#ifdef PATURN6
	struct coordinates COOD_XY[10] = {
		{ 36, 0, 's'},
		{ 36, 0, 's'},
		{ 850, 0 , 's'},
		{ 1190, 310,'s'},
		{ 1190, 680,'s'},
		{ 1530, 1020, 's'},
		{ 1905, 1020, 's'},
		{ BRAKE, 1020 , 'b'}
	};
#endif	
	init_clock();
	init_cmt0();
//	init_cmt1();
	init_serial();
	init_encoder_MTU1();
	init_encoder_MTU2();
	init_MTU6_pwm();
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
					x_coordinates = 36;
					y_coordinates = 0;
					g_over_MTU1_count   = 0,	//���I�[�o�[�t���[�J�E���g
					g_under_MTU1_count  = 0,	//���A���_�[�t���[�J�E���g
					g_over_MTU2_count  = 0,	//�E�I�[�o�[�t���[�J�E���g
					g_under_MTU2_count = 0;	//�E�A���_�[�t���[�J�E���g
				}else
				if( swt1 == 1 ){
					swt1_x = 0;
				}
			}else
			//�X�C�b�`��ON�̂Ƃ��ȉ��̓���������Ȃ�
			if( status == ON ){
//				PORT9.DR.BIT.B1 = 1;
				enc_LEFT  =  TCNT_LEFT  + ( 65536 * g_over_MTU1_count)  + ( ( -65536 ) * g_under_MTU1_count  ); //���G���R�[�_�[�̒l
				enc_RIGHT =  TCNT_RIGHT + ( 65536 * g_over_MTU2_count) + ( ( -65536 ) * g_under_MTU2_count ); //�E�G���R�[�_�[�̒l
				distance_LEFT 	= -1 * ( enc_LEFT * PI * DIAMETER_LEFT)  / (PULSE_LEFT  * 4 ); //���֐i�smm
				distance_RIGHT 	= ( enc_RIGHT * PI * DIAMETER_RIGHT) / (PULSE_RIGHT * 4 ); //�E�֐i�smm
							
				add_distance_LEFT  = distance_LEFT  - old_distance_LEFT; //���֕΍�			
				add_distance_RIGHT = distance_RIGHT - old_distance_RIGHT; //�E�֕΍�
				
				add_distance_central = ( add_distance_LEFT + add_distance_RIGHT ) / 2;
				
				halfway_distance += add_distance_central;
				
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
				
				x_coordinates += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * cos( add_rad ) ; //�����W���݈ʒu�i�c�j
				y_coordinates += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * sin( add_rad ) ; //y���W���݈ʒu�i���j
				
				if( COOD_XY[task].Y == COOD_XY[task - 1].Y && COOD_XY[task].X == COOD_XY[task - 1].X ){
					target_x = x_coordinates;
					target_y = y_coordinates;
				}else{
					target_x = COOD_XY[task - 1].X + ( halfway_distance + 100) * cos( atan2( COOD_XY[task].Y - COOD_XY[task - 1].Y , COOD_XY[task].X - COOD_XY[task - 1].X )); 
					target_y = COOD_XY[task - 1].Y + ( halfway_distance + 100 )* sin( atan2( COOD_XY[task].Y - COOD_XY[task - 1].Y , COOD_XY[task].X - COOD_XY[task - 1].X )); 
				}
				if( COOD_XY[task + 1].X == BRAKE ){
					target_x = COOD_XY[task].X;
					target_y = COOD_XY[task].Y;
				}
				//atan2�G���[���
				if( COOD_XY[task].X == x_coordinates && COOD_XY[task].Y == y_coordinates ){
					target_degree = 0;
				}else{
					target_degree = atan2( target_y - y_coordinates , target_x - x_coordinates ) * 180 / PI; //���ݒl���猩���ړI�n�̊p�x
				}
				
				difference_distance = distance_position_xy( x_coordinates, y_coordinates, target_x, target_y);
				
				if( COOD_XY[task].X == BRAKE ){
					vertical_difference_distance = vertical_via( x_coordinates, y_coordinates, COOD_XY[task - 1].X, COOD_XY[task - 1].Y );
				}else{
					vertical_difference_distance = vertical_via( x_coordinates, y_coordinates, COOD_XY[task].X, COOD_XY[task].Y );
				}
				difference_degree = target_degree - g_degree; //�p�x�΍��i���j

				// -180 < difference_degree < 180
				if ( difference_degree > 180 ){
					difference_degree = -360 + difference_degree;
				}else
				if ( difference_degree < -180 ){
					difference_degree = difference_degree + 360;
				}
								
				if(g_timer_count2 >= 500 ){
					g_timer_count2 = 0;
					sprintf(string," %.3lf   ,%.3lf ,%.3lf   ,%.3lf  ,%.3lf   ,%d\n\r",x_coordinates,y_coordinates ,target_x,target_y,vertical_difference_distance, task);
					transmission( string );
				}

				//�ړI�n��������				//�ړI�n��������@��������������
				if( ((vertical_difference_distance <= 180 && COOD_XY[task+1].X != BRAKE) || ( fabs( vertical_difference_distance ) <= 2 && COOD_XY[task+1].X == BRAKE)) && COOD_XY[task].X != BRAKE ){
					halfway_distance = 0;
					task++;
					sprintf( string,"%.3lf,%.3lf,%.3lf,%.3lf\n\r",x_coordinates,y_coordinates,target_x,target_y);
					transmission( string );
				}
				if( COOD_XY[task+1].X == BRAKE || COOD_XY[task].X == BRAKE ){
					straight_output = straight_PD( vertical_difference_distance , 0 );
				}else{
					straight_output = straight_PD( difference_distance , 0 ); //���iPD����
				}
				turn_output = turn_PD ( difference_degree );

				old_distance_LEFT  = distance_LEFT;
				old_distance_RIGHT = distance_RIGHT;
				
				turn_output = Limit_ul( turn_output , 100 , -100 );
				straight_output =  Limit_ul( straight_output , 100 , -100 );
				Move( straight_output + ( -1 ) * turn_output ,straight_output + turn_output ); //���[�^�֐�	

				//�u���[�L
/*				if( COOD_XY[task].X == BRAKE && fabs( difference_degree ) <= 2 &&  fabs( vertical_via( x_coordinates, y_coordinates, COOD_XY[task - 1].X, COOD_XY[task - 1].Y )) < 2){
					FRONT_LEFT = 1;
					BACK_LEFT = 1;
					FRONT_RIGHT = 1;
					BACK_RIGHT = 1;
					pwm( 99, 99 );
				}*/
			}
//		sprintf( string,"%d\n\r",count_while);
//		transmission( string );
//		count_while = 0;
	}
}
}