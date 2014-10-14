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
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

#include "typedefine.h"
#include "iodefine.h"
#include "machine.h"
#include "init.h"
#include "calculate.h"
#include "transmission.h"
#include "calculate_motor_output.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//�悭�ς���}�N����`
#define	PS3			//�R���g���[�����[�h PS3 or PS2
#define HIGH_SPEED_MODE		//�R�����g�A�E�g����ƑS���͂��o�Ȃ��Ȃ�
#define	MODE_SCIDATA_BOX	4	//�V���A���ʐM�Ō������ϐ��̐�
#define LIMIT_MOTOR_DUTY_TIRE	95	//���[�^�[�̍ō��o��
#define TURN_P_GAIN		1.8	//�p�x���b�NP�Q�C��
#define TURN_D_GAIN		10.0	//�p�x���b�ND�Q�C��
#define	INERTIA_PERCENT		0.7	//�|�W�V�������b�N�ɂ����銵���Ői�񂾋�������
#define OPERATE_DEGREE		90		//�ō��p���x�@��/s
#define	PWM_PER			95.0	//PWM�ω���
#define PWM_PERIOD_FUN			(48000000/ 64) / 60.5
//�E�O���[�^�[
#define	RIGHT_FLONT_CW		PORTB.DR.BIT.B3
#define	RIGHT_FLONT_CCW		PORTB.DR.BIT.B7
#define	RIGHT_FLONT_DUTY	MTU9.TGRD
#define	RIGHT_FLONT_MAX_DUTY	MTU9.TGRC
//�E�ヂ�[�^�[
#define	RIGHT_BACK_CW		PORTB.DR.BIT.B2
#define	RIGHT_BACK_CCW		PORTB.DR.BIT.B6
#define	RIGHT_BACK_DUTY		MTU9.TGRB
#define	RIGHT_BACK_MAX_DUTY	MTU9.TGRA
//���O���[�^�[
#define	LEFT_FLONT_CW		PORT7.DR.BIT.B7
#define	LEFT_FLONT_CCW		PORT7.DR.BIT.B5
#define	LEFT_FLONT_DUTY		MTU4.TGRD
#define	LEFT_FLONT_MAX_DUTY	MTU4.TGRC
//���ヂ�[�^�[
#define	LEFT_BACK_CW		PORT7.DR.BIT.B6
#define	LEFT_BACK_CCW		PORT7.DR.BIT.B4
#define	LEFT_BACK_DUTY		MTU4.TGRB
#define	LEFT_BACK_MAX_DUTY	MTU4.TGRA



//�t�@��
//#define	FUN_DUTY		MTU10.TGRB
#define	RAKET1_CW		PORT9.DR.BIT.B0
#define	RAKET1_CCW		PORT9.DR.BIT.B3
#define	RAKET1			MTU10.TGRB
#define	RAKET2_CW		PORT9.DR.BIT.B1
#define	RAKET2_CCW		PORT9.DR.BIT.B2
#define RAKET2			MTU10.TGRD

#define	ON			1
#define	OFF			0
#define	NO_DATA			0
#define INTERRUPT_START		CMT.CMSTR0.BIT.STR0 = 1;	//�J�E���g�J�n
#define	INTERRUPT_TIME 		5
#define	M_PI 			3.14159265
#define	BRAKE			1000
#define STICK_NO_MOVE_RANGE	0.2		//����肪�����Ȃ��X�e�B�b�N�̒l�͈̔�

#define	LED1				PORT8.DR.BIT.B0
#define	LED2				PORT8.DR.BIT.B1
#define	BUZZER				PORTD.DR.BIT.B7
#define	AIR				PORT6.DR.BIT.B0
#define	AIR2				PORT5.DR.BIT.B6
#define	AIR3				PORT3.DR.BIT.B2

#define	PWM_PERIOD		(48000000/1) / 100000	

#define	VERTICAL_ENCODER	MTU1.TCNT
#define	HORIZONTAL_ENCODER	MTU2.TCNT
#define	DIAMETER_VERTICAL_WHEEL		51.0	//�G���R�[�_�[�^�C���a
#define	DIAMETER_HORIZONTAL_WHEEL	51.0
#define PULSE_VERTICAL_ENCODER		500	//�G���R�[�_�[�p���X��
#define	PULSE_HORIZONTAL_ENCODER	500

#define END '#'			//�ʐM�f�[�^�̏I�[����
#define RECEIVE_STR_COLUMN 32	//1�f�[�^������̍ő啶����		��: a123# (6����)

#ifdef	PS3
	#define	LEFT_STICK_HIGH			g_AtoZ_value[0]
	#define	LEFT_STICK_WIDE			g_atoz_value[0]
	#define	RIGHT_STICK_WIDE		g_atoz_value[1]
	#define	KEY_UP				g_atoz_value[2]
	#define	KEY_RIGHT			g_atoz_value[3]
	#define	KEY_DOWN			g_atoz_value[4]
	#define	KEY_LEFT			g_atoz_value[5]
	#define	KEY_TRIANGLE			g_atoz_value[6]
	#define	KEY_CIRCLE			g_atoz_value[7]
	#define	KEY_CROSS			g_atoz_value[8]
	#define	KEY_SQUARE			g_atoz_value[9]
	#define	KEY_START			g_atoz_value[10]
	#define	ACCEL_X				g_atoz_value[12]
	#define	ACCEL_Y				g_atoz_value[13]
	#define	KEY_L1				g_atoz_value[14]
	#define	KEY_R1				g_atoz_value[15]
	#define	KEY_L2				g_atoz_value[16]
	#define	KEY_R2				g_atoz_value[17]
#endif

#ifdef	PS2
	#define	LEFT_STICK_HIGH			getdate3.byte.left_stick_high
	#define	LEFT_STICK_WIDE			getdate2.byte.left_stick_wide
	#define	RIGHT_STICK_WIDE		getdate2.byte.right_stick_wide
	#define	KEY_UP				getdate1.byte.up_sw
	#define	KEY_RIGHT			getdate1.byte.right_sw
	#define	KEY_DOWN			getdate1.byte.down_sw
	#define	KEY_LEFT			getdate1.byte.left_sw
	#define	KEY_CROSS			getdate2.byte.cross_sw
	#define	KEY_CIRCLE			getdate2.byte.circle_sw
	#define	KEY_L1				getdate2.byte.l1_sw
	#define	KEY_R1				getdate2.byte.r1_sw
	#define	KEY_L2				getdate2.byte.l2_sw
	#define	KEY_R2				getdate2.byte.r2_sw
	#define	CON_STATE			getdate1.byte.model_number
#endif

//�O���[�o���ϐ��Ɋi�[����ꍇ	���΂��ȗ�
float	g_atoz_value[26] = {127.00, 127.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
float	g_AtoZ_value[26] = {127.00, 127.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

float 	g_interrupt_timer_count	=	0.00,
	g_interrupt_timer_count2 = 	0.00;
int	g_over_vertical_count	=	0,
	g_under_vertical_count 	=	0,
	g_over_horizontal_count =	0, 	
	g_under_horizontal_count = 	0;
float 	g_Angle_f;

volatile unsigned long	g_controller_receive_1st	= 0;	//�R���g���[������A���Ă���f�[�^�̊i�[�t�H���_1st�͕s��ł悢�B
volatile unsigned long	g_controller_receive_2nd	= 0;
volatile unsigned long	g_controller_receive_3rd	= 0;
int 	getdate1 = 0, getdate2 = 0, getdate3 = 0;

unsigned char 	g_input_r1350n[15] = {0};

typedef	struct {
	float	velocity;
	float	x_c;
	float	y_c;
	float	degree;
}robodata_t;

typedef struct {
	float	X;
	float	Y;
	float 	TURN;
	float	rf;
	float	lf;
	float	lb;
	float	rb;
}motor_output_t;

typedef struct{
	float	sci_data1;
	float	sci_data2;
	float	sci_data3;
	float	sci_data4;
	float	sci_data5;
	float	sci_data6;
	float	sci_data7;
	float	sci_data8;
}sci_data_t;

union	psdate1{
	unsigned long	dword;
	struct{
		unsigned char	byte1;
		unsigned char	model_number;
		unsigned char	byte3;
		unsigned char	select_sw:1;
		unsigned char	l3_sw:1;
		unsigned char	r3_sw:1;
		unsigned char	start_sw:1;
		unsigned char	up_sw:1;
		unsigned char	right_sw:1;
		unsigned char	down_sw:1;
		unsigned char	left_sw:1;
	}byte;
};

union	psdate2{
	unsigned long	dword;
	struct{
		unsigned char	l2_sw:1;
		unsigned char	r2_sw:1;
		unsigned char	l1_sw:1;
		unsigned char	r1_sw:1;
		unsigned char	triangle_sw:1;
		unsigned char	circle_sw:1;
		unsigned char	cross_sw:1;
		unsigned char	square_sw:1;
		unsigned char 	right_stick_wide;
		unsigned char 	right_stick_high;
		unsigned char 	left_stick_wide;
	}byte;
};

union	 psdate3{
	unsigned long 	dword;
	struct{
		unsigned char 	left_stick_high;
	}byte;
};

typedef struct{
	float	rf;
	float	rb;
	float	lf;
	float	lb;
}motor_timer_count_t;

motor_timer_count_t	motor_timer_count;

void All_setup(void);
void over_flow_MTU1(void);
void under_flow_MTU1(void);
void over_flow_MTU2(void);
void under_flow_MTU2(void);
void wait_interrupt(void);
char Receive_uart_c_SCI0(void);
char Receive_uart_c_SCI2(void);
float	change_float(char *str);
void receive_order_depot(int target_box, char *storage_str, int minus_flag, int after_point_count, int large_size_flag);
void receive_order_c(char character);
void receive_att(void);
void Deadtime_right_flont_tire(void);
void Deadtime_left_flont_tire(void);
void Deadtime_left_back_tire(void);
void Deadtime_right_back_tire(void);
void input_R1350N(void);
float cal_straight_output_x(float l_stick_high);
float cal_straight_output_y(float l_stick_wide);
float	cal_add_turn(float r_stick_wide);
float Turn_PD(float target_degree , float now_degree);
void position_lock(float target_x , float target_y , float degree_now , int lock_count , float now_x , float now_y ,motor_output_t	*motor_output);
void calculate_coordnates(robodata_t	*robo);
void sci_transformer(sci_data_t	*string);
void Move(float	right_flont_duty, float	left_flont_duty,float	left_back_duty,	float	right_back_duty ,float limit_duty);
void Move_right_flont_tire(float right_flont_duty ,float limit_duty);
void Move_left_flont_tire(float left_flont_duty ,float limit_duty);
void Move_right_back_tire(float right_back_duty ,float limit_duty);
void Move_left_back_tire(float left_back_duty ,float limit_duty);
void Deadtime_right_flont_tire(void);
void Deadtime_left_flont_tire(void);
void Deadtime_left_back_tire(void);
void Deadtime_right_back_tire(void);
void air_mode(float air_key);
void controler_error();
void Rspi_recive_send_line_dualshock(void);

int main(void)
{
	int 	lock_count = 0,
		lock_count_old = 0,
		start_flug = 0;
	
	float	old_accel1 = 0.0,
		old_accel2 = 0.0,
		old_degree = 0.0,
		target_degree = 0.0,
		position_lock_x = 0.0,
		position_lock_y = 0.0,
		old_x_c = 0.0,
		old_y_c = 0.0,
		high_speed_output = 0.0,
		fun_power	= 0.0,
		limit_duty	= 50;
		
	int stop_flug_count = 0;
	
	sci_data_t	string = {0.0};
	robodata_t	robo;
	motor_output_t	motor_output;	
	
	union psdate1 getdate1;
	union psdate2 getdate2;
	union psdate3 getdate3;	
	
	All_setup();
	
//	FUN_DUTY = ((PWM_PERIOD * 0) / 100.0);
	while(1){
		if(start_flug == OFF && KEY_START >= 1){
			start_flug = ON;
			BUZZER = ON;
			g_over_vertical_count	=	0;
			g_under_vertical_count 	=	0;
			g_over_horizontal_count =	0; 	
			g_under_horizontal_count = 	0;
		}
		if(start_flug == ON){
			if(g_interrupt_timer_count >= INTERRUPT_TIME){	
				g_interrupt_timer_count = 0;
				
	#ifdef	PS2			
				//�f���A���V���b�N�̒l�����
				Rspi_recive_send_line_dualshock();
				getdate1.dword = g_controller_receive_1st;
				getdate2.dword = g_controller_receive_2nd;
				getdate3.dword = g_controller_receive_3rd;
	#endif

				if( fabs( -g_Angle_f - old_degree ) < 30 || fabs( -g_Angle_f - old_degree ) > 330){
					robo.degree = -g_Angle_f;
				}
				calculate_coordnates(&robo);

				motor_output.X = cal_straight_output_x((float) LEFT_STICK_HIGH);
				motor_output.Y = cal_straight_output_y((float) LEFT_STICK_WIDE);
				
				target_degree =	revision_degree(target_degree +	cal_add_turn((float)RIGHT_STICK_WIDE));

				motor_output.TURN =	Turn_PD( target_degree , robo.degree );
				
				motor_output.lf = get_motor_output_lf( motor_output.X, motor_output.Y, 0.0 );
				motor_output.rf = get_motor_output_rf( motor_output.X, motor_output.Y, 0.0 );
				motor_output.lb = get_motor_output_lb( motor_output.X, motor_output.Y, 0.0 );
				motor_output.rb = get_motor_output_rb( motor_output.X, motor_output.Y, 0.0 );
				
	#ifdef PS3	
				if(old_accel1 == ACCEL_X && old_accel2 == ACCEL_Y ){
					stop_flug_count ++;
				}else{
					stop_flug_count = 0;
				}
				old_accel1 = ACCEL_X;
				old_accel2 = ACCEL_Y;
	#endif

	#ifdef PS2	
				if(CON_STATE != 's'){
					stop_flug_count ++;
				}else{
					stop_flug_count = 0;
				}
	#endif
				
				if( stop_flug_count >= 120 ){
					controler_error();
					BUZZER = OFF;
				}
	#ifdef HIGH_SPEED_MODE	
				else if	(KEY_UP >= 1){
					lock_count_old = lock_count;
					BUZZER = OFF;
					
					high_speed_output += 0.5;
					motor_output.rf = high_speed_output;
					motor_output.lf = -1 * high_speed_output;
					motor_output.rb = -1 * high_speed_output;
					motor_output.lb = high_speed_output;
					
				}else	if( KEY_RIGHT >= 1){
					lock_count_old = lock_count;
					BUZZER = OFF;
					
					high_speed_output += 0.5;
					motor_output.rf = -1 * high_speed_output;
					motor_output.lf = -1 * high_speed_output;
					motor_output.rb = high_speed_output;
					motor_output.lb = high_speed_output;
				}
	#endif
				else if( KEY_CROSS >= 1 || ( motor_output.X == 0.00 && motor_output.Y == 0.00 && cal_add_turn((float)RIGHT_STICK_WIDE) == 0.00) ){
					BUZZER= OFF;
					if(lock_count == lock_count_old){
						lock_count ++;
					}
//					position_lock( position_lock_x , position_lock_y ,robo.degree, lock_count, robo.x_c, robo.y_c, &motor_output);
					controler_error();
					BUZZER = OFF;
				}else if( KEY_CROSS == 0 && stop_flug_count < 120 ){
					lock_count_old = lock_count;
					BUZZER = OFF;
				}
				
				if( KEY_R1 >= 1 ){
					fun_power += 1;
	//				FUN_DUTY = ((PWM_PERIOD * 50) / 100.0);
				}else if( KEY_L1 >= 1 ){
					fun_power -= 1;	
				}
	//			FUN_DUTY = ((PWM_PERIOD * Limit_ul(99,0,fun_power)) / 100.0);
	//			FUN_DUTY = ((PWM_PERIOD * 50) / 100.0);
				
				if( KEY_R2 >= 1 ){
	//				limit_duty += 1;
				}else if( KEY_L2 >= 1 ){
	//				limit_duty -= 1;	
				}
				limit_duty = Limit_ul(99,0,limit_duty);			
				
				if( stop_flug_count < 120 ){
					Move( motor_output.rf + motor_output.TURN , motor_output.lf + motor_output.TURN, motor_output.lb + motor_output.TURN, motor_output.rb + motor_output.TURN ,limit_duty);
				}else{
					controler_error();
				}
				
				if( stop_flug_count < 120 && KEY_CROSS == 0 && ( motor_output.X != 0.00 || motor_output.Y != 0.00 || cal_add_turn((float)RIGHT_STICK_WIDE) == 0.00)){
					
					if( robo.x_c - old_x_c > 0 ){
						position_lock_x = robo.x_c + INERTIA_PERCENT * 1000 * ( pow((fabs( robo.x_c - old_x_c ) /  ( (float)INTERRUPT_TIME / 1000 )) * 60 * 60 / 1000000,2) / ( 254 * 0.35 ));
					}else{
						position_lock_x = robo.x_c - INERTIA_PERCENT * 1000 * ( pow((fabs( robo.x_c - old_x_c ) /  ( (float)INTERRUPT_TIME / 1000 )) * 60 * 60 / 1000000,2) / ( 254 * 0.35 ));
					}
					
					if( robo.y_c - old_y_c > 0 ){
						position_lock_y = robo.y_c + INERTIA_PERCENT * 1000 * ( pow((fabs( robo.y_c - old_y_c ) /  ( (float)INTERRUPT_TIME / 1000 )) * 60 * 60 / 1000000,2) / ( 254 * 0.35 ));
					}else{
						position_lock_y = robo.y_c - INERTIA_PERCENT * 1000 * ( pow((fabs( robo.y_c - old_y_c ) /  ( (float)INTERRUPT_TIME / 1000 )) * 60 * 60 / 1000000,2) / ( 254 * 0.35 ));
					}
				}
				air_mode(KEY_TRIANGLE);
				if(KEY_CIRCLE >= 1){
					RAKET1_CW = 0;
					RAKET1_CCW = 1;
					RAKET2_CCW = 1;
					RAKET2_CW = 0;				
				}else if(KEY_SQUARE >= 1){
					RAKET1_CW = 1;
					RAKET1_CCW = 0;
					RAKET2_CCW = 0;
					RAKET2_CW = 1;
				}
				if(KEY_CIRCLE >= 1 || KEY_SQUARE >= 1){
					RAKET1 = (PWM_PERIOD * 50)/100;
					RAKET2 = (PWM_PERIOD * 50)/100;
				}else{
					RAKET1_CW = 0;
					RAKET1_CCW = 0;
					RAKET2_CCW = 0;
					RAKET2_CW = 0;
					RAKET1 = (PWM_PERIOD * 0)/100;
					RAKET2 = (PWM_PERIOD * 0)/100;
				}
				old_x_c = robo.x_c;
				old_y_c = robo.y_c;
				old_degree = robo.degree;
			}
		}
/*		if(g_interrupt_timer_count2 >= INTERRUPT_TIME * 10 ){
			FUN_DUTY = ((PWM_PERIOD_FUN * 80) / 100.0);
			g_interrupt_timer_count2 = 0;
			string.sci_data2 = robo.x_c;
			string.sci_data1 = robo.y_c;
			string.sci_data3 = VERTICAL_ENCODER;
			string.sci_data4 = KEY_L2;
			sci_transformer(&string);
		}*/
	}
}


void All_setup(void)
{
	init_clock();
	init_CMT0();
	init_pwm();
	init_all_encoder();
#ifdef	PS3
	init_Sci_0();
#endif
	init_Sci_1();
	init_Sci_2();
	init_dafalut();
	
	INTERRUPT_START//���荞�݃^�C�}�J�n
}

/******************************************************************************
*	�^�C�g�� �F flow_count
*	  �֐��� �F flow_count
*	  �߂�l �F void�^ �I�[�o�[�t���[�@�A���_�[�t���[�̃J�E���g
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/07/17
******************************************************************************/

void over_flow_MTU1(){
	g_over_vertical_count += 1;
}

void under_flow_MTU1(){
	g_under_vertical_count += 1;
}

void over_flow_MTU2(){
	g_over_horizontal_count += 1;
}

void under_flow_MTU2(){
	g_under_horizontal_count += 1;
}

void wait_interrupt(void)
{
	IR(CMT0,CMI0) = OFF;
	
	g_interrupt_timer_count ++;
	g_interrupt_timer_count2 ++;
}

/******************************************************************************
*	�^�C�g�� �F SPI�ʐM�Ŏ�M�������̂�Ԃ�
*	  �֐��� �F Rspi_send_1
*	  �߂�l �Funsigned lond�^
*	    ���� �F unsingned long moji
******************************************************************************/
unsigned long Rspi_send_1(unsigned long moji)
{
	RSPI1.SPDR.LONG = moji;
	while( RSPI1.SPSR.BIT.SPRF == 0 );	//��M�o�b�t�@�ɂȂɂ�����܂ő҂�
	return RSPI1.SPDR.LONG;
}
/******************************************************************************
*	�^�C�g�� �F SPI�ʐM�Ŏ�M�������̂�Ԃ�
*	  �֐��� �F Rspi_send_1
*	  �߂�l �Funsigned lond�^
*	    ���� �F unsingned long moji
******************************************************************************/
unsigned long Rspi_send_short_1(unsigned short int moji)
{
	RSPI1.SPDR.LONG = moji;
	while( RSPI1.SPSR.BIT.SPRF == 0 );	//��M�o�b�t�@�ɂȂɂ�����܂ő҂�
	return RSPI1.SPDR.LONG;
}
/******************************************************************************
*	�^�C�g�� �F �f���A���V���b�N����̑��M�f�[�^���i�[
*	  �֐��� �F Rspi_receive_send_line_dualshock
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
******************************************************************************/
void Rspi_recive_send_line_dualshock(void)	//DualShock�A�i���O�R���g���[��(�A�i���O���[�h��LED)�p���M�v���O����
{
	while( RSPI1.SPSR.BIT.SPRF == 1 ){				//��M�o�b�t�@���t���Ȃ烊�[�h���ăN���A����
		RSPI1.SPDR.LONG;
	}	
	g_controller_receive_1st = Rspi_send_1(0x00004201);
	g_controller_receive_2nd = Rspi_send_1(0x00000000);
	g_controller_receive_3rd = Rspi_send_short_1(0x00);
}

/******************************************************************************
*	�^�C�g�� �Fp12,p13�œ�����M�f�[�^���i�[���Ԃ�
*	  �֐��� �F Receive_uart_c
*	  �߂�l �F char�^
*	    �����F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2014/01/22
******************************************************************************/
char Receive_uart_c_SCI0(void)
{
	if (SCI0.SSR.BIT.RDRF == 0);		//RDRF = 0�FSCRDR �ɗL���Ȏ�M�f�[�^���i�[����Ă��Ȃ����Ƃ�\��
	SCI0.SSR.BIT.RDRF = 0;				//RDRF��ҋ@��ԂɕύX	
	return SCI0.RDR;
}

/******************************************************************************
*	�^�C�g�� �Fp12,p13�œ�����M�f�[�^���i�[���Ԃ�
*	  �֐��� �F Receive_uart_c
*	  �߂�l �F char�^
*	    �����F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2014/01/22
******************************************************************************/
char Receive_uart_c_SCI2(void)
{
	if (SCI2.SSR.BIT.RDRF == 0);		//RDRF = 0�FSCRDR �ɗL���Ȏ�M�f�[�^���i�[����Ă��Ȃ����Ƃ�\��
	SCI2.SSR.BIT.RDRF = 0;				//RDRF��ҋ@��ԂɕύX	
	return SCI2.RDR;
}

//�������float�^�ɕϊ�	���̊֐��̓��{�e�B�N�X�̃}�C�R���̎��ƂŔz��ꂽ��ł���
float change_float(char *str)
{
    float n = 0;
    int i = 0;
    while(str[i]!='\0')
    {
        if(str[i]<'0' || str[i]>'9') break;
        n=n*10+str[i]-'0';
        i++;
    }
    return(n);
}

//��͂������߂ɉ����Đ��l���O���[�o���ϐ��Ɋi�[����֐�	���̊֐��̑����݂�����
void receive_order_depot(int target_box, char *storage_str, int minus_flag, int after_point_count, int large_size_flag)
{    
    float value = 0.000;
    
    value = change_float(storage_str);

    if(minus_flag == 1){
        value *= ( -1.000 );
    }
    
    value = value * pow( 0.100, after_point_count );

    if( (target_box >= 0) && (target_box <= 25) ){
        if(large_size_flag == 0){
            g_atoz_value[target_box] = value;
        }else{
            g_AtoZ_value[target_box] = value;
        }
    }
}

//�ꕶ�����Ƃɉ�͂���֐�	���[�̂���̗͍�
void receive_order_c(char character)
{
    static int target_box = 255;						 //�i�[���߂̊J�n����(ASCII�R�[�h��a�`z,A�`Z)
    static char storage_str[RECEIVE_STR_COLUMN] = "";	 //�����̊i�[�p�̕�����
    static int storage_num = 0;							 //�����𕶎���̂ǂ��Ɋi�[���邩
    static int minus_flag = 0;							 //�}�C�i�X�l���ۂ�
    static int point_flag = 0;							 //�����_�ȉ����܂܂�Ă��邩�ۂ�
    static int after_point_count = 0;					 //�����_�ȉ��ɂǂꂾ���������邩
    static int large_size_flag = 0;						 //�啶���Ȃ̂��ۂ�
    int reset = 0;										 //�����̃��Z�b�g�����邩�ۂ�
	const char end = END;								 //�i�[���߂̏I������
    
    if(character == end){
            storage_str[storage_num] = '\0';
            receive_order_depot(target_box, storage_str, minus_flag, after_point_count, large_size_flag);
            reset = 1;
            target_box = 255;
            large_size_flag = 0;
    }else{
        if( (character >= '0') && (character <= '9') ){
            storage_str[storage_num] = character;
            storage_num++;  
            if( point_flag == 1 ){
                after_point_count++;
            }
        }else if( (character >= 'a') && (character <= 'z') ){
            reset = 1;
            target_box = (int)(character - 'a');
            large_size_flag = 0;
        }else if( (character >= 'A') && (character <= 'Z') ){
            reset = 1;
            target_box = (int)(character - 'A');
            large_size_flag = 1;
        }else if( character == '-' ){
            minus_flag = 1;
        }else if( character == '.' ){
            point_flag = 1;
        }
    }
    
    if( reset == 1 ){
       strcpy(storage_str,"");
       storage_num = 0;
       minus_flag = 0;
       point_flag = 0;
       after_point_count = 0;
    }    
}

//�ʐM���肩��̎�M���荞��	�����̓}�C�R���ɂ���ĈقȂ�
void receive_att(void)
{    
    char c;
    IR(SCI0,RXI0) = 0;
    c = Receive_uart_c_SCI0();//��M�f�[�^
    receive_order_c(c);
}

/******************************************************************************
*	�^�C�g�� �FR1350N�p��M�֐�
*	  �֐��� �F input_R1350N
*	  �߂�l �F void�^ 
*	   ����1 �F void
*	  �쐬�� �F �L�{��
*	  �쐬�� �F 2014/01/29
******************************************************************************/
void input_R1350N(void)
{
	static int i = 0;
	static int read_start = OFF;
	static float start_Angle = 0;
	unsigned int angle;
	static int start_flug	= 0;
	
	g_input_r1350n[i] = Receive_uart_c_SCI2();	
	
	//HEADER�l����
	if(g_input_r1350n[0] == 0xAA){
		read_start = ON;
	}
	
	if(read_start == ON){
		i++;
		//0�`14�܂ł�1�Z�b�g�̕�����
		if(i >= 15){
			i = 0;
			read_start = OFF;

			//�f�[�^��g�ݗ��Ă�
			angle = (g_input_r1350n[3] & 0xFF) | ((g_input_r1350n[4] << 8) & 0xFF00);
			
			if( start_flug == 0 ){
				start_flug = 1;
				start_Angle = angle / 100.0;
				if(start_Angle > 179){
					start_Angle = start_Angle - 655.0;
				}
				while(start_Angle > 179){
					start_Angle -= 360;
				}
				while(start_Angle < -179){
					start_Angle += 360;
				}
			}
			
			//�p�x�Ɗp���x�̒P�ʂ�ʏ�l�i���ɖ߂��f�[�^���L������
			g_Angle_f = angle / 100.0;
			
			g_Angle_f -= start_Angle;
			
			if(g_Angle_f > 179){
				g_Angle_f = g_Angle_f - 655.0;
			}
			
			g_Angle_f = revision_degree(g_Angle_f);
		}
	}
}

float cal_straight_output_x(float l_stick_high)
{	
	float straight_cal_x = 0.00;
	
	straight_cal_x = ( 255.0 - l_stick_high) / 255.0;
	straight_cal_x = ( straight_cal_x - 0.5 ) * 2;
	if( fabs( straight_cal_x ) <= STICK_NO_MOVE_RANGE ){
		straight_cal_x = 0;
	}
	straight_cal_x *= PWM_PER;
	
	return(straight_cal_x);
}

float cal_straight_output_y( float l_stick_wide ){
	
	float straight_cal_y = 0.00;
	
	straight_cal_y = ( 255.0 - l_stick_wide ) / 255.0;
	straight_cal_y = ( straight_cal_y - 0.5 ) * 2;
	if( fabs( straight_cal_y ) <= STICK_NO_MOVE_RANGE ){
		straight_cal_y = 0;
	}
	straight_cal_y *= PWM_PER;
	
	return(straight_cal_y);
}

float	cal_add_turn( float r_stick_wide )
{	
	float turn_cal = 0.00;
	
	turn_cal = ( 255.0 - r_stick_wide ) / 255.0;
	turn_cal = ( turn_cal - 0.5 ) * 2;
	if( fabs( turn_cal ) <= STICK_NO_MOVE_RANGE ){
		turn_cal = 0;
	}
	turn_cal *= ( OPERATE_DEGREE / ( 1.00 / ( INTERRUPT_TIME / 1000.0 ) ));
	
	return(turn_cal);
}

//PD��]����
float Turn_PD(float target_degree , float now_degree)
{	
	float output = 0.00;
	float difference_degree = 0.00;
	static float old_difference_degree = 0.00;
	
	difference_degree = target_degree - now_degree;

	if ( difference_degree > 180 ){
		difference_degree = -360 + difference_degree;
	}else if ( difference_degree < -180 ){
		difference_degree = difference_degree + 360;
	}
	output = ( TURN_P_GAIN * difference_degree ) + ( TURN_D_GAIN * ( difference_degree - old_difference_degree ));
	
	output = Limit_ul( 50, -50 , output);
	
	old_difference_degree = difference_degree;
	
	return output;
}	

void position_lock(float target_x , float target_y , float degree_now , int lock_count , float now_x , float now_y ,motor_output_t	*motor_output)
{
	float  motor_output_x = 0.0,	motor_output_y = 0.0;
	float p_gain = 0.02,d_gain = 0.2;
	float gap_x = 0.0, gap_y = 0.0;
	static float gap_old_x = 0.0 , gap_old_y = 0.0;
	static int lock_count_old = 0;
//	char string[60] = {0};
	
	if(lock_count != lock_count_old){
//		gap_old = gap_now;
		gap_old_x = gap_x;
		gap_old_y = gap_y;
	}

//	gap_now = get_distance(target_x,target_y,robo.x_c,robo.y_c);
	gap_x = fabs(target_x - now_x);
	gap_y = fabs(target_y - now_y);

//	motor_output = (p_gain * gap_now) + (d_gain * (gap_now - gap_old));
//	gap_old = gap_now;

//	motor_output_x = motor_output * cos(convert_radian(target_degree));
//	motor_output_y = motor_output * sin(convert_radian(target_degree));
	if(target_x - now_x > 0){
		motor_output_x = (p_gain * gap_x) + (d_gain * (gap_x - gap_old_x));
	}else{
		motor_output_x = -1 * ((p_gain * gap_x) + (d_gain * (gap_x - gap_old_x)));
	}
	if(target_y - now_y > 0){
		motor_output_y = (p_gain * gap_y) + (d_gain * (gap_y - gap_old_y));
	}else{
		motor_output_y = -1 * ((p_gain * gap_y) + (d_gain * (gap_y - gap_old_y)));
	}
	motor_output->lf = get_motor_output_lf(motor_output_x,motor_output_y,degree_now);
	motor_output->rf = get_motor_output_rf(motor_output_x,motor_output_y,degree_now);
	motor_output->lb = get_motor_output_lb(motor_output_x,motor_output_y,degree_now);
	motor_output->rb = get_motor_output_rb(motor_output_x,motor_output_y,degree_now);
	
	lock_count_old = lock_count;
	gap_old_x = gap_x;
	gap_old_y = gap_y;	
//	sprintf(string,"%f,%f,%f,%f,%f\n\r",target_degree,target_x,target_y,robo.x_c,robo.y_c);
//	transmission_string(string);
}

void calculate_coordnates(robodata_t	*robo)
{
	int	vertical_enc_count = 0,
		horizontal_enc_count = 0;
	
	static int	old_vertical_enc_count = 0,
			old_horizontal_enc_count = 0;
			
	float	add_distance_vertical = 0.0,
		add_distance_horizontal = 0.0,
		add_distance = 0.0,
		add_distance_degree = 0.0,
		x,
		y;
		
	sci_data_t	string = {0.0};	
			
	vertical_enc_count = VERTICAL_ENCODER  + ( 65536 * g_over_vertical_count)  + ( ( -65536 ) * g_under_vertical_count ); //�����G���R�[�_�[�̒l
	horizontal_enc_count = HORIZONTAL_ENCODER  + ( 65536 * g_over_horizontal_count)  + ( ( -65536 ) * g_under_horizontal_count ); //�����G���R�[�_�[�̒l				
	
	add_distance_vertical = ( ( vertical_enc_count - old_vertical_enc_count ) * M_PI * DIAMETER_VERTICAL_WHEEL ) / ( PULSE_VERTICAL_ENCODER * 4 );
	add_distance_horizontal = ( ( horizontal_enc_count - old_horizontal_enc_count ) * M_PI * DIAMETER_HORIZONTAL_WHEEL ) / ( PULSE_HORIZONTAL_ENCODER * 4 );
	add_distance = pow(add_distance_vertical * add_distance_vertical + add_distance_horizontal * add_distance_horizontal,0.5);
	
	if(add_distance_horizontal != 0 || add_distance_vertical != 0 ){
		add_distance_degree = atan2( add_distance_horizontal , add_distance_vertical ) * 180 / M_PI;
		robo->x_c = robo->x_c + add_distance * cos( convert_radian( add_distance_degree + robo->degree  ));
		x = robo->x_c;
		robo->y_c = robo->y_c + add_distance * sin( convert_radian( add_distance_degree + robo->degree  ));
		y = robo->y_c;
	}
	robo->velocity = add_distance / ( (float)INTERRUPT_TIME / 1000 );
	
	old_vertical_enc_count = vertical_enc_count;
	old_horizontal_enc_count = horizontal_enc_count;
	
	string.sci_data2 = y;
	string.sci_data1 = x;
	string.sci_data3 = 1;
	string.sci_data4 = 2;
//	sci_transformer(&string);
	
}

/******************************************************************************
*	�^�C�g�� �F Excel�����̂��߂̃f�[�^���V���A�����M
*	  �֐��� �F sci_transformer
*	  �߂�l �F void�^ 
*	    ���� �F �Ȃ�
*	  �쐬�� �F �����N�G
*	  �쐬�� �F 2013/02/25
******************************************************************************/
void sci_transformer(sci_data_t	*string)
{	
	#if MODE_SCIDATA_BOX != OFF
		char 	sc1[50],sc2[50],sc3[50],sc4[50],sc5[50],sc6[50],sc7[50],sc8[50];
	#endif
	
	#if MODE_SCIDATA_BOX >= 1
		sprintf(sc1,"%f",(float)string->sci_data1);
		transmission_string(",");
		transmission_string(sc1);
	#endif
	#if MODE_SCIDATA_BOX >= 2
		sprintf(sc2,"%f",(float)string->sci_data2);
		transmission_string(",");
		transmission_string(sc2);
	#endif
	#if MODE_SCIDATA_BOX >= 3
		sprintf(sc3,"%f",(float)string->sci_data3);
		transmission_string(",");
		transmission_string(sc3);
	#endif
	#if MODE_SCIDATA_BOX >= 4
		sprintf(sc4,"%f",(float)string->sci_data4);
		transmission_string(",");
		transmission_string(sc4);
	#endif
	#if MODE_SCIDATA_BOX >= 5
		sprintf(sc5,"%f",(float)string->sci_data5);
		transmission_string(",");
		transmission_string(sc5);
	#endif
	#if MODE_SCIDATA_BOX >= 6
		sprintf(sc6,"%5d",(long)string->sci_data6);
		transmission_string(",");
		transmission_string(sc6);		
	#endif
	#if MODE_SCIDATA_BOX >= 7
		sprintf(sc7,"%5d",(long)string->sci_data7);
		transmission_stringg(",");
		transmission_string(sc7);
	#endif
	#if MODE_SCIDATA_BOX >= 8
		sprintf(sc8,"%5d",(long)string->sci_data8);
		transmission_string(",");
		transmission_string(sc8);
	#endif
	
	transmission_string("\n\r");
}

void wait_timer_count(void)
{
	IR(CMT0,CMI0) = OFF;	
	motor_timer_count.rf ++;
	motor_timer_count.rb ++;
	motor_timer_count.lf ++;
	motor_timer_count.lb ++;
}

/******************************************************************************
*	�^�C�g�� �F �E�O�^�C���̏o�̓��Z�b�g
*	  �֐��� �F Deadtime_right_flont_tire
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Deadtime_right_flont_tire(void)
{
	RIGHT_FLONT_CW = OFF;
	RIGHT_FLONT_CCW = OFF;
	RIGHT_FLONT_DUTY = OFF;
}

/******************************************************************************
*	�^�C�g�� �F �E�O�^�C���̓���
*	  �֐��� �F Move_right_flont_tire
*	  �߂�l �F void�^
*	    ����1�F float�^ right_duty 
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Move_right_flont_tire(float right_flont_duty ,float limit_duty)
{
	static int i = 0;
	static float right_flont_duty_old = 0.0;

	if(right_flont_duty == BRAKE){
		right_flont_duty = 0.0;
	}
	
	if(((right_flont_duty > 0.0) && (right_flont_duty_old > 0.0)) || ((right_flont_duty < 0.0) && (right_flont_duty_old < 0.0))){
		if(fabs(right_flont_duty - right_flont_duty_old) >= 100.0){
			right_flont_duty = (right_flont_duty + right_flont_duty_old) / 2.000;
		}else{
			right_flont_duty = right_flont_duty;
		}
	}
	
	if(right_flont_duty < 0){
		if(i != 0){
			Deadtime_right_flont_tire();
			if(motor_timer_count.rf >= 10){
				i = 0;
			}
		}else{
			motor_timer_count.rf = 0;
			right_flont_duty *= (-1);
			RIGHT_FLONT_CW = OFF;
			RIGHT_FLONT_CCW = ON;
			i = 0;
		}
	}else if(right_flont_duty == BRAKE){
		if(i != 1){
			Deadtime_right_flont_tire();
			if(motor_timer_count.rf >= 10){
				i = 1;
			}
		}else{
			motor_timer_count.rf = 0;
			RIGHT_FLONT_CW = ON;
			RIGHT_FLONT_CCW = ON;
			right_flont_duty = 100.0;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_right_flont_tire();
			if(motor_timer_count.rf >= 10){
				i = 2;
			}
		}else{
			motor_timer_count.rf = 0;
			RIGHT_FLONT_CW = ON;
			RIGHT_FLONT_CCW = OFF;
			i = 2;
		}
	}
	
	if(right_flont_duty > 15){
		right_flont_duty =	Limit_ul(limit_duty , 0, right_flont_duty);
		RIGHT_FLONT_DUTY = ((PWM_PERIOD * right_flont_duty) / 100.0);
		}
	else{
		Deadtime_right_flont_tire();
		}
	
	right_flont_duty_old = right_flont_duty;
}

/******************************************************************************
*	�^�C�g�� �F ���O�^�C���̏o�̓��Z�b�g
*	  �֐��� �F Deadtime_left_tire
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Deadtime_left_flont_tire(void)
{
	LEFT_FLONT_CW = OFF;
	LEFT_FLONT_CCW = OFF;
	LEFT_FLONT_DUTY = OFF;
}

/******************************************************************************
*	�^�C�g�� �F ���O�^�C������
*	  �֐��� �F Move_left_tire
*	  �߂�l �F void�^
*	    ����1�F float�^ left_duty 
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Move_left_flont_tire(float left_flont_duty ,float limit_duty)
{
	static int i = 0;
	static float left_flont_duty_old = 0.0;
	
	if(left_flont_duty == BRAKE){
		left_flont_duty = 0.0;
	}
	
	if(((left_flont_duty > 0.0) && (left_flont_duty_old > 0.0)) || ((left_flont_duty < 0.0) && (left_flont_duty_old < 0.0))){
		if(fabs(left_flont_duty - left_flont_duty_old) >= 100.0){
			left_flont_duty = (left_flont_duty + left_flont_duty_old) / 2.000;
		}else{
			left_flont_duty = left_flont_duty;
		}
	}
	
	if(left_flont_duty < 0){
		if(i != 0){
			Deadtime_left_flont_tire();
			if(motor_timer_count.lf >= 10){
				i = 0;
			}
		}else{
			motor_timer_count.lf = 0;
			left_flont_duty *= (-1);
			LEFT_FLONT_CW = OFF;
			LEFT_FLONT_CCW = ON;
			i = 0;
		}
	}else if(left_flont_duty == BRAKE){
		if(i != 1){
			Deadtime_left_flont_tire();
			if(motor_timer_count.lf >= 10){
				i = 1;
			}
		}else{
			motor_timer_count.lf = 0;
			LEFT_FLONT_CW = ON;
			LEFT_FLONT_CCW = ON;
			left_flont_duty = 100;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_left_flont_tire();
			if(motor_timer_count.lf >= 10){
				i = 2;
			}	
		}else{
			motor_timer_count.lf = 0;
			LEFT_FLONT_CW = ON;
			LEFT_FLONT_CCW = OFF;
			i = 2;
		}
	}
	
	if(left_flont_duty > 15){
		left_flont_duty = Limit_ul(limit_duty,0.0,left_flont_duty);
		LEFT_FLONT_DUTY = (PWM_PERIOD * left_flont_duty) / 100;
	}else{
		Deadtime_left_flont_tire();
	}
	
	left_flont_duty_old = left_flont_duty;
}

/******************************************************************************
*	�^�C�g�� �F ����^�C���̏o�̓��Z�b�g
*	  �֐��� �F Deadtime_left_tire
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Deadtime_left_back_tire(void)
{
	LEFT_BACK_CW = OFF;
	LEFT_BACK_CCW = OFF;
	LEFT_BACK_DUTY = OFF;
}

/******************************************************************************
*	�^�C�g�� �F ����^�C������
*	  �֐��� �F Move_left_back_tire
*	  �߂�l �F void�^
*	    ����1�F float�^ left_back_duty 
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Move_left_back_tire(float left_back_duty ,float limit_duty)
{
	static int i = 0;
	static float left_back_duty_old = 0.0;
	
	if(left_back_duty == BRAKE){
		left_back_duty = 0.0;
	}
	
	if(((left_back_duty > 0.0) && (left_back_duty_old > 0.0)) || ((left_back_duty < 0.0) && (left_back_duty_old < 0.0))){
		if(fabs(left_back_duty - left_back_duty_old) >= 100.0){
			left_back_duty = (left_back_duty + left_back_duty_old) / 2.000;
		}else{
			left_back_duty = left_back_duty;
		}
	}
	
	if(left_back_duty < 0){
		if(i != 0){
			Deadtime_left_back_tire();
			if(motor_timer_count.lb >= 10){
				i = 0;
			}
		}else{
			motor_timer_count.lb = 0;
			left_back_duty *= (-1);
			LEFT_BACK_CW = OFF;
			LEFT_BACK_CCW = ON;
			i = 0;
		}
	}else if(left_back_duty == BRAKE){
		if(i != 1){
			Deadtime_left_back_tire();
			if(motor_timer_count.lb >= 10){
				i = 1;
			}
		}else{
			motor_timer_count.lb = 0;
			LEFT_BACK_CW = ON;
			LEFT_BACK_CCW = ON;
			left_back_duty = 100;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_left_back_tire();
			if(motor_timer_count.lb >= 10){
				i = 2;
			}	
		}else{
			motor_timer_count.lb = 0;
			LEFT_BACK_CW = ON;
			LEFT_BACK_CCW = OFF;
			i = 2;
		}
	}
	
	if(left_back_duty > 15){
		left_back_duty = Limit_ul(limit_duty,0.0,left_back_duty);
		LEFT_BACK_DUTY = ((PWM_PERIOD * left_back_duty) / 100);
	}else{
		Deadtime_left_back_tire();
	}
	
	left_back_duty_old = left_back_duty;
}

/******************************************************************************
*	�^�C�g�� �F �E��^�C���̏o�̓��Z�b�g
*	  �֐��� �F Deadtime_right_back_tire
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Deadtime_right_back_tire(void)
{
	RIGHT_BACK_CW = OFF;
	RIGHT_BACK_CCW = OFF;
	RIGHT_BACK_DUTY = OFF;
}

/******************************************************************************
*	�^�C�g�� �F �E��^�C���̓���
*	  �֐��� �F Move_right_back_tire
*	  �߂�l �F void�^
*	    ����1�F float�^ right_back_duty 
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Move_right_back_tire(float right_back_duty ,float limit_duty)
{
	static int i = 0;
	static float right_back_duty_old = 0.0;
	
	if(right_back_duty == BRAKE){
		right_back_duty = 0.0;
	}
	
	if(((right_back_duty > 0.0) && (right_back_duty_old > 0.0)) || ((right_back_duty < 0.0) && (right_back_duty_old < 0.0))){
		if(fabs(right_back_duty - right_back_duty_old) >= 100.0){
			right_back_duty = (right_back_duty + right_back_duty_old) / 2.000;
		}else{
			right_back_duty = right_back_duty;
		}
	}
	
	if(right_back_duty < 0){
		if(i != 0){
			Deadtime_right_back_tire();
			if(motor_timer_count.rb >= 10){
				i = 0;
			}
		}else{
			motor_timer_count.rb = 0;
			right_back_duty *= (-1);
			RIGHT_BACK_CW = OFF;
			RIGHT_BACK_CCW = ON;
			i = 0;
		}
	}else if(right_back_duty == BRAKE){
		if(i != 1){
			Deadtime_right_back_tire();
			if(motor_timer_count.rb >= 10){
				i = 1;
			}
		}else{
			motor_timer_count.rb = 0;
			RIGHT_BACK_CW = ON;
			RIGHT_BACK_CCW = ON;
			right_back_duty = 100.0;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_right_back_tire();
			if(motor_timer_count.rb >= 10){
				i = 2;
			}
		}else{
			motor_timer_count.rb = 0;
			RIGHT_BACK_CW = ON;
			RIGHT_BACK_CCW = OFF;
			i = 2;
		}
	}
	
	if(right_back_duty > 15){
		right_back_duty =	Limit_ul(limit_duty , 0	, right_back_duty);
		RIGHT_BACK_DUTY = ((PWM_PERIOD * right_back_duty) / 100.0);
		}
	else{
		Deadtime_right_back_tire();
		}
	
	right_back_duty_old = right_back_duty;
}

/******************************************************************************
*	�^�C�g�� �F ���[�^�֐�
*	  �֐��� �F Move
*	  �߂�l �F void�^
*	    ����1�F float�^ left_duty
*	    ����2�F float�^ right_duty
*	    ����3�F float�^ back_duty
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
void Move(float	right_flont_duty, float	left_flont_duty,float	left_back_duty,	float	right_back_duty ,float limit_duty)
{
	Move_left_flont_tire(left_flont_duty,limit_duty);
	Move_right_flont_tire(right_flont_duty,limit_duty);
	Move_left_back_tire(left_back_duty,limit_duty);
	Move_right_back_tire(right_back_duty,limit_duty);
}

void air_mode(float air_key)
{
	static int	change_flug = 0;
	
	if( air_key >= 1 && change_flug == 1){
		AIR = 1 - AIR;
		change_flug = 0;
	}else	if( air_key == 0 ){
		change_flug = 1;
	}
}

void	controler_error(void)
{
	BUZZER = ON;
	
	LEFT_FLONT_CW = OFF;
	LEFT_FLONT_CCW = OFF;
	LEFT_BACK_CW = OFF;
	LEFT_BACK_CCW = OFF;
	RIGHT_FLONT_CW = OFF;
	RIGHT_FLONT_CCW = OFF;
	RIGHT_BACK_CW = OFF;
	RIGHT_BACK_CCW = OFF;
}
