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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define ON					1
#define OFF					0
#define NO_DATA				0
#define INTERRUPT_START		CMT.CMSTR0.BIT.STR0 = 1;//�J�E���g�J�n
#define INTERRUPT_TIME 			5
#define STICK_NO_MOVE_RANGE	0.2		//����肪�����Ȃ��X�e�B�b�N�̒l�͈̔�
#define OPERATE_DEGREE			90
#define RIGHT_FLONT_CW			PORT7.DR.BIT.B0
#define RIGHT_FLONT_CCW		PORT7.DR.BIT.B2
#define LEFT_BACK_CW			PORT7.DR.BIT.B7
#define LEFT_BACK_CCW			PORT7.DR.BIT.B5
#define LEFT_FLONT_CW			PORT7.DR.BIT.B6
#define LEFT_FLONT_CCW			PORT7.DR.BIT.B4
#define RIGHT_BACK_CW			PORT7.DR.BIT.B1
#define RIGHT_BACK_CCW			PORT7.DR.BIT.B3
#define RIGHT_FLONT_DUTY		MTU6.TGRB
#define LEFT_FLONT_DUTY		MTU4.TGRB
#define LEFT_BACK_DUTY			MTU4.TGRD
#define RIGHT_BACK_DUTY		MTU6.TGRD
#define RIGHT_FLONT_MAX_DUTY	MTU4.TGRA
#define LEFT_FLONT_MAX_DUTY	MTU6.TGRA
#define LEFT_BACK_MAX_DUTY		MTU6.TGRC
#define RIGHT_BACK_MAX_DUTY	MTU4.TGRC
#define M_PI 					3.14159265
#define LIMIT_MOTOR_DUTY_TIRE	95
#define BRAKE					1000
#define END '#'						//�ʐM�f�[�^�̏I�[����
#define RECEIVE_STR_COLUMN 32		//1�f�[�^������̍ő啶����		��: a123# (6����)
#define VERTICAL_ENCODER			MTU1.TCNT
#define HORIZONTAL_ENCODER			MTU2.TCNT
#define DIAMETER_VERTICAL_WHEEL		51
#define DIAMETER_HORIZONTAL_WHEEL	51
#define PULSE_VERTICAL_ENCODER		500
#define PULSE_HORIZONTAL_ENCODER	500
#define START_X_COORDNATES			0.00
#define START_Y_COORDNATES			0.00
#define PI							3.14159265358979
#define TURN_P_GAIN				1.5
#define TURN_D_GAIN				10
#define LEFT_STICK_HIGH				g_AtoZ_value[0]	
#define LEFT_STICK_WIDE				g_atoz_value[0]
#define RIGHT_STICK_WIDE			g_atoz_value[1]
#define CROSS						g_atoz_value[8]
#define PWM_PER					95

//�\���̐錾	
struct coordinates
{
	float	X;
	float	Y;
	float degree;
};

struct calc_velocity
{
	float lf;
	float rf;
	float lb;
	float rb;
}motor_output_velocity;

//�O���[�o���ϐ��Ɋi�[����ꍇ	���΂��ȗ�
float g_atoz_value[26] = {127.00, 127.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
float g_AtoZ_value[26] = {127.00, 127.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 
                                0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

float g_interrupt_timer_count = 0.00,
	g_interrupt_timer_count2 = 0.00 ,
	g_interrupt_timer_count3 = 0.00 ,
	g_right_flont_motor_timer_count = 0.00,
	g_right_back_motor_timer_count = 0.00,
	g_left_flont_motor_timer_count = 0.00,
	g_left_back_motor_timer_count = 0.00;
int	g_over_vertical_count	 = 0, g_under_vertical_count = 0, g_over_horizontal_count = 0, g_under_horizontal_count = 0;
float	g_x_coordnates = START_X_COORDNATES, g_y_coordnates = START_Y_COORDNATES;
float g_Angle;
float g_Rate;
float g_Angle_f;
float g_Rate_f;
float g_X_acc;
float g_Y_acc;
float g_Z_acc;
float g_velocity;
float g_x_velocity;
float g_y_velocity;
unsigned char  g_input_r1350n[15] = {0};

void All_setup(void);
void over_flow_MTU1(void);
void under_flow_MTU1(void);
void over_flow_MTU2(void);
void under_flow_MTU2(void);
void wait_interrupt(void);
float convert_radian(float degree);
float Limit_ul(float upper,float lower,float figure);
char Receive_uart_c(void);
char Receive_uart_c_SCI2(void);
float change_float(char *str);
void receive_order_depot(int target_box, char *storage_str, int minus_flag, int after_point_count, int large_size_flag);
void receive_order_c(char character);
void receive_att(void);
void transmission_string(char *s);
float get_motor_output_lf(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_rf(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_lb(float motor_output_x,float motor_output_y,float degree_now);
float get_motor_output_rb(float motor_output_x,float motor_output_y,float degree_now);
void Deadtime_right_flont_tire(void);
void Deadtime_left_flont_tire(void);
void Deadtime_left_back_tire(void);
void Deadtime_right_back_tire(void);
void Move_right_flont_tire(float right_flont_duty);
void Move_left_flont_tire(float left_flont_duty);
void Move_left_back_tire(float left_back_duty);
void Move_right_back_tire(float right_back_duty);
void Move(float right_flont_duty,float left_flont_duty,float left_back_duty,float right_back_duty);
void input_R1350N(void);
float straight_output_x(void);
float straight_output_y(void);
float turn_output(void);
float Turn_PD(float target_degree , float now_degree);
float get_target_degree(float target_x,float target_y,float x_now,float y_now);
float get_distance(float target_x,float target_y,float x_now,float y_now);
float revision_degree(float degree);
void get_motor_output(float x_start,  float y_start,  float x_now,  float y_now,  float target_x, float target_y, float degree_now, float a_up,  float a_down,  float max_velocity,  int task);
int main(void);

void All_setup(void)
{
	init_clock();
	init_CMT0();
	init_pwm();
	init_all_encoder();
	init_Sci_0();
	init_Sci_1();
	init_Sci_2();
	
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
void over_flow_MTU1()
{
	g_over_vertical_count += 1;
}

void under_flow_MTU1()
{
	g_under_vertical_count += 1;
}

void over_flow_MTU2()
{
	g_over_horizontal_count += 1;
}

void under_flow_MTU2()
{
	g_under_horizontal_count += 1;
}

void wait_interrupt(void)
{
	IR(CMT0,CMI0) = OFF;
//	PORT8.DR.BIT.B1 = 1;
	g_interrupt_timer_count ++;
	g_interrupt_timer_count2 ++;
	g_interrupt_timer_count3 ++;	
	g_right_flont_motor_timer_count ++;
	g_right_back_motor_timer_count ++;
	g_left_flont_motor_timer_count ++;
	g_left_back_motor_timer_count ++;
}

/******************************************************************************
*	�^�C�g�� �F �x���̃��W�A���ϊ�
*	  �֐��� �F convert_radian
*	  �߂�l �F float�^
*	    ����1�F float�^ degree 
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
float convert_radian(float degree)
{
	float radian = 0.0;
	while(degree > 180){
		degree -= 360;
	}
	while(degree < -180){
		degree += 360;
	}
	radian = degree * (PI / 180);
	return (radian);
}

/******************************************************************************
*	�^�C�g�� �F �ݒ肵���͈͓��̒l��Ԃ�
*	  �֐��� �F Limit_ul
*	  �߂�l �F float�^ �o�͒l
*	   ����1 �F float�^ upper  ����̐��l
*	   ����2 �F float�^ lower  �����̐��l
*	   ����3 �F float�^ figure  ��r���鐔�l
*	  �쐬�� �F �s�� �q��
*	  �쐬�� �F 2011/08/31
******************************************************************************/
float Limit_ul(float upper,float lower,float figure)
{
	if(upper < figure){
		return(upper);
	}else if(figure < lower){
		return(lower);
	}else{
		return(figure);
	}
}

/******************************************************************************
*	�^�C�g�� �Fp12,p13�œ�����M�f�[�^���i�[���Ԃ�
*	  �֐��� �F Receive_uart_c
*	  �߂�l �F char�^
*	    �����F �Ȃ�
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2014/01/22
******************************************************************************/
char Receive_uart_c(void)
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
    PORT8.DR.BIT.B1 = 1;
	c = Receive_uart_c();//��M�f�[�^
    receive_order_c(c);
}

/******************************************************************************
*	�^�C�g�� �F ��������V���A�����M
*	  �֐��� �F transmission_string
*	  �߂�l �F void�^ 
*	   ����1 �F char�^ *s  
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/09/25
******************************************************************************/
void transmission_string(char *s)
{
	int i = 0;
	while(s[i] != '\0'){	
			
		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			SCI1.TDR = s[i];
			i++;
		}
	}
}

/******************************************************************************
*	�^�C�g�� �F ���O�I���j�^�C���̏o�͌���
*	  �֐��� �F get_motor_output_lf
*	  �߂�l �F float�^
*	    ����1 �Ffloat�^ motor_output_x
*	    ����2 �Ffloat�^ motor_output_y
*	    ����3 �Ffloat�^ degree_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/11/21
******************************************************************************/
float get_motor_output_lf(float motor_output_x,float motor_output_y,float degree_now)
{
	float 	motor_output_lf = 0.0,
		degree_reverse_x = 0.0,
		degree_reverse_y = 0.0;
	
	if(motor_output_x < 0.0){
		degree_reverse_x = 180.0;
	}else{
		degree_reverse_x = 0.0;
		}
	if(motor_output_y < 0.0){
		degree_reverse_y = 180.0;
	}else{
		degree_reverse_y = 0.0;
	}
	
	motor_output_lf = fabs(motor_output_x) * sin(convert_radian(degree_now + (120.0 + degree_reverse_x) + 90)) - fabs(motor_output_y) * cos(convert_radian(degree_now + (120.0 + degree_reverse_y) + 90));
	return(motor_output_lf);
}

/******************************************************************************
*	�^�C�g�� �F �E�O�I���j�^�C���̏o�͌���
*	  �֐��� �F get_motor_output_rf
*	  �߂�l �F float�^
*	    ����1 �Ffloat�^ motor_output_x
*	    ����2 �Ffloat�^ motor_output_y
*	    ����3 �Ffloat�^ degree_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/11/21
******************************************************************************/
float get_motor_output_rf(float motor_output_x,float motor_output_y,float degree_now)
{
	float 	motor_output_rf = 0.0,
		degree_reverse_x = 0.0,
		degree_reverse_y = 0.0;
	
	if(motor_output_x < 0.0){
		degree_reverse_x = 180.0;
	}else{
		degree_reverse_x = 0.0;
		}
	if(motor_output_y < 0.0){
		degree_reverse_y = 180.0;
	}else{
		degree_reverse_y = 0.0;
	}
	
	motor_output_rf = fabs(motor_output_x) * sin(convert_radian(degree_now +( 60.0 + degree_reverse_x) + 90)) - fabs(motor_output_y) * cos(convert_radian(degree_now + (60.0 + degree_reverse_y) + 90));
	return(motor_output_rf);
}

/******************************************************************************
*	�^�C�g�� �F ����I���j�^�C���̏o�͌���
*	  �֐��� �F get_motor_output_lb
*	  �߂�l �F float�^
*	    ����1 �Ffloat�^ motor_output_x
*	    ����2 �Ffloat�^ motor_output_y
*	    ����3 �Ffloat�^ degree_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/11/21
******************************************************************************/
float get_motor_output_lb(float motor_output_x,float motor_output_y,float degree_now)
{
	float 	motor_output_lb = 0.0,
		degree_reverse_x = 0.0,
		degree_reverse_y = 0.0;
	
	if(motor_output_x < 0.0){
		degree_reverse_x = 180.0;
	}else{
		degree_reverse_x = 0.0;
		}
	if(motor_output_y < 0.0){
		degree_reverse_y = 180.0;
	}else{
		degree_reverse_y = 0.0;
	}
	
	motor_output_lb = fabs(motor_output_x) * sin(convert_radian(degree_now  + (degree_reverse_x - 120) + 90)) - fabs(motor_output_y) * cos(convert_radian(degree_now +( degree_reverse_y - 120) + 90));
	return(motor_output_lb);
}

/******************************************************************************
*	�^�C�g�� �F �E��I���j�^�C���̏o�͌���
*	  �֐��� �F get_motor_output_rb
*	  �߂�l �F float�^
*	    ����1 �Ffloat�^ motor_output_x
*	    ����2 �Ffloat�^ motor_output_y
*	    ����3 �Ffloat�^ degree_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/11/21
******************************************************************************/
float get_motor_output_rb(float motor_output_x,float motor_output_y,float degree_now)
{
	float 	motor_output_rb = 0.0,
		degree_reverse_x = 0.0,
		degree_reverse_y = 0.0;
	
	if(motor_output_x < 0.0){
		degree_reverse_x = 180.0;
	}else{
		degree_reverse_x = 0.0;
		}
	if(motor_output_y < 0.0){
		degree_reverse_y = 180.0;
	}else{
		degree_reverse_y = 0.0;
	}
	
	motor_output_rb = fabs(motor_output_x) * sin(convert_radian(degree_now  + (degree_reverse_x - 60.0) + 90)) - fabs(motor_output_y) * cos(convert_radian(degree_now +( degree_reverse_y - 60.0) + 90));
	
	return(motor_output_rb);
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
void Move_right_flont_tire(float right_flont_duty)
{
	static int i = 0;
	static float right_flont_duty_old = 0.0;

	if(right_flont_duty == BRAKE){
		right_flont_duty = 0.0;
	}
	
	if(((right_flont_duty > 0.0) && (right_flont_duty_old > 0.0)) || ((right_flont_duty < 0.0) && (right_flont_duty_old < 0.0))){
		if(fabs(right_flont_duty - right_flont_duty_old) >= 80.0){
			right_flont_duty = (right_flont_duty + right_flont_duty_old) / 2.000;
		}else{
			right_flont_duty = right_flont_duty;
		}
	}
	
	if(right_flont_duty < 0){
		if(i != 0){
			Deadtime_right_flont_tire();
			if(g_right_flont_motor_timer_count >= 10){
				i = 0;
			}
		}else{
			g_right_flont_motor_timer_count = 0;
			right_flont_duty *= (-1);
			RIGHT_FLONT_CW = OFF;
			RIGHT_FLONT_CCW = ON;
			i = 0;
		}
	}else if(right_flont_duty == BRAKE){
		if(i != 1){
			Deadtime_right_flont_tire();
			if(g_right_flont_motor_timer_count >= 10){
				i = 1;
			}
		}else{
			g_right_flont_motor_timer_count = 0;
			RIGHT_FLONT_CW = ON;
			RIGHT_FLONT_CCW = ON;
			right_flont_duty = 100.0;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_right_flont_tire();
			if(g_right_flont_motor_timer_count >= 10){
				i = 2;
			}
		}else{
			g_right_flont_motor_timer_count = 0;
			RIGHT_FLONT_CW = ON;
			RIGHT_FLONT_CCW = OFF;
			i = 2;
		}
	}
	
	if(right_flont_duty > 5){
		right_flont_duty =	Limit_ul(LIMIT_MOTOR_DUTY_TIRE , 0	, right_flont_duty);
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
void Move_left_flont_tire(float left_flont_duty)
{
	static int i = 0;
	static float left_flont_duty_old = 0.0;
	if(left_flont_duty == BRAKE){
		left_flont_duty = 0.0;
	}
	
	if(((left_flont_duty > 0.0) && (left_flont_duty_old > 0.0)) || ((left_flont_duty < 0.0) && (left_flont_duty_old < 0.0))){
		if(fabs(left_flont_duty - left_flont_duty_old) >= 80.0){
			left_flont_duty = (left_flont_duty + left_flont_duty_old) / 2.000;
		}else{
			left_flont_duty = left_flont_duty;
		}
	}
	
	if(left_flont_duty < 0){
		if(i != 0){
			Deadtime_left_flont_tire();
			if(g_left_flont_motor_timer_count >= 10){
				i = 0;
			}
		}else{
			g_left_flont_motor_timer_count = 0;
			left_flont_duty *= (-1);
			LEFT_FLONT_CW = OFF;
			LEFT_FLONT_CCW = ON;
			i = 0;
		}
	}else if(left_flont_duty == BRAKE){
		if(i != 1){
			Deadtime_left_flont_tire();
			if(g_left_flont_motor_timer_count >= 10){
				i = 1;
			}
		}else{
			g_left_flont_motor_timer_count = 0;
			LEFT_FLONT_CW = ON;
			LEFT_FLONT_CCW = ON;
			left_flont_duty = 100;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_left_flont_tire();
			if(g_left_flont_motor_timer_count >= 10){
				i = 2;
			}	
		}else{
			g_left_flont_motor_timer_count = 0;
			LEFT_FLONT_CW = ON;
			LEFT_FLONT_CCW = OFF;
			i = 2;
		}
	}
	
	if(left_flont_duty > 5){
		left_flont_duty = Limit_ul(LIMIT_MOTOR_DUTY_TIRE,0.0,left_flont_duty);
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
void Move_left_back_tire(float left_back_duty)
{
	static int i = 0;
	static float left_back_duty_old = 0.0;
	
	if(left_back_duty == BRAKE){
		left_back_duty = 0.0;
	}
	
	if(((left_back_duty > 0.0) && (left_back_duty_old > 0.0)) || ((left_back_duty < 0.0) && (left_back_duty_old < 0.0))){
		if(fabs(left_back_duty - left_back_duty_old) >= 80.0){
			left_back_duty = (left_back_duty + left_back_duty_old) / 2.000;
		}else{
			left_back_duty = left_back_duty;
		}
	}
	
	if(left_back_duty < 0){
		if(i != 0){
			Deadtime_left_back_tire();
			if(g_left_back_motor_timer_count >= 10){
				i = 0;
			}
		}else{
			g_left_back_motor_timer_count = 0;
			left_back_duty *= (-1);
			LEFT_BACK_CW = OFF;
			LEFT_BACK_CCW = ON;
			i = 0;
		}
	}else if(left_back_duty == BRAKE){
		if(i != 1){
			Deadtime_left_back_tire();
			if(g_left_back_motor_timer_count >= 10){
				i = 1;
			}
		}else{
			g_left_back_motor_timer_count = 0;
			LEFT_BACK_CW = ON;
			LEFT_BACK_CCW = ON;
			left_back_duty = 100;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_left_back_tire();
			if(g_left_back_motor_timer_count >= 10){
				i = 2;
			}	
		}else{
			g_left_back_motor_timer_count = 0;
			LEFT_BACK_CW = ON;
			LEFT_BACK_CCW = OFF;
			i = 2;
		}
	}
	
	if(left_back_duty > 5){
		left_back_duty = Limit_ul(LIMIT_MOTOR_DUTY_TIRE,0.0,left_back_duty);
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
void Move_right_back_tire(float right_back_duty)
{
	static int i = 0;
	static float right_back_duty_old = 0.0;

	if(right_back_duty == BRAKE){
		right_back_duty = 0.0;
	}
	
	if(((right_back_duty > 0.0) && (right_back_duty_old > 0.0)) || ((right_back_duty < 0.0) && (right_back_duty_old < 0.0))){
		if(fabs(right_back_duty - right_back_duty_old) >= 80.0){
			right_back_duty = (right_back_duty + right_back_duty_old) / 2.000;
		}else{
			right_back_duty = right_back_duty;
		}
	}
	
	//right_duty = (int)right_duty_row;
	
	if(right_back_duty < 0){
		if(i != 0){
			Deadtime_right_back_tire();
			if(g_right_back_motor_timer_count >= 10){
				i = 0;
			}
		}else{
			g_right_back_motor_timer_count = 0;
			right_back_duty *= (-1);
			RIGHT_BACK_CW = OFF;
			RIGHT_BACK_CCW = ON;
			i = 0;
		}
	}else if(right_back_duty == BRAKE){
		if(i != 1){
			Deadtime_right_back_tire();
			if(g_right_back_motor_timer_count >= 10){
				i = 1;
			}
		}else{
			g_right_back_motor_timer_count = 0;
			RIGHT_BACK_CW = ON;
			RIGHT_BACK_CCW = ON;
			right_back_duty = 100.0;
			i = 1;
		}
	}else{
		if(i != 2){
			Deadtime_right_back_tire();
			if(g_right_back_motor_timer_count >= 10){
				i = 2;
			}
		}else{
			g_right_back_motor_timer_count = 0;
			RIGHT_BACK_CW = ON;
			RIGHT_BACK_CCW = OFF;
			i = 2;
		}
	}
	
	if(right_back_duty > 5){
		right_back_duty =	Limit_ul(LIMIT_MOTOR_DUTY_TIRE , 0	, right_back_duty);
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
void Move( float right_flont_duty, float left_flont_duty, float left_back_duty, float right_back_duty)
{
	Move_left_flont_tire(left_flont_duty);
	Move_right_flont_tire(right_flont_duty);
	Move_left_back_tire(left_back_duty);
	Move_right_back_tire(right_back_duty);
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
//	unsigned char index;
	unsigned int angle;
//	float angle_f;
	static float start_Rate	= 0;
	unsigned int rate;
//	float rate_f;
//	float now_Rate;
	unsigned int x_acc;
	unsigned int y_acc;
	unsigned int z_acc;
//	unsigned int reserved;
	unsigned char check_sum;
	char string[20] = {0};
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

			//�p�P�b�g�̃w�b�_�[�����m�F����
			if(g_input_r1350n[0] != 0xAA){
				sprintf(string, "Heading ERROR");
				transmission_string(string);
			}
			
			//�f�[�^��g�ݗ��Ă�
//			index = g_input_r1350n[2];
			rate = (g_input_r1350n[3] & 0xFF) | ((g_input_r1350n[4] << 8) & 0xFF00);
			angle = (g_input_r1350n[5] & 0xFF) | ((g_input_r1350n[6] << 8) & 0XFF00);
//			rate_f = (g_input_r1350n[3] & 0xFF) | ((g_input_r1350n[4] << 8) & 0xFF00);
//			angle_f = (g_input_r1350n[5] & 0xFF) | ((g_input_r1350n[6] << 8) & 0XFF00);
			x_acc = (g_input_r1350n[7] & 0xFF) | ((g_input_r1350n[8] << 8) & 0xFF00);
			y_acc = (g_input_r1350n[9] & 0xFF) | ((g_input_r1350n[10] << 8) & 0XFF00);
			z_acc = (g_input_r1350n[11] & 0xFF) | ((g_input_r1350n[12] << 8) & 0xFF00);
//			reserved = g_input_r1350n[13];
			
			//�`�F�b�N�T���̊m�F
			check_sum = 	g_input_r1350n[2] + g_input_r1350n[3] + g_input_r1350n[4] + g_input_r1350n[5]
					     + g_input_r1350n[6] + g_input_r1350n[7] + g_input_r1350n[8] + g_input_r1350n[9]
					     + g_input_r1350n[10] + g_input_r1350n[11] + g_input_r1350n[12] + g_input_r1350n[13];
			
			if(check_sum != g_input_r1350n[14]){
				sprintf(string, "Check_Sum ERROR");
				transmission_string(string);
			}
			if( start_flug == 0 ){
				start_flug = 1;
				start_Rate = rate / 100.0;
				if(start_Rate > 179){
					start_Rate = start_Rate - 655.0;
				}
				while(start_Rate > 179){
					start_Rate -= 360;
				}
				while(start_Rate < -179){
					start_Rate += 360;
				}
			}
			
			//�p�x�Ɗp���x�̒P�ʂ�ʏ�l�i���ɖ߂��f�[�^���L������
			g_Rate = rate / 100;
			g_Angle = angle / 100;
			g_Rate_f = rate / 100.0;
			g_Angle_f = angle / 100.0;
			g_X_acc = x_acc;
			g_Y_acc = y_acc;
			g_Z_acc = z_acc;
			
			if(g_Rate > 179){
				g_Rate = g_Rate - 655;
			}
			
			if(g_Angle > 179){
				g_Angle = g_Angle - 655;
			}
			
			if(g_Rate_f > 179){
				g_Rate_f = g_Rate_f - 655.0;
			}
			while(g_Rate_f > 179){
				g_Rate_f -= 360;
			}
			while(g_Rate_f < -179){
				g_Rate_f += 360;
			}
			
			g_Rate_f -= start_Rate;
			
			if(g_Rate_f > 179){
				g_Rate_f = g_Rate_f - 655.0;
			}
			while(g_Rate_f > 179){
				g_Rate_f -= 360;
			}
			while(g_Rate_f < -179){
				g_Rate_f += 360;
			}
			
			if(g_Angle_f > 179){
				g_Angle_f = g_Angle_f - 655.0;
			}
//				sprintf(string, "%d\n\r",rate);
//				transmission_string(string);
		}
	}
}
/******************************************************************************
*	�^�C�g�� �F���iX�������o�͌���
*	  �֐��� �F straight_output_x
*	  �߂�l �F float�^ 
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/09/25
******************************************************************************/
float straight_output_x( void )
{	
	float straight_cal_x = 0.00;
	
	straight_cal_x = ( 255.0 - (float)LEFT_STICK_HIGH ) / 255.0;
	straight_cal_x = ( straight_cal_x - 0.5 ) * 2;
	if( fabs( straight_cal_x ) <= STICK_NO_MOVE_RANGE ){
		straight_cal_x = 0;
	}
	straight_cal_x *= PWM_PER;
	
	return(straight_cal_x);
}
/******************************************************************************
*	�^�C�g�� �F���iY�������o�͌���
*	  �֐��� �F straight_output_y
*	  �߂�l �F float�^ 
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/09/25
******************************************************************************/
float straight_output_y( void )
{	
	float straight_cal_y = 0.00;
	
	straight_cal_y = ( 255.0 - (float)LEFT_STICK_WIDE ) / 255.0;
	straight_cal_y = ( straight_cal_y - 0.5 ) * 2;
	if( fabs( straight_cal_y ) <= STICK_NO_MOVE_RANGE ){
		straight_cal_y = 0;
	}
	straight_cal_y *= PWM_PER;
	
	return(straight_cal_y);
}

/******************************************************************************
*	�^�C�g�� �F ��]�����o�͌���
*	  �֐��� �F turn_output
*	  �߂�l �F float�^ 
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/09/25
******************************************************************************/
float turn_output( void )
{	
	float turn_cal = 0.00;
	
	turn_cal = ( 255.0 - (float)RIGHT_STICK_WIDE ) / 255.0;
	turn_cal = ( turn_cal - 0.5 ) * 2;
	if( fabs( turn_cal ) <= STICK_NO_MOVE_RANGE ){
		turn_cal = 0;
	}
	turn_cal *= ( OPERATE_DEGREE / ( 1.00 / ( INTERRUPT_TIME / 1000.0 ) ));
	
	return(turn_cal);
}

/******************************************************************************
*	�^�C�g�� �F ��]PD����
*	  �֐��� �F Turn_PD
*	  �߂�l �F float�^ 
*	   ����1 �F float�^ target_degree  
*	   ����2 �F float�^ now_degree  
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/09/25
******************************************************************************/
float Turn_PD( float target_degree , float now_degree )
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

/******************************************************************************
*	�^�C�g���F ���ݒn����ڕW�l�܂ł̊p�x��Ԃ�
*	  �֐��� �F get_target_degree
*	  �߂�l �F float�^
*	    ����1�F float�^ target_x
*	    ����2�F float�^ target_y
*	    ����3�F float�^ x_now
*	    ����4�F float�^ y_now
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/08/30
******************************************************************************/
float get_target_degree(float target_x,float target_y,float x_now,float y_now)
{
	float target_degree = 0.0;
	static float target_degree_old = 0.0;

	if((target_x - x_now != 0.0) || (target_y - y_now != 0.0)){
		target_degree = atan2(target_y - y_now,target_x - x_now) * 180 /M_PI;
		target_degree_old = target_degree;
	}else{
		target_degree = target_degree_old;//�����ĂȂ�������O�̊p�x
		}

	return (target_degree);
}

/******************************************************************************
*	�^�C�g�� �F2�_�̋���
*	  �֐��� �F get_distance
*	  �߂�l �F float�^
*	    ����1�F float�^ target_x
*	    ����2�F float�^ target_y
*	    ����3�F float�^ now_x
*	    ����4�F float�^ now_y
*	  �쐬�� �F �Έ�x�j
*	  �쐬�� �F 2014/09/26
******************************************************************************/
float get_distance(float target_x,float target_y,float now_x,float now_y)
{
	float distance = 0.00;
	distance = sqrtf(powf(target_x - now_x,2) + powf(target_y - now_y,2));
	return (distance);
}

/******************************************************************************
*	�^�C�g�� �F�p�x��͈͓��Ɏ��߂�
*	  �֐��� �F revision_degree
*	  �߂�l �F float�^ 
*	   ����1 �F float�^ degree  
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/09/25
******************************************************************************/
float revision_degree(float degree)
{
	while(degree > 179){
		degree = degree - 360;
	}
	while(degree < -179){
		degree = degree + 360;
	}
	return(degree);
}

/******************************************************************************
*	�^�C�g�� �F�ڕW���x�A�o�͌v�Z
*	  �֐��� �F get_motor_output
*	  �߂�l �F void�^
*	    ����1�F float�^ target_x
*	    ����2�F float�^ target_y
*	    ����3�F float�^ now_x
*	    ����4�F float�^ now_y
*	    ����5�F float�^ degree
*	    ����6�F float�^ a_up
*	    ����7�F float�^ a_down
*	    ����8�F float�^ max_velocity
*	  �쐬�� �F �Έ�x�j
*	  �쐬�� �F 2014/09/26
******************************************************************************/
void get_motor_output(float x_start,  float y_start,  float now_x,  float now_y,  float target_x, float target_y, float degree, float a_up,  float a_down,  float max_velocity,  int task)
{
	float	motor_output_x 	= 0.00,
		motor_output_y 	= 0.00,
		target_distance 		= 0.00,	
		target_degree		= 0.00,
		p_gain 			= 0.15,
		x_target_velocity 	= 0.00, 
		y_target_velocity	 = 0.00;
	static float target_velocity	= 0.00;
	static int task_old = 0;
	char string[100] = { 0 };
	
	if(task != task_old){
		target_velocity = 0.0;
	}

	target_degree = get_target_degree(target_x,target_y,now_x,now_y);

	target_distance = get_distance(target_x,target_y,now_x,now_y);

	if(target_distance > 0.5 * max_velocity * max_velocity / a_down ){
		if(target_velocity < max_velocity){
			target_velocity += (a_up * 0.001 * INTERRUPT_TIME);
		}else{
			target_velocity = max_velocity;
		}
	}else{
		target_velocity = sqrtf( 2 * a_down * target_distance);
	}
	
	x_target_velocity = target_velocity * cos(convert_radian(target_degree));
	y_target_velocity = target_velocity * sin(convert_radian(target_degree));

	motor_output_x = ( x_target_velocity - g_x_velocity ) * p_gain;
	motor_output_y = ( y_target_velocity - g_y_velocity ) * p_gain;
	
	motor_output_velocity.lf = get_motor_output_lf(motor_output_x,motor_output_y,degree);
	motor_output_velocity.rf = get_motor_output_rf(motor_output_x,motor_output_y,degree);
	motor_output_velocity.lb = get_motor_output_lb(motor_output_x,motor_output_y,degree);
	motor_output_velocity.rb = get_motor_output_rb(motor_output_x,motor_output_y,degree);
//	sprintf(string,"%f,%f,%f,%f,%f,%f,%f,%f\n\r",motor_output,motor_output_x,motor_output_y,direction_degree_now,direction_velocity,target_velocity,target_x,target_y);
//	transmission_string(string);
	task_old = task;
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

int main(void)
{
	int 	vertical_enc_count = 0,
		horizontal_enc_count = 0,
		old_vertical_enc_count	= 0,
		old_horizontal_enc_count	= 0,
		task = 0,
		motor = ON;
		
	float	add_distance_vertical		= 0.00,
		add_distance_horizontal	= 0.00,
		add_distance			= 0.00,
		add_distance_degree		= 0.00;
	
	float	degree 				= 0.00,
		old_degree 			= 0.00,	
		target_degree 			= 0.00;
		
	float	Motor_output_turn 		= 0.00,
		task_start_x			= START_X_COORDNATES,
		task_start_y			= START_Y_COORDNATES;
		
	float	a_up 				= 2000.0,
		a_down 				= 2000.0,
		max_velocity 			= 2000.0;
		
	float old_x = START_X_COORDNATES,
		old_y = START_Y_COORDNATES;
		
	char string[100] = { 0 };
	
	struct coordinates COORD[5] = {
		{ 5000.0, 0.00, 0.00},
		{ 5000.0, 5000.0, 0.00},
		{ 0.00, 5000.0 , 0.00},
		{ 0.00, 0.00, 0.00},
		{ BRAKE,BRAKE,BRAKE },
	};
	
	All_setup();

	PORT8.DDR.BIT.B1 = 1;
	PORTD.DDR.BIT.B7 = 1;
	
	while(1){
		if( g_interrupt_timer_count2 >= 400 ){
			if(g_interrupt_timer_count >= INTERRUPT_TIME){
				g_interrupt_timer_count = 0;
				
				if( fabs( -g_Rate_f - old_degree ) < 30 || fabs( -g_Rate_f - old_degree ) > 330){
					degree = -g_Rate_f;
				}
				
				vertical_enc_count = VERTICAL_ENCODER  + ( 65536 * g_over_vertical_count)  + ( ( -65536 ) * g_under_vertical_count  ); //�����G���R�[�_�[�̒l
				horizontal_enc_count = HORIZONTAL_ENCODER  + ( 65536 * g_over_horizontal_count)  + ( ( -65536 ) * g_under_horizontal_count  ); //�����G���R�[�_�[�̒l				
				
				add_distance_vertical = ( ( vertical_enc_count - old_vertical_enc_count ) * PI * DIAMETER_VERTICAL_WHEEL ) / ( PULSE_VERTICAL_ENCODER * 4 );
				add_distance_horizontal = ( ( horizontal_enc_count - old_horizontal_enc_count ) * PI * DIAMETER_HORIZONTAL_WHEEL ) / ( PULSE_HORIZONTAL_ENCODER * 4 );
				add_distance = pow(add_distance_vertical * add_distance_vertical + add_distance_horizontal * add_distance_horizontal,0.5);
				
				if(add_distance_horizontal != 0 || add_distance_vertical != 0 ){
					add_distance_degree = atan2( add_distance_horizontal , add_distance_vertical ) * 180 / PI;
					g_x_coordnates += add_distance * cos( convert_radian( add_distance_degree + degree  )); 
					g_y_coordnates += add_distance * sin( convert_radian( add_distance_degree + degree  ));
				}
				g_x_velocity = (g_x_coordnates - old_x) / ( (float)INTERRUPT_TIME / 1000 );
				g_y_velocity = (g_y_coordnates - old_y) / ( (float)INTERRUPT_TIME / 1000 );
				g_velocity = pow(pow((g_x_coordnates - old_x) / ( (float)INTERRUPT_TIME / 1000 ),2) + pow((g_y_coordnates - old_y) / ( (float)INTERRUPT_TIME / 1000 ),2),0.5);
				
				get_motor_output( task_start_x , task_start_y , g_x_coordnates , g_y_coordnates, COORD[task].X , COORD[task].Y , degree , a_up , a_down , max_velocity , task );
//				get_target_angular_velocity(difference_degree, angle_a_up, angle_a_down, max_angular_velocity);
				target_degree = revision_degree(target_degree + turn_output());
				
				Motor_output_turn = Turn_PD( target_degree , degree );
				if( get_distance( COORD[task].X, COORD[task].Y, g_x_coordnates, g_y_coordnates) < 10 && COORD[task+1].X == BRAKE){
					motor = OFF;
				}else if( get_distance( COORD[task].X, COORD[task].Y, g_x_coordnates, g_y_coordnates) < 3 ){
					task ++;
					task_start_x = g_x_coordnates;
					task_start_y = g_y_coordnates;
				}
				sprintf(string,"%f,%f,%f,%d\n\r",degree,g_x_coordnates,g_y_coordnates,task);
				transmission_string(string);
				
				old_vertical_enc_count = vertical_enc_count;
				old_horizontal_enc_count = horizontal_enc_count;
				old_degree = degree;
				old_x = g_x_coordnates;
				old_y = g_y_coordnates;
				
				if(g_interrupt_timer_count3 > 400 ){
					g_interrupt_timer_count3 = 0;
//					sprintf(string,"%f\n\r",g_velocity);
//					transmission_string(string);
				}
				if(motor == OFF){
					Move( 0, 0, 0, 0);
				}else{
					Move( motor_output_velocity.rf + Motor_output_turn , motor_output_velocity.lf + Motor_output_turn, motor_output_velocity.lb + Motor_output_turn, motor_output_velocity.rb + Motor_output_turn); 
				}
			}
		}
	}
}