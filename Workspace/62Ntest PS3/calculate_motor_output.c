#include"calculate_motor_output.h"
#include"iodefine.h"
#include"calculate.h"
#include "machine.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/******************************************************************************
*	�^�C�g�� �F �x���̃��W�A���ϊ�
*	  �֐��� �F convert_radian
*	  �߂�l �F float�^
*	    ����1�F float�^ degree 
*	  �쐬�� �F �≺���F
*	  �쐬�� �F 2013/10/24
******************************************************************************/
/*float convert_radian(float degree)
{
	float radian = 0.0;
	while(degree > 180){
		degree -= 360;
	}
	while(degree < -180){
		degree += 360;
	}
	radian = degree * (M_PI / 180);
	return (radian);
}*/
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