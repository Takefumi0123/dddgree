#include "calculate.h"
#include"iodefine.h"
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
float convert_radian(float degree)
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
*	�^�C�g�� �F�p�x��͈͓��Ɏ��߂�
*	  �֐��� �F revision_degree
*	  �߂�l �F float�^ 
*	   ����1 �F float�^ degree  
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/09/25
******************************************************************************/
float revision_degree(float degree)
{
	int i = 0;

	for(i = 0;  i <= 3; i++){
		//�p�x�͈͉̔z���h�~
		if(degree > 180){
			degree = degree - 360;
		}
		if(degree < -180){
			degree = degree + 360;
		}
	}

	if((degree > 180) || (degree < -180)){
		return (0);
	}else{
		return (degree);
	}
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
