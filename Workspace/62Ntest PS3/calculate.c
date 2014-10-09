#include "calculate.h"
#include"iodefine.h"
#include "machine.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/******************************************************************************
*	タイトル ： 度数のラジアン変換
*	  関数名 ： convert_radian
*	  戻り値 ： float型
*	    引数1： float型 degree 
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
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
*	タイトル ： 設定した範囲内の値を返す
*	  関数名 ： Limit_ul
*	  戻り値 ： float型 出力値
*	   引数1 ： float型 upper  上限の数値
*	   引数2 ： float型 lower  下限の数値
*	   引数3 ： float型 figure  比較する数値
*	  作成者 ： 市川 智章
*	  作成日 ： 2011/08/31
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
*	タイトル ：角度を範囲内に収める
*	  関数名 ： revision_degree
*	  戻り値 ： float型 
*	   引数1 ： float型 degree  
*	  作成者 ： 石井
*	  作成日 ： 2014/09/25
******************************************************************************/
float revision_degree(float degree)
{
	int i = 0;

	for(i = 0;  i <= 3; i++){
		//角度の範囲越え防止
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
*	タイトル： 現在地から目標値までの角度を返す
*	  関数名 ： get_target_degree
*	  戻り値 ： float型
*	    引数1： float型 target_x
*	    引数2： float型 target_y
*	    引数3： float型 x_now
*	    引数4： float型 y_now
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/08/30
******************************************************************************/
float get_target_degree(float target_x,float target_y,float x_now,float y_now)
{
	float target_degree = 0.0;
	static float target_degree_old = 0.0;

	if((target_x - x_now != 0.0) || (target_y - y_now != 0.0)){
		target_degree = atan2(target_y - y_now,target_x - x_now) * 180 /M_PI;
		target_degree_old = target_degree;
	}else{
		target_degree = target_degree_old;//動いてなかったら前の角度
		}

	return (target_degree);
}
/******************************************************************************
*	タイトル ：2点の距離
*	  関数名 ： get_distance
*	  戻り値 ： float型
*	    引数1： float型 target_x
*	    引数2： float型 target_y
*	    引数3： float型 now_x
*	    引数4： float型 now_y
*	  作成者 ： 石井岳史
*	  作成日 ： 2014/09/26
******************************************************************************/
float get_distance(float target_x,float target_y,float now_x,float now_y)
{
	float distance = 0.00;
	distance = sqrtf(powf(target_x - now_x,2) + powf(target_y - now_y,2));
	return (distance);
}
