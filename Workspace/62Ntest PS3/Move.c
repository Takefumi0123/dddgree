#include"move.h"
#include"iodefine.h"

typedef struct{
	float	rf;
	float	rb;
	float	lf;
	float	lb;
}motor_timer_count_t;

motor_timer_count_t	motor_timer_count;

void wait_timer_count(void)
{
	IR(CMT0,CMI0) = OFF;
    PORT8.DR.BIT.B1 = 1;	
	motor_timer_count.rf ++;
	motor_timer_count.rb ++;
	motor_timer_count.lf ++;
	motor_timer_count.lb ++;
}

/******************************************************************************
*	タイトル ： モータ関数
*	  関数名 ： Move
*	  戻り値 ： void型
*	    引数1： float型 left_duty
*	    引数2： float型 right_duty
*	    引数3： float型 back_duty
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Move(float	right_flont_duty, float	left_flont_duty,float	left_back_duty,	float	right_back_duty ,float limit_duty)
{
	Move_left_flont_tire(left_flont_duty,limit_duty);
	Move_right_flont_tire(right_flont_duty,limit_duty);
	Move_left_back_tire(left_back_duty,limit_duty);
	Move_right_back_tire(right_back_duty,limit_duty);
}

/******************************************************************************
*	タイトル ： 右前タイヤの出力リセット
*	  関数名 ： Deadtime_right_flont_tire
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Deadtime_right_flont_tire(void)
{
	RIGHT_FLONT_CW = OFF;
	RIGHT_FLONT_CCW = OFF;
	RIGHT_FLONT_DUTY = OFF;
}

/******************************************************************************
*	タイトル ： 右前タイヤの動作
*	  関数名 ： Move_right_flont_tire
*	  戻り値 ： void型
*	    引数1： float型 right_duty 
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
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
	
	if(right_flont_duty > 5){
		right_flont_duty =	Limit_ul(limit_duty , 0, right_flont_duty);
		RIGHT_FLONT_DUTY = ((PWM_PERIOD * right_flont_duty) / 100.0);
		}
	else{
		Deadtime_right_flont_tire();
		}
	
	right_flont_duty_old = right_flont_duty;
}

/******************************************************************************
*	タイトル ： 左前タイヤの出力リセット
*	  関数名 ： Deadtime_left_tire
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Deadtime_left_flont_tire(void)
{
	LEFT_FLONT_CW = OFF;
	LEFT_FLONT_CCW = OFF;
	LEFT_FLONT_DUTY = OFF;
}

/******************************************************************************
*	タイトル ： 左前タイヤ動作
*	  関数名 ： Move_left_tire
*	  戻り値 ： void型
*	    引数1： float型 left_duty 
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
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
	
	if(left_flont_duty > 5){
		left_flont_duty = Limit_ul(limit_duty,0.0,left_flont_duty);
		LEFT_FLONT_DUTY = (PWM_PERIOD * left_flont_duty) / 100;
	}else{
		Deadtime_left_flont_tire();
	}
	
	left_flont_duty_old = left_flont_duty;
}

/******************************************************************************
*	タイトル ： 左後タイヤの出力リセット
*	  関数名 ： Deadtime_left_tire
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Deadtime_left_back_tire(void)
{
	LEFT_BACK_CW = OFF;
	LEFT_BACK_CCW = OFF;
	LEFT_BACK_DUTY = OFF;
}

/******************************************************************************
*	タイトル ： 左後タイヤ動作
*	  関数名 ： Move_left_back_tire
*	  戻り値 ： void型
*	    引数1： float型 left_back_duty 
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
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
	
	if(left_back_duty > 5){
		left_back_duty = Limit_ul(limit_duty,0.0,left_back_duty);
		LEFT_BACK_DUTY = ((PWM_PERIOD * left_back_duty) / 100);
	}else{
		Deadtime_left_back_tire();
	}
	
	left_back_duty_old = left_back_duty;
}

/******************************************************************************
*	タイトル ： 右後タイヤの出力リセット
*	  関数名 ： Deadtime_right_back_tire
*	  戻り値 ： void型
*	    引数 ： なし
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
******************************************************************************/
void Deadtime_right_back_tire(void)
{
	RIGHT_BACK_CW = OFF;
	RIGHT_BACK_CCW = OFF;
	RIGHT_BACK_DUTY = OFF;
}

/******************************************************************************
*	タイトル ： 右後タイヤの動作
*	  関数名 ： Move_right_back_tire
*	  戻り値 ： void型
*	    引数1： float型 right_back_duty 
*	  作成者 ： 坂下文彦
*	  作成日 ： 2013/10/24
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
	
	if(right_back_duty > 5){
		right_back_duty =	Limit_ul(limit_duty , 0	, right_back_duty);
		RIGHT_BACK_DUTY = ((PWM_PERIOD * right_back_duty) / 100.0);
		}
	else{
		Deadtime_right_back_tire();
		}
	
	right_back_duty_old = right_back_duty;
}

