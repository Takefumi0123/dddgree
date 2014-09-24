#include "iodefine.h"
#include "Move.h"

/******************************************************************************
*	タイトル ： Move
*	  関数名 ： Move
*	  戻り値 ： void型 
*	   引数1 ： float型 L_output  
*	   引数2 ： float型 R_output  
*	  作成者 ： 石井
*	  作成日 ： 2014/07/17
******************************************************************************/

//PWM	引数は％
/*void pwm( float TGRD, float TGRB )
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
}*/	

