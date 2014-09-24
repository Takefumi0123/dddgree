#include"init_dafalut.h"
#include"iodefine.h"

/******************************************************************************
*	タイトル ： init_dafault
*	  関数名 ： init_dafault
*	  戻り値 ： void型 初期設定
*	    引数 ： なし
*	  作成者 ： 石井
*	  作成日 ： 2014/07/17
******************************************************************************/

void init_dafault( void )
{	
	//入出力の初期化
	PORTA.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B1 = 1;
	PORTB.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B3 = 1;
	PORTB.DDR.BIT.B4 = 0;
	PORT9.DDR.BYTE   = 0x3F;
	
	//回転方向の初期化
        FRONT_LEFT  	= 0;
        BACK_LEFT    	= 0;
        FRONT_RIGHT 	= 0;
        BACK_RIGHT  	= 0;	
}
