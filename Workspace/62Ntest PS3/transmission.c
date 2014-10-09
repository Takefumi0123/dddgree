#include"transmission.h"
#include"iodefine.h"

/******************************************************************************
*	タイトル ： シリアル通信
*	  関数名 ： transmission
*	  戻り値 ： void型 シリアル通信
*	   引数1 ： char型 *s  
*	  作成者 ： 石井
*	  作成日 ： 2014/06/21
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
