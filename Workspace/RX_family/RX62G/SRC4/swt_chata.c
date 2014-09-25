#include "swt_chata.h"

/******************************************************************************
*	タイトル ： chata_down
*	  関数名 ： chata_down
*	  戻り値 ： int型 スイッチチャタリング関数
*	   引数1 ： int型 swt  
*	   引数2 ： int型 i  
*	  作成者 ： 石井
*	  作成日 ： 2014/07/17
******************************************************************************/

//チャタリング（ダウンエッジ）
//スイッチを押し続ける（離し続ける）間切り替わるのが swt_keep
//スイッチを押す（離す）と切り替わるのが swt_change
int chata_down ( int swt, int i )
{
	static int status[3]	 	= { 0, 0, 0 },
		   swt_change[3] 	= { 0, 0, 0 },
		   swt_keep[3]		= { 0, 0, 0 };
	
	if( swt == 1 ){
		status[i]++;
	}else
	if( swt == 0 ){
		status[i] = 0;
	}
	
	if( swt_keep[i] == 0 && status[i] >= 5 ){
		swt_change[i] 	= 1 - swt_change[i];
		swt_keep[i] 	= 1;
		
	}
	if( status[i] == 0 ){
		swt_keep[i] = 0;
	}
	
	return swt_change[i]; //継続状態
//	return swt_keep[i]; //状態変化	
}