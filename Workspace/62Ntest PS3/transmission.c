#include"transmission.h"
#include"iodefine.h"

/******************************************************************************
*	�^�C�g�� �F �V���A���ʐM
*	  �֐��� �F transmission
*	  �߂�l �F void�^ �V���A���ʐM
*	   ����1 �F char�^ *s  
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/06/21
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
