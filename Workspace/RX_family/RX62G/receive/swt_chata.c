#include "swt_chata.h"

/******************************************************************************
*	�^�C�g�� �F chata_down
*	  �֐��� �F chata_down
*	  �߂�l �F int�^ �X�C�b�`�`���^�����O�֐�
*	   ����1 �F int�^ swt  
*	   ����2 �F int�^ i  
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/07/17
******************************************************************************/

//�`���^�����O�i�_�E���G�b�W�j
//�X�C�b�`������������i����������j�Ԑ؂�ւ��̂� swt_keep
//�X�C�b�`�������i�����j�Ɛ؂�ւ��̂� swt_change
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
	
	return swt_change[i]; //�p�����
//	return swt_keep[i]; //��ԕω�	
}