#include"iodefine.h"
#include<stdio.h>

#define PCLK	48
#define bitreet	9600

void init_clock(void){
	
	SYSTEM.SCKCR.BIT.PCK = 1; //�~4 ICK�Ƃ͊֌W�Ȃ��ꏊ�̑���
	SYSTEM.SCKCR.BIT.ICK = 0; //�~8 �}�C�R�����̂��̂̑���
	
}

void serial(void){
	
	int i;
	
	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;
	
	PORTD.ICR.BIT.B5 = 1;
	
	SCI1.SCR.BIT.TEIE = 0; //TEIE�����ݗv�����֎~ P815
	SCI1.SCR.BIT.MPIE = 0; //�ʏ�̎�M���� P815
	SCI1.SCR.BIT.RIE = 1; //RXI�����ERI�����ݗv�������� P815
	SCI1.SCR.BIT.TIE = 1; //TXI�����ݗv�������� P815
	SCI1.SCR.BIT.RE = 0; //�V���A����M������֎~ P815
	SCI1.SCR.BIT.TE = 0; //�V���A�����M������֎~ P815
	
	SCI1.SCR.BIT.CKE = 0; //�����|�[���[�g�W�F�l���[�^�BP815
	
	SCI1.SMR.BIT.CM = 0; //�������������[�h P813
	SCI1.SMR.BIT.CHR = 0; //�f�[�^��8�r�b�g�ő���M P813
	SCI1.SMR.BIT.PE = 0; //�p���e�B�r�b�g�Ȃ� P813
	SCI1.SMR.BIT.PM = 0; //�����p���e�B�ő���M P813
	SCI1.SMR.BIT.STOP = 0; //1�X�g�b�v�r�b�g
	SCI1.SMR.BIT.MP = 0; //�����۾���ʐM�@�\���֎~
	SCI1.SMR.BIT.CKS = 0; //PCLK�N���b�N(n=0)
	
	SCI1.BRR = PCLK * 1000000 / ( 64 * 0.5 * bitreet ) - 1; //BRR���W�X�^�̐ݒ�l P822
	
	for(i=0;i > 80000;i++);
	
	SCI1.SCR.BIT.RE = 1; //�V���A����M��������� P815
	SCI1.SCR.BIT.TE = 1; //�V���A�����M��������� P815
	
}
void ad(void){
	SYSTEM.MSTPCRA.BIT.MSTPA23 = 0;
	
	AD0.ADCSR.BIT.CH = 3; //�V���O�����[�h AN5
	AD0.ADCSR.BIT.ADST = 0; //AD�ϊ���~
	AD0.ADCSR.BIT.ADIE = 0; //AD�ϊ��I���ɂ��ADI�����݋֎~
	
	AD0.ADCR.BIT.MODE = 0; //�V���O�����[�h
	AD0.ADCR.BIT.CKS = 1; //�N���b�N�I�� PCLK/8
	
	AD0.ADDPR.BIT.DPPRC = 0; //AD�f�[�^���W�X�^��10�r�b�g���x�Ŋi�[
	AD0.ADDPR.BIT.DPSEL = 0; //�f�[�^��LSB�l��(������)
	
	AD0.ADSSTR = 0xFF; //�T���v�����O����
	
}
/*void send(int  a){
	int i;
	char s[50]={0};
	
		sprintf(s,"%d\n\r",a);
		

		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			
			if(s[i] == '\0'){
				i=0;
			}
			SCI1.TDR = s[i];
			i++;
		}

}*/

int main(void){
	
	int a;
		int i;
	char s[50]={0};
	init_clock();
	serial();
	ad();
	
	while(1){
		AD0.ADCSR.BIT.ADST = 1; //AD�ϊ��J�n
		
		while(AD0.ADCSR.BIT.ADST == 1);
		
		a = AD0.ADDRD;

	//	send(a);
		
	
		sprintf(s,"%d\n\r",a);
		

		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			
			if(s[i] == '\0'){
				i=0;
			}
			SCI1.TDR = s[i];
			i++;
		}

	}
}