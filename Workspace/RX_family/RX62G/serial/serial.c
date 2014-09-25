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

void send(int a){
	int i;
	char s[50]={0};
	
	while( 1 ){
		sprintf(s,"array:a=%s\n\r",a);
		
		if(s[i] == '\0'){
			i=0;
		}
		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			SCI1.TDR = s[i];
			i++;
			while(SCI1.SSR.BIT.TEND == 1){
			}
		}
	}
}

int main(void){
	
	char a[10] ={"1234"};

	init_clock();
	serial();
	send(a);

}