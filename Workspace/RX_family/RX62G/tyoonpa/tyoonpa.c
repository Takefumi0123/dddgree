#include"iodefine.h"
#include<stdio.h>
#include<math.h>

#define PCLK	48
#define bitreet	9600	//�r�b�g���[�g

int count = 0;

void warikomi(void){
	
	count++;
	
}

void init_clock(void){
	
	SYSTEM.SCKCR.BIT.PCK = 1; //�~4 ICK�Ƃ͊֌W�Ȃ��ꏊ�̑���
	SYSTEM.SCKCR.BIT.ICK = 0; //�~8 �}�C�R�����̂��̂̑���
	
}

void init_cmt0(void){

	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //���W���[���X�g�b�v��Ԃ̉���
	CMT.CMSTR0.BIT.STR0 = 0; //�J�E���g����̒�~
	
	CMT0.CMCR.BIT.CKS = 0;//�N���b�N�I�� 1/8
	CMT0.CMCOR = 6;   //CMCOR�̌��� 48mhz/8/1000000
	
	CMT0.CMCNT = 0;//������
	CMT0.CMCR.BIT.CMIE = 1;  //���荞�݋���
	CMT.CMSTR0.BIT.STR0 = 1;

	
	IEN(CMT0,CMI0) = 1; //���荞�ݗv�������W�X�^
	IPR(CMT0,CMI0) = 15; //�D��xMAX
}

void serial(void){
	
	int i;
	
	SYSTEM.MSTPCRB.BIT.MSTPB30 = 0;  //���W���[���X�g�b�v��Ԃ̉���
	
	PORTD.ICR.BIT.B5 = 1; //�|�[�g�I��
	
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
void tyoonpa(void){

	PORT2.DDR.BIT.B4 = 1; //�g���K�[�o��
	PORT2.DDR.BIT.B3 = 0; //�G�R�[����
	
}

void send(char *s){
	int i = 0;
	while(s[i] != '\0'){	
			
		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			SCI1.TDR = s[i];
			i++;
		}
	}
}



int main(void){
	
	int phase =  1;
	double    kyori;
	char s[50] = { 0 };
	
	PORT9.DDR.BYTE = 0x3F;
	PORT9.DR.BYTE = 0x00;
	
	 init_clock();
	 init_cmt0();
	 serial();
	 tyoonpa();

	while ( 1 ){

		/*
		if(IR(CMT0,CMI0) == 1){
			IR(CMT0,CMI0) = 0;
			count ++;
		}*/
		
		if(phase == 1){
			PORT2.DR.BIT.B4 = 1;
			if( count >= 10){
				PORT2.DR.BIT.B4 = 0;
				phase ++;
			}
			PORT9.DR.BYTE |= 0x01;
		}
		else if( phase == 2){
			if( PORT2.PORT.BIT.B3 == 1){
				phase ++;
				count = 0;
			}
			PORT9.DR.BYTE |= 0x02;
		}
		else if( phase == 3){
			PORT9.DR.BYTE |= 0x04;
			if(PORT2.PORT.BIT.B3 == 0){
				PORT9.DR.BYTE |= 0x04;
				phase = 1;
				kyori = count * 0.017;
				
				sprintf(s,"%.3f\n\r",kyori);
				send(s);
				count = 0;
			}
		}
	
	}
}