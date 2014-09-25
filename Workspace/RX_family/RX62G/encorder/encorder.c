#include"iodefine.h"
#include<stdio.h>
#include<math.h>

#define PCLK	48
#define bitreet	9600	//�r�b�g���[�g
#define pulse_LEFT	500	//�G���R�[�_�[�̃p���X
#define pulse_RIGHT	500	//�G���R�[�_�[�̃p���X
#define M_PI	3.141592653589793	//�~����
#define diameter_LEFT	56	//���a		
#define diameter_RIGHT	56	//���a	
#define flank		200	//���ւ̕�

int g_over_LEFT = 0,
    g_under_LEFT = 0,
    g_over_RIGHT = 0,
    g_under_RIGHT = 0;
    
//�I�[�o�[�t���[�������A�ϐ�g_over��1����
void over_flow_LEFT(void){
	g_over_LEFT++;
}

//�A���_�[�t���[�������A�ϐ�g_under�ɂP����
void under_flow_LEFT(void){
	g_under_LEFT++;
}

//�I�[�o�[�t���[�������A�ϐ�g_over��1����
void over_flow_RIGHT(void){
	g_over_RIGHT++;
}

//�A���_�[�t���[�������A�ϐ�g_under�ɂP����
void under_flow_RIGHT(void){
	g_under_RIGHT++;
}

void init_clock(void){
	
	SYSTEM.SCKCR.BIT.PCK = 1; //�~4 ICK�Ƃ͊֌W�Ȃ��ꏊ�̑���
	SYSTEM.SCKCR.BIT.ICK = 0; //�~8 �}�C�R�����̂��̂̑���
	
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

void isou_MTU2(void){
	
	IOPORT.PFCMTU.BIT.TCLKS = 1;  //MTCLKA�[�q�Ƃ��đI��
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //���W���[���X�g�b�v��Ԃ̉���
		
	PORT1.ICR.BIT.B1 = 1; //�|�[�g�I��
	PORT1.ICR.BIT.B0 = 1; //�|�[�g�I��	

	MTU.TSTRA.BIT.CST2 = 0; //MTU2�̃J�E���g����̒�~
		
	MTU2.TCR.BIT.CCLR = 0; //TCNT�̃N���A�֎~ P390	
	
	MTU2.TMDR1.BIT.MD = 0x04; //�ʑ��v�����[�h�P P393
	
	MTU2.TIER.BIT.TCIEV = 1; //�I�[�o�[�t���[�����ݗv��������
	MTU2.TIER.BIT.TCIEU = 1; //�A���_�[�t���[�����ݗv��������
	
	IEN(MTU2,TCIV2) = 1;
	IEN(MTU2,TCIU2) = 1;	
	
	IPR(MTU2,TCIV2) = 15;
	IPR(MTU2,TCIU2) = 15;

	MTU2.TCNT = 0x00; //������
	MTU.TSTRA.BIT.CST2 = 1; //MTU2�̃J�E���g����̊J�n

}


void isou_MTU1(void){
	
	IOPORT.PFCMTU.BIT.TCLKS = 1;  //MTCLKA�[�q�Ƃ��đI��
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //���W���[���X�g�b�v��Ԃ̉���
	
	PORT2.ICR.BIT.B1 = 1; //�|�[�g�I��
	PORT2.ICR.BIT.B0 = 1; //�|�[�g�I��

	MTU.TSTRA.BIT.CST1 = 0; //MTU1�̃J�E���g����̒�~
	
	MTU1.TCR.BIT.CCLR = 0; //TCNT�̃N���A�֎~ P390
	
	MTU1.TMDR1.BIT.MD = 0x04; //�ʑ��v�����[�h�P P393
	
	MTU1.TIER.BIT.TCIEV = 1; //�I�[�o�[�t���[�����ݗv��������
	MTU1.TIER.BIT.TCIEU = 1; //�A���_�[�t���[�����ݗv��������
	
	IEN(MTU1,TCIV1) = 1;
	IEN(MTU1,TCIU1) = 1;	
	
	IPR(MTU1,TCIV1) = 15;
	IPR(MTU1,TCIU1) = 15;

	MTU1.TCNT = 0x00; //������
	MTU.TSTRA.BIT.CST1 = 1; //MTU1�̃J�E���g����̊J�n

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
	
	int tcnt_LEFT,
	    tcnt_RIGHT;
	double length_LEFT,
	       length_RIGHT,
	       angle,
	       rad,
	       rad2,
	       angle2,
	       hyouji_rad,
	       x,
	       y;
	
	char s[100]={0};
	
	init_clock();
	serial();
	isou_MTU1();
	isou_MTU2();


	while(1){

		tcnt_LEFT = MTU1.TCNT; //�ϐ���TCNT�̐��l���i�[(����)	
		tcnt_RIGHT = MTU2.TCNT; //�ϐ���TCNT�̐��l���i�[(�E��)
		
		length_LEFT = ( ( tcnt_LEFT +(65536 * g_over_LEFT) + (-65536 * g_under_LEFT) ) * M_PI * diameter_LEFT) / (pulse_LEFT *4 ); //���֐i�smm
		length_RIGHT = ( ( tcnt_RIGHT +(65536 * g_over_RIGHT) + (-65536 * g_under_RIGHT) ) * M_PI * diameter_RIGHT) / (pulse_RIGHT *4 ); //�E�֐i�smm		

//		sprintf(s,"%d\n\r",( tcnt +(65536 * g_over) + (65536 * g_under) ));
		
		rad = ( length_RIGHT - length_LEFT ) / flank;

		angle = 180 * ( length_RIGHT - length_LEFT ) / ( flank * M_PI );
		
		angle2 = angle;
		
		if (rad > M_PI ){
			rad2 = -1 * M_PI + rad;
		}
		if (rad <= -1 * M_PI ){
			rad2 = rad + M_PI;
		}
		
		if (angle > 180 ){
			angle2 = -360 + angle;
		}
		if (angle < -180 ){
			angle2 = angle + 360;
		}
		
		y = (90 * sin ( rad ) * ( length_LEFT + length_RIGHT)) / (M_PI * angle);
		//x = ( length_LEFT + length_RIGHT ) / 2 * sin( rad /2 );
		
		x = (90 * sin ( rad ) * -1 * tan ( rad / 2)  * ( length_LEFT + length_RIGHT)) / (M_PI * angle);		
		//y = ( length_LEFT + length_RIGHT ) / 2 * cos( rad /2 );
		
//		sprintf(s,"LEFT : %.4f mm  RIGHT : %.4f mm  angle : %.4f   x : %.4f  y : %.4f\n\r",length_LEFT,length_RIGHT,angle2,x,y);
		sprintf(s,"%d    %d   %d  \n\r",tcnt_LEFT,g_over_LEFT,g_under_LEFT);
		
		send(s);
		
		
	}
}