#include"iodefine.h"
#include <stdio.h>
#include <math.h>

#define PCLK	48
#define BITREET	9600	//�r�b�g���[�g
#define PULSE_LEFT	500	//�G���R�[�_�[�̃p���X
#define PULSE_RIGHT	500	//�G���R�[�_�[�̃p���X
#define M_PI	3.141592653589793	//�~����
#define DIAMETER_LEFT	56	//���a		
#define DIAMETER_RIGHT	56	//���a	
#define FLANK		200	//���ւ̕�
#define PGAIN		0.4
#define DGAIN		1.0
#define FRONT_LEFT	PORTB.DR.BIT.B1
#define BACK_LEFT	PORTB.DR.BIT.B3
#define FRONT_RIGHT	PORTA.DR.BIT.B2
#define BACK_RIGHT	PORTB.DR.BIT.B2
#define PLUS_SWITCH	PORTB.PORT.BIT.B4

int g_over_LEFT = 0,
    g_under_LEFT = 0,
    g_over_RIGHT = 0,
    g_under_RIGHT = 0,
    count = 0;

void count_plus(void){
	count ++;
}
     
    
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

void init_cmt0(void){

	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //���W���[���X�g�b�v��Ԃ̉���
	CMT.CMSTR0.BIT.STR0 = 0; //�J�E���g����̒�~
	
	CMT0.CMCR.BIT.CKS = 2;//�N���b�N�I�� 1/128
	CMT0.CMCOR = 375;   //CMCOR�̌��� 48mhz/128/1000
	
	CMT0.CMCNT = 0;//������
	CMT0.CMCR.BIT.CMIE = 1;  //���荞�݋���
	CMT.CMSTR0.BIT.STR0 = 1;
	
	IEN(CMT0,CMI0) = 1; //���荞�ݗv�������W�X�^
	IPR(CMT0,CMI0) = 15; //�D��xMAX
	
}



void init_pwm(void){
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;
	MTU.TSTRB.BIT.CST6 = 0; //�J�E���^����̒�~
	
	MTU6.TCNT = 0x00;
	MTU6.TCR.BIT.TPSC = 0x02; //�N���b�N���� 1/16
	MTU6.TCR.BIT.CKEG = 0x00; //�J�E���g�G�b�W
	MTU6.TCR.BIT.CCLR = 0x01; //�J�E���g�N���A�v��

	
	MTU6.TMDR1.BIT.MD = 0x02; //PWM���[�h�ݒ�
	
	MTU6.TIORH.BIT.IOA = 0x02;
	MTU6.TIORH.BIT.IOB = 0x01;
	MTU6.TIORL.BIT.IOC = 0x02;
	MTU6.TIORL.BIT.IOD = 0x01;
	
	MTU6.TGRA = 3000; //�����ݒ�
	MTU6.TGRB = 3000; //�����ݒ�
	
	MTU6.TGRC = 3000; //�����ݒ�
	MTU6.TGRD = 3000; //�����ݒ�
	
//	MTU.TOERB.BIT.OE6A = 1;	
	MTU.TOERB.BIT.OE6B = 1;
//	MTU.TOERB.BIT.OE6C = 1;
	MTU.TOERB.BIT.OE6D = 1;
	
	MTU.TSTRB.BIT.CST6 = 1; //�J�E���^����̊J�n
	
}

void init_serial(void){
	
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
	
	SCI1.BRR = PCLK * 1000000 / ( 64 * 0.5 * BITREET ) - 1; //BRR���W�X�^�̐ݒ�l P822
	
	for(i=0;i > 80000;i++);
	
	SCI1.SCR.BIT.RE = 1; //�V���A����M��������� P815
	SCI1.SCR.BIT.TE = 1; //�V���A�����M��������� P815
	
}

void init_counter_MTU2(void){
	
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


void init_counter_MTU1(void){
	
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

void pwm2(double TGRB,double TGRD){
	MTU6.TGRB = 30 * ( 100 - TGRB );
	MTU6.TGRD = 30 * (  100 - TGRD );
	
}

void Move(double L_output,double R_output)
{
	int straight = 0;
	//L_output = L_output  + straight;
	//R_output = -1 * R_output  + straight;
	
    if (L_output >= 100){
        L_output = 99;
    }
    if (L_output <= -100){
        L_output = -99;
    }

    if (R_output >= 100){
        R_output = 99;
    }
    if (R_output <= -100){
        R_output = -99;
    }    
    
    if (L_output > 0){
        FRONT_LEFT = 1;
        BACK_LEFT = 0;
    }else
    if (L_output < 0){
        FRONT_LEFT = 0;
        BACK_LEFT = 1;
	L_output = -1 * L_output;
    }    

    if (R_output > 0){
        FRONT_RIGHT = 1;
        BACK_RIGHT = 0;
    }else
    if (R_output < 0){
        FRONT_RIGHT = 0;
        BACK_RIGHT = 1;
	R_output = -1 * R_output;
    }    
    
    pwm2(R_output,L_output);
}


void PD( double genzai, double mokuhyou ){
	
	double output = 0.00,
		hensa = 0.00;
	static double old_hensa = 0.00;
	
	hensa = mokuhyou - genzai;
	
	output = ( PGAIN * hensa )/* + ( DGAIN * ( hensa - old_hensa ))*/;
	
	old_hensa = hensa;
	
	Move( output, output );
}
	
//�A�b�v�G�b�W�@�`���^�����O�֐��B5������𖞂�����ON��Ԃ�
int chata_up (int swt1, int i2){

	static int i[3] = { 0, 0, 0},
		   kekka[3] = { 0, 0, 0},
		   zyoutai[3] = { 0, 0, 0};
	
	if(swt1 == 1){
		i[i2]++;
	}else if(swt1 == 0){
		i[i2] = 0;
	}
	
	if(zyoutai[i2] == 0 && i[i2] >= 4){
		kekka[i2] = 1 - kekka[i2];
		zyoutai[i2] = 1;
		
	}
	if(i[i2] == 0){
		zyoutai[i2] = 0;
	}
	
//	return kekka[i2]; //�p�����
	return zyoutai[i2]; //��ԕω�
	
}


int main(void){
	
	int tcnt_LEFT,
	    tcnt_RIGHT,
	    swt1 = 0,
	    swt1_x = 0;
	    
	double length_LEFT,
	       length_RIGHT,
	       enc_LEFT,
	       enc_RIGHT,
	       old_length_LEFT = 0,
	       old_length_RIGHT = 0,
	       d_length_LEFT = 0,
	       d_length_RIGHT = 0,
	       old_enc_LEFT = 0,
	       old_enc_RIGHT = 0,
	       d_enc_LEFT = 0,
	       d_enc_RIGHT = 0,
	       degree,
	       old_degree = 0,
	       d_degree = 0,
	       rad,
	       old_rad = 0,
	       d_rad = 0,
	       x = 0,
	       y = 0,
	       output = 0,
	       straight = 100,
	       TGRB = 0;
	
	init_clock();
	init_serial();
	init_counter_MTU1();
	init_counter_MTU2();
	init_cmt0();
	init_pwm();
	
        FRONT_LEFT = 0;
        BACK_LEFT = 0;
        FRONT_RIGHT = 0;
        BACK_RIGHT = 0;
	
	PORTA.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B1 = 1;
	PORTB.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B3 = 1;
	PORT9.DDR.BYTE = 0x3F;
	PORTB.DDR.BIT.B4 = 0;

	while(1){

		if( count >= 5 ){
			
			count = 0;
			tcnt_LEFT = MTU1.TCNT; //�ϐ���TCNT�̐��l���i�[(����)	
			tcnt_RIGHT = MTU2.TCNT; //�ϐ���TCNT�̐��l���i�[(�E��)
			
			enc_LEFT =  tcnt_LEFT +(65536 * g_over_LEFT) + (-65536 * g_under_LEFT);
			enc_RIGHT =  tcnt_RIGHT +(65536 * g_over_RIGHT) + (-65536 * g_under_RIGHT);
			
			length_LEFT = ( ( tcnt_LEFT +(65536 * g_over_LEFT) + (-65536 * g_under_LEFT) ) * M_PI * DIAMETER_LEFT) / (PULSE_LEFT *4 ); //���֐i�smm
			length_RIGHT = ( ( tcnt_RIGHT +(65536 * g_over_RIGHT) + (-65536 * g_under_RIGHT) ) * M_PI * DIAMETER_RIGHT) / (PULSE_RIGHT *4 ); //�E�֐i�smm		

			degree = 180 * ( length_RIGHT - length_LEFT ) / ( FLANK * M_PI ); //�p�x�i���j
			rad = ( length_RIGHT - length_LEFT ) / FLANK; //�p�x�iradian)

			d_length_LEFT = length_LEFT - old_length_LEFT; //���֕΍�			
			d_length_RIGHT = length_RIGHT - old_length_RIGHT; //�E�֕΍�
			
			d_enc_LEFT = enc_LEFT - old_enc_LEFT; //���֕΍�			
			d_enc_RIGHT = enc_RIGHT - old_enc_RIGHT; //�E�֕΍�
			
			d_degree = degree - old_degree; //�p�x�΍��i���j
			d_rad = rad - old_rad; //�p�x�΍��iradian�j
			
//			x =  x+(90 * sin ( rad ) * -1 * tan ( rad / 2)  * ( d_length_LEFT + d_length_RIGHT)) / (M_PI * degree);
//			y =  y+(90 * sin ( rad ) * ( d_length_LEFT + d_length_RIGHT)) / (M_PI * degree);			
			
//			x = x + ( d_length_LEFT + d_length_RIGHT ) * sin( rad ) / 2; //�����W���݈ʒu�i���j
//			y = y + ( d_length_LEFT + d_length_RIGHT ) * cos( rad ) / 2; //y���W���݈ʒu�i�c�j
			
			x = x + (( d_enc_LEFT + d_enc_RIGHT ) * sin( rad ) * M_PI * DIAMETER_RIGHT)/ (2 *  (PULSE_RIGHT *4 )); //�����W���݈ʒu�i���j
			y = y + (( d_enc_LEFT + d_enc_RIGHT ) * cos( rad ) * M_PI * DIAMETER_RIGHT)/ (2 *  (PULSE_RIGHT *4 )); //y���W���݈ʒu�i�c�j
			
			//�u���֕΍��@�E�֕΍��@�p�x�΍��i���j�@x���W�@y���W�v�o��
//			sprintf(s,"LEFT : %.4f mm  RIGHT : %.4f mm  degree : %.4f   x : %.4f  y : %.4f\n\r",length_LEFT,length_RIGHT,degree,x,y);
			
			
			//�u���֕΍��@�E�֕΍��@�p�x��PU���i���j�@x���W�@y���W�v�o��
//			sprintf(s,"%.4f %.4f %.4f\n\r",degree,x,y);
			
			//1�Â��ϐ��Ɋi�[
			old_length_LEFT = length_LEFT;
			old_length_RIGHT = length_RIGHT;
			old_enc_LEFT = enc_LEFT;
			old_enc_RIGHT = enc_RIGHT;
			old_degree = degree;
			old_rad = rad;
			
			if (degree > 180 ){
				degree = -360 + degree;
			}
			if (degree < -180 ){
				degree= degree + 360;
			}
			PD( degree , 0);

			swt1 = chata_up(1-(PLUS_SWITCH) , 0);
					
/*			if(swt1 == 1 && swt1_x ==0){
				TGRB = TGRB + 10;
				if(TGRB == 100){
					TGRB = 0;}
				swt1_x = 1;
			}
			if(swt1 == 0){
				swt1_x = 0;
			}*/			
		}

	}
}