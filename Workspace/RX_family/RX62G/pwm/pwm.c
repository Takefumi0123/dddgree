#include"iodefine.h"

int count = 0;

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

	
}
void init_MTU7A_PWM(void)
{//p480 PWM���[�h
	MSTP(MTU7)=0;			//p186 0�F���W���[���X�g�b�v��Ԃ̉���
	MTU.TSTRB.BIT.CST7=0;		//p426 MTU7.TCNT�̃J�E���g����͒�~
	MTU7.TCR.BIT.TPSC=0x02;		//p390 �����N���b�N:ICLK/16�ŃJ�E���g
	MTU7.TCR.BIT.CKEG=0x00;		//p388 �����オ��G�b�W�ŃJ�E���g
	MTU7.TCR.BIT.CCLR=0x01;		//p390 TGRA�̍��y�A�}�b�`/�C���v�b�g�L���v�`����TCNT�N���A
	MTU7.TMDR1.BIT.MD=0x02;		//p393 PWM���[�h�P
	MTU7.TIORH.BIT.IOA=0x01;	//p408 TGRA �����o�͂�Low�o�́@�R���y�A�}�b�`��Low�o��
	MTU7.TIORH.BIT.IOB=0x02;	//p402 TGRB �����o�͂�Low�o�́@�R���y�A�}�b�`��High�o��
	
	MTU7.TCNT=0;
	
	MTU7.TGRA=1023;
	MTU7.TGRB=900;
	
	MTU.TOERB.BIT.OE7A=1;		//p433 �}�X�^����MTIOC7A�r�b�g 1:MTU�o�͋���
	
	MTU.TSTRB.BIT.CST7=1;		//p426 MTU7.TCNT�̓J�E���g����
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
	MTU6.TGRB = 2999; //�����ݒ�
	
	MTU6.TGRC = 3000; //�����ݒ�
	MTU6.TGRD = 2999; //�����ݒ�
	
//	MTU.TOERB.BIT.OE6A = 1;	
	MTU.TOERB.BIT.OE6B = 1;
//	MTU.TOERB.BIT.OE6C = 1;
	MTU.TOERB.BIT.OE6D = 1;
	
	MTU.TSTRB.BIT.CST6 = 1; //�J�E���^����̊J�n
	
}

void pwm( int TGRB , int TGRD ){
	MTU6.TGRB = TGRB;
	MTU6.TGRD = TGRD;
}

void count_plus(void){
	count++;
}

int main(){
	
	int 	TGRB = 0,
		TGRD = 0;
	
	PORT9.DDR.BYTE = 0x3F;
	PORTB.DDR.BYTE = 0x00;
	PORTE.DDR.BYTE = 0x00;
	init_clock();
	init_cmt0();
	init_pwm();
	while(1){
		
		if(count >= 1000){
			pwm(TGRB,TGRD);

			TGRB = TGRB + 500;
			if(TGRB == 3500){
			TGRB = 0;}
			TGRD = 3000 - TGRB;

			count = 0;
		}
	}

}

//p94 MTU7 TGRA TGRB
//p91 MTU7 TGRC TGRD