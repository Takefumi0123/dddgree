/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "typedefine.h"
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif




#ifdef __cplusplus
void abort(void)
{

}
#endif
#include "iodefine.h"
#include <math.h>
#include <stdio.h>

#define ON			1					//�X�C�b�`���
#define OFF			0
#define PCLK			48					//PCLK
#define BAUDREET		9600					//�{�[���[�g
#define PLUS_SWITCH	PORTB.PORT.BIT.B4		//�X�C�b�`�|�[�g
#define M_PI			3.141592653589793		//�~����
#define TCNT_RIGHT		MTU2.TCNT			//�E�G���R�[�_TCNT
#define TCNT_LEFT		MTU1.TCNT			//���G���R�[�_TCNT
#define DIAMETER_LEFT	22.0					//���G���R�[�_���a
#define DIAMETER_RIGHT	22.0					//�E�G���R�[�_���a
#define PULSE_LEFT		100					//���G���R�[�_�p���X
#define PULSE_RIGHT	100					//�E�G���R�[�_�p���X
#define FLANK			228					//�G���R�[�_�ԋ���
#define TURN_PGAIN		100					//��]�o��P�Q�C��
#define TURN_DGAIN		150					//��]�o��D�Q�C��
#define STRAIGHT_PGAIN	0.2					//���i�o��P�Q�C��
#define STRAIGHT_DGAIN	5.0					//���i�o��D�Q�C��
#define FRONT_LEFT		PORTB.DR.BIT.B3		//������]�o�͋���
#define BACK_LEFT		PORTB.DR.BIT.B1		//���t��]�o�͋���
#define FRONT_RIGHT	PORTB.DR.BIT.B2		//�E����]�o�͋���
#define BACK_RIGHT		PORTA.DR.BIT.B2		//�E�t��]�o�͋���
#define NOW_TARGET_X	COOD_XY[task].X		//�ڕW�����������W
#define NOW_TARGET_Y	COOD_XY[task].Y		//�ڕW�����������W
#define END_DEGREE		COOD_XY[task].end_degree//�ڕW�n�_������p�x
#define OLD_TARGET_X	COOD_XY[task-1].X		//�ڕW�����������W
#define OLD_TARGET_Y	COOD_XY[task-1].Y		//�ڕW�����������W
#define BRAKE			999999				//�u���[�L

int	g_count       =	0,	//1ms�J�E���g
	g_count2      =	0,	//1ms�J�E���g
	g_wave_count  = 0,	//1��s�J�E���g
	g_over_LEFT   = 0,	//���I�[�o�[�t���[�J�E���g
	g_under_LEFT  = 0,	//���A���_�[�t���[�J�E���g
	g_over_RIGHT  = 0,	//�E�I�[�o�[�t���[�J�E���g
	g_under_RIGHT = 0;	//�E�A���_�[�t���[�J�E���g

double	g_degree	= 0.00, //�@�̂̌��݊p�x�i���@�j
		g_rad	= 0.00;//�@�̂̌��݊p�x�irad�j
	
//�\���̐錾	
struct 	coordinates
{
	double	X;			//�����������W
	double	Y;			//�����������W
	double	end_degree;	//�ڕW������p�x
};
	
//PCK ICK�ݒ�	
void init_clock(void)
{
	SYSTEM.SCKCR.BIT.PCK = 1; //�~4 ICK�Ƃ͊֌W�Ȃ��ꏊ�̑���
	SYSTEM.SCKCR.BIT.ICK = 0; //�~8 �}�C�R�����̂��̂̑���	
}
  
//�^�C�}�[�ݒ�i1ms�j
void init_cmt0(void)
{
	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //���W���[���X�g�b�v��Ԃ̉���
	CMT.CMSTR0.BIT.STR0 = 0; //�J�E���g����̒�~
	
	CMT0.CMCR.BIT.CKS = 2;//�N���b�N�I�� 1/128
	CMT0.CMCOR = 375;   //CMCOR�̌��� 48mhz/128/1000
	
	CMT0.CMCNT = 0;//������
	CMT0.CMCR.BIT.CMIE  = 1;  //���荞�݋���
	CMT.CMSTR0.BIT.STR0 = 1;
	
	IEN(CMT0,CMI0) = 1; //���荞�ݗv�������W�X�^
	IPR(CMT0,CMI0) = 15; //�D��xMAX	
}
  
//�^�C�}�[�ݒ�i1��s�j
void init_cmt1(void)
{
	SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //���W���[���X�g�b�v��Ԃ̉���
	CMT.CMSTR0.BIT.STR1 = 0; //�J�E���g����̒�~
	
	CMT1.CMCR.BIT.CKS = 0;//�N���b�N�I�� 1/8
	CMT1.CMCOR = 6;   //CMCOR�̌��� 48mhz/8/1000000
	
	CMT1.CMCNT = 0;//������
	CMT1.CMCR.BIT.CMIE  = 1;  //���荞�݋���
	CMT.CMSTR0.BIT.STR1 = 1;
	
	IEN(CMT1,CMI1) = 1; //���荞�ݗv�������W�X�^
	IPR(CMT1,CMI1) = 15; //�D��xMAX	
}  
  
//PWM�����ݒ�
void init_pwm(void)
{
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
		
	MTU.TOERB.BIT.OE6B = 1;
	MTU.TOERB.BIT.OE6D = 1;
	
	MTU.TSTRB.BIT.CST6 = 1; //�J�E���^����̊J�n	
}

//�V���A���ʐM�����ݒ�
void init_serial(void)
{	
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
	
	SCI1.BRR = PCLK * 1000000 / ( 64 * 0.5 * BAUDREET ) - 1; //BRR���W�X�^�̐ݒ�l P822
	
	for( i = 0 ; i > 80000; i ++ );
	
	SCI1.SCR.BIT.RE = 1; //�V���A����M��������� P815
	SCI1.SCR.BIT.TE = 1; //�V���A�����M��������� P815	
}

//�G���R�[�_�[MTU2
void init_counter_MTU2(void)
{	
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

//�G���R�[�_�[MTU1
void init_counter_MTU1(void)
{	
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

//�����ݒ�
void init_dafault(void)
{	
	//���o�͂̏�����
	PORTA.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B1 = 1;
	PORTB.DDR.BIT.B2 = 1;
	PORTB.DDR.BIT.B3 = 1;
	PORTB.DDR.BIT.B4 = 0;
	PORT9.DDR.BYTE   = 0x3F;
	
	//��]�����̏�����
        FRONT_LEFT  = 0;
        BACK_LEFT   = 0;
        FRONT_RIGHT = 0;
        BACK_RIGHT  = 0;	
}

//PWM	�����́�
void pwm(double TGRD,double TGRB)
{	
	MTU6.TGRB = 3000 * ( 100 - TGRB ) / 100;
	MTU6.TGRD = 3000 * ( 100 - TGRD ) / 100;	
}

//�͈͓��o��
double Limit_ul(double figure,double max,double min)
{
	if(figure > max){
		return ( max );
	}else if(figure < min){
		return ( min );
	}else{
		return ( figure );
	}
}

//���[�^�֐�
void Move(double L_output,double R_output)
{		
	L_output = Limit_ul(L_output,100,-100);
	R_output = Limit_ul(R_output,100,-100);
	
	if (L_output > 0){
	        FRONT_LEFT = 1;
	        BACK_LEFT = 0;
	}else
	if (L_output < 0){
	        FRONT_LEFT = 0;
	        BACK_LEFT = 1;
		L_output = -1 * L_output;
	}else 
	if( L_output == 0 ){
		FRONT_LEFT = 1;
		BACK_LEFT  = 1;
	}
	
        if (R_output > 0){
	        FRONT_RIGHT = 1;
	        BACK_RIGHT = 0;
        }else
	if (R_output < 0){
	        FRONT_RIGHT = 0;
	        BACK_RIGHT = 1;
		R_output = -1 * R_output;
	}else
	if( R_output == 0){
		FRONT_RIGHT = 1;
		BACK_RIGHT  = 1;
	}  
	    
	pwm( L_output , R_output );
}	

//PD���i����
double straight_PD(double difference )
{	
	double output = 0.00;
		
	static double	old_difference = 0.00,
			old_output = 0.00;
	
	output = old_output + ( STRAIGHT_PGAIN * difference ) + ( STRAIGHT_DGAIN * ( difference - old_difference ));
	
	output = Limit_ul( output,100,-100 );
	
	old_difference = difference;
	old_output = output;
	
	return output;
}	

//PD��]����
double turn_PD( double difference )
{	
	double output = 0.00;
		
	static double old_difference = 0.00;
	
	output = ( TURN_PGAIN * difference ) + ( TURN_DGAIN * ( difference - old_difference ));
	
	output = Limit_ul( output,80,-80 );
	
	old_difference = difference;
	
	return output;
}	
	
//�V���A���ʐM	
void transmission(char *string)
{
	int i = 0;
	
	while( string[i] != '\0' ){	
		if(SCI1.SSR.BIT.TDRE == 1){
			SCI1.SSR.BIT.TDRE = 0;
			SCI1.TDR = string[i];
			i ++;
		}
	}
}

//�J�E���g�i1ms�j
void count_plus(void)
{	
	g_count ++;
	g_count2 ++;
}

//�J�E���g�i1��s�j
void sonic_count_plus(void)
{	
	g_wave_count ++;	
}

//�I�[�o�[�t���[�������A�ϐ�g_over��1����
void over_flow_LEFT(void)
{
	g_over_LEFT++;
}

//�A���_�[�t���[�������A�ϐ�g_under�ɂP����
void under_flow_LEFT(void)
{
	g_under_LEFT++;
}

//�I�[�o�[�t���[�������A�ϐ�g_over��1����
void over_flow_RIGHT(void)
{
	g_over_RIGHT++;
}

//�A���_�[�t���[�������A�ϐ�g_under�ɂP����
void under_flow_RIGHT(void)
{
	g_under_RIGHT++;
}

//�`���^�����O�i�_�E���G�b�W�j
//�X�C�b�`������������i����������j�Ԑ؂�ւ��̂� swt_keep
//�X�C�b�`�������i�����j�Ɛ؂�ւ��̂� swt_change
int chata_down (int swt1, int i)
{
	static int status[3] = { 0, 0, 0},
		   swt_change[3] = { 0, 0, 0},
		   swt_keep[3] = { 0, 0, 0};
	
	if(swt1 == 1){
		status[i]++;
	}else if(swt1 == 0){
		status[i] = 0;
	}
	
	if(swt_keep[i] == 0 && status[i] >= 5){
		swt_change[i] = 1 - swt_change[i];
		swt_keep[i] = 1;
		
	}
	if(status[i] == 0){
		swt_keep[i] = 0;
	}
	
	return swt_change[i]; //�p�����
//	return swt_keep[i]; //��ԕω�	
}

int main(void)
{	
	int	swt1		= 1,			//�X�C�b�`���
		swt1_x 	= 1,
		status	= ON,
		task		= 1,			//�^�X�N
		end_to_start = 0,
		task_change = 0;
		
	double	distance_LEFT		= 0.00,	//���֐i�s����
			distance_RIGHT		= 0.00,	//�E�֐i�s����
			add_distance_LEFT	= 0.00,	//���֐i�s�����΍�
			add_distance_RIGHT	= 0.00,	//�E�֐i�s�����΍�
			old_distance_LEFT	= 0.00,	//�O��̍��֐i�s����
			old_distance_RIGHT	= 0.00,	//�O��̉E�֐i�s����
			add_degree		= 0.00,	//�p�x���v�i���j
			target_degree		= 0.00, //�ڕW���W�܂ł̊p�x�i���j
			difference_degree	= 0.00,	//�ڕW�܂ł̊p�x�Ƃ̕΍��i���j
			another_difference_degree = 0.00, //�ڕW�܂ł̊p�x�Ƃ̕΍��@�������������̌v�Z�ɗp����
			add_rad			= 0.00,	//�p�x���v�irad�j
			x				= 0.00,	//�����������W
			y				= 0.00,	//�����������W
			enc_RIGHT		= 0.00,	//�E�G���R�[�_�l
			enc_LEFT			= 0.00, //���G���R�[�_�l
			difference_distance	= 0.00,	//�ڕW�܂ł̋���
			straight_output		= 0.00,	//���i�o��
			turn_output		= 0.00,	//��]�o��
			x_difference_distance	= 0.00,	//�ڕW�܂ł̐�������
			y_difference_distance	= 0.00;	//�ڕW�܂ł̐�������
		
	char string[50] = { 0 };		//������錾
	
	//�\���̂̏�����
	struct coordinates COOD_XY[10] = {
		{ 0, 0, 0},
		{ 0, 0 , 0},
		{ 500, 0 , 30},
		{ 1515, 865, 0},
		{ 2015, 865 , 0},
		{ BRAKE, BRAKE , BRAKE}
	};
	
	init_clock();
	init_cmt0();
	init_cmt1();
	init_serial();
	init_counter_MTU1();
	init_counter_MTU2();
	init_pwm();
	init_dafault();
	
	while(1){
		 // 5ms���Ɉȉ��̓�����s��
		if( g_count >= 5 ){
			g_count = 0;
			//�X�C�b�`��OFF�̂Ƃ��ȉ��̓���������Ȃ�
			if( status == OFF ){
				swt1 = chata_down(( PLUS_SWITCH ), 0); //�X�C�b�`�̏�Ԃ̊m�F
				
				if( swt1 == 0 && swt1_x == 0){
					status = ON;
					swt1_x = 1;
					TCNT_RIGHT = 0;
					TCNT_LEFT = 0;
					x = 0;
					y = 0;
				}else
				if( swt1 == 1){
					swt1_x = 0;
				}
			}else
			//�X�C�b�`��ON�̂Ƃ��ȉ��̓���������Ȃ�
			if( status == ON ){
				enc_LEFT  =  TCNT_LEFT  + ( 65536 * g_over_LEFT)  + ( ( -65536 ) * g_under_LEFT  ); //���G���R�[�_�[�̒l
				enc_RIGHT =  TCNT_RIGHT + ( 65536 * g_over_RIGHT) + ( ( -65536 ) * g_under_RIGHT ); //�E�G���R�[�_�[�̒l
				distance_LEFT 	= -1 * ( enc_LEFT * M_PI * DIAMETER_LEFT)  / (PULSE_LEFT  * 4 ); //���֐i�smm
				distance_RIGHT 	= ( enc_RIGHT * M_PI * DIAMETER_RIGHT) / (PULSE_RIGHT * 4 ); //�E�֐i�smm
							
				add_distance_LEFT  = distance_LEFT  - old_distance_LEFT; //���֕΍�			
				add_distance_RIGHT = distance_RIGHT - old_distance_RIGHT; //�E�֕΍�
							
				g_rad = ( distance_RIGHT - distance_LEFT ) / FLANK; //�p�x�iradian�j
				add_rad +=  ( add_distance_RIGHT - add_distance_LEFT ) / FLANK; //�݌v�p�x�iradian�j
				g_degree = g_rad * 180 / M_PI; //�p�x�i���j
				add_degree = add_degree + 180 * ( add_distance_RIGHT - add_distance_LEFT ) / ( FLANK * M_PI ); //�݌v�p�x�i���j
				
				x += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * cos( add_rad ) ; //�����W���݈ʒu�i�c�j
				y += (( add_distance_LEFT + add_distance_RIGHT ) / 2.00 ) * sin( add_rad ) ; //y���W���݈ʒu�i���j
				
				//atan2�G���[���
				if( NOW_TARGET_X == x && NOW_TARGET_Y == y ){
					target_degree = 0;
				}else{								
					target_degree = atan2( NOW_TARGET_Y - y , NOW_TARGET_X - x ) * 180 / M_PI; //���ݒl���猩���ړI�n�̊p�x
				}
				
				difference_distance = sqrt( ( NOW_TARGET_X - x ) * ( NOW_TARGET_X - x ) + ( NOW_TARGET_Y - y ) * ( NOW_TARGET_Y - y )); //���ݒl���猩���ړI�n�܂ł̋���
				
				// -180 < degree < 180
				while ( g_degree > 180 ){
					g_degree = -360 + g_degree;
				}
				while ( g_degree < -180 ){
					g_degree = g_degree + 360;
				}
								
				difference_degree = target_degree - g_degree; //�p�x�΍��i���j
				
				// -180 < difference_degree < 180
				if ( difference_degree > 180 ){
					difference_degree = -360 + difference_degree;
				}else
				if ( difference_degree < -180 ){
					difference_degree = difference_degree + 360;
				}
				
				another_difference_degree = difference_degree; //�������������v�Z�ɗp����p�x
				
				// -90 < another_difference_degree < 90
				if ( difference_degree > 90 ){
					another_difference_degree = 180 - difference_degree;
					x_difference_distance = -1 * difference_distance * cos( another_difference_degree / 180 * M_PI );//�ړI�n�܂ł̐�������
				}else
				if ( difference_degree < -90 ){
					another_difference_degree = -180 - difference_degree ;
					x_difference_distance = -1 * difference_distance * cos( another_difference_degree / 180 * M_PI );//�ړI�n�܂ł̐�������					
				}else{
					x_difference_distance = difference_distance * cos( another_difference_degree / 180 * M_PI );//�ړI�n�܂ł̐�������
				}
				//1�O�̒l�Ɋi�[
				old_distance_LEFT  = distance_LEFT;
				old_distance_RIGHT = distance_RIGHT;
				
				//1�b���ɃV���A���ʐM���s��
				if(g_count2 >= 1000 ){
					g_count2 = 0;
//					sprintf(string,"%.3lf   %.3lf   %.3lf   %.3lf   %d\n\r",x,y,x_difference_distance,y_difference_distance,task);
//					transmission( string );
				}
				
				//�ړI�n��������@��������������
				if( x_difference_distance <= 2 && end_to_start == 0){
					end_to_start = 1; //�ڕW�n�_�����t���O
//					sprintf( string,"%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%d\n\r",x,y,x_difference_distance,y_difference_distance,g_degree,task);
//					transmission( string );
				}
				//�ڕW�n�_�ɒ����Ă���͈͓��̊p�x�ɓ���ƈȉ��ɓ���
				if( x_difference_distance <= 1 && x_difference_distance >= -1 && ( END_DEGREE - g_degree ) <= 1 && ( END_DEGREE - g_degree) >= -1 && end_to_start == 1){
					task_change ++;
				}
				//�ڕW�n�_�ɒ����Ă���͈͓��̊p�x��5�����ƈȉ��ɓ���
				if( task_change >= 500 ){
					task_change = 0;
					end_to_start = 0;
					task ++;
//					sprintf( string,"%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%d\n\r",x,y,x_difference_distance,y_difference_distance,g_degree,task);
//					transmission( string );
				}
				//�ڕW�n�_�ɒ����Ă��玟�̖ڕW�n�_�Ɍ������܂ł̐���
				if( end_to_start == 1){
					// -180 < END_DEGREE - g_degree < 180
					if( END_DEGREE - g_degree > 180){
						turn_output = turn_PD( END_DEGREE - g_degree - 360);
					}else
					if( END_DEGREE - g_degree < -180){
						turn_output = turn_PD( END_DEGREE - g_degree + 360);
					}else{
						turn_output = turn_PD( END_DEGREE - g_degree);
					}
					straight_output = straight_PD( x_difference_distance ); //���iPD����
					Move( straight_output - turn_output ,straight_output + turn_output ); //���[�^�֐�
				}else{
					//�ʏ퐧��
					turn_output = turn_PD ( difference_degree ); //��]PD����
					straight_output = straight_PD( x_difference_distance ); //���iPD����
					Move( straight_output - turn_output ,straight_output + turn_output ); //���[�^�֐�
				}
				//�u���[�L
				if( NOW_TARGET_X == BRAKE && NOW_TARGET_Y == BRAKE ){
					FRONT_LEFT = 1;
					BACK_LEFT = 1;
					FRONT_RIGHT = 1;
					BACK_RIGHT = 1;
					pwm( 99, 99);
				}
			}
		}
	}
}
