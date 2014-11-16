//include StandardLibraly
#include <stdio.h>
#include <inttypes.h>

//include Repository
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_crc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_fsmc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

//include UserPrograms
#include "init_STM32F4.h"


//�}�N���̐錾
#define	PORT_PUSH_SW	GPIOA
#define	PIN_PUSH_SW		GPIO_Pin_0

#define	ADD_TIMER_COUNT	0.001

#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)	//ADC3�̕ϊ����ʃ��W�X�^�̃A�h���X

float	g_timer_count = 0.0;
int		g_encl_ou = 0;

uint16_t ADC3ConvertedValue[2];	//�ϊ����ʂ�DMA�]�������z��
int ADvoltage[2];

void All_Setup(void);
void myDelay(void);
void SysTick_Handler(void);
float Limit_ul(float max,float min,float figure);
int	main(void);
void change_revolution(int key_state);
void USART2_SendString(const char *str);

void All_Setup(void){

	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

	//CPU����N���b�N���g��������
	//�V�X�e���N���b�N:168Mhz
	SystemInit();

	Init_GPIOs();

	Init_USART();

	Init_Systick(ADD_TIMER_COUNT);

	Init_Timer();

	Init_Pwm();

	Init_Encoder();

	ADC3_DMA_Config(ADC3ConvertedValue);

	ADC_SoftwareStartConv(ADC3);
}

void TIM2_IRQHandler(void){
	if((TIM2->CR1) >> 4){
		g_encl_ou-=1;
	}else{
		g_encl_ou+=1;
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}

/******************************************************************************
*	�^�C�g�� �F �����݊֐�
*	  �֐��� �F SysTick_Handler
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/11/04
******************************************************************************/
void SysTick_Handler(void){
	g_timer_count = g_timer_count + ADD_TIMER_COUNT;
}

/******************************************************************************
*	�^�C�g�� �F ���C����
*	  �֐��� �F main
*	  �߂�l �F int�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/11/04
******************************************************************************/
int main(void){
	int		i = 0,
			up_down_flug = 1;
	char	string[20] = {0};

	All_Setup();
	TIM_SetCounter(TIM2,0);
	g_encl_ou = 0;
/*	while(1){
		if(g_timer_count >= 1.0){
			g_timer_count = 0.0;
			if(i>8){
				up_down_flug = 0;
			}else if(i==0){
				up_down_flug = 1;
			}
			if(up_down_flug==1){
				i++;
			}else{
				i--;
			}
	        TIM_SetCompare4(TIM4,PWM_PERIOD*i*5*0.01);
	        TIM_SetCompare3(TIM4,PWM_PERIOD*i*10*0.01);
	        TIM_SetCompare2(TIM4,PWM_PERIOD*i*20*0.01);
	        TIM_SetCompare1(TIM4,PWM_PERIOD*i*30*0.01);
	        TIM_SetCompare1(TIM3,PWM_PERIOD*i*10*0.01);

	        sprintf(string,"i = %d\n\r",i);
	        USART2_SendString(string);
//	        USART_SendData(USART2,'A');
		}
		change_revolution(GPIO_ReadInputDataBit(PORT_PUSH_SW, PIN_PUSH_SW));
	}
*/
	while(1){
		if(g_timer_count >= 0.005){
			g_timer_count = 0.0;
			//�ǂݏo�����ʂ�d���ɕϊ����Č��ʂ�z��ɓ����
			ADvoltage[0] = ADC3ConvertedValue[0] *3300/0xFFF;
			ADvoltage[1] = ADC3ConvertedValue[1] *3300/0xFFF;
			i = TIM_GetCounter(TIM2)+g_encl_ou*65535;
	        sprintf(string,"i = %d,%d\n\r",ADvoltage[0],ADvoltage[1]);
	        USART2_SendString(string);
		}
	}
}

/******************************************************************************
*	�^�C�g�� �F ���[�^�[��]�����ϊ�
*	  �֐��� �F change_revolution
*	  �߂�l �F void�^
*	   ����1 �F int�^ key_state
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/11/04
******************************************************************************/
void change_revolution(int key_state){
	static int	change_flug = 0,
				revolution_mode = 0;

	if( key_state >= 1 && change_flug == 1){
		revolution_mode = 1 - revolution_mode;
		change_flug = 0;
	}else	if( key_state == 0 ){
		change_flug = 1;
	}
	if(revolution_mode == 1){
		GPIO_SetBits(GPIOD,GPIO_Pin_1);
		GPIO_ResetBits(GPIOD,GPIO_Pin_0);
	}else{
		GPIO_SetBits(GPIOD,GPIO_Pin_0);
		GPIO_ResetBits(GPIOD,GPIO_Pin_1);
	}
}

/******************************************************************************
*	�^�C�g�� �F USART2�ł̕����񑗐M
*	  �֐��� �F USART2_SendString
*	  �߂�l �F void�^
*	   ����1 �F const char�^ *str
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/11/04
******************************************************************************/
void USART2_SendString(const char *str){
  const char *ptr = str;
  while (*ptr != '\0')
  {
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
    }
    USART_SendData(USART2, *ptr);
    ptr++;
  }
}

/******************************************************************************
*	�^�C�g�� �F �x���֐�
*	  �֐��� �F myDelay
*	  �߂�l �F void�^
*	    ���� �F �Ȃ�
*	  �쐬�� �F �Έ�
*	  �쐬�� �F 2014/11/04
******************************************************************************/
void myDelay(void){
	uint32_t ii;
	ii=0;
	//1,000,000�񃋁[�v�����
	while(ii<100000000){
		ii++;
	}
}

float Limit_ul(float max,float min,float figure){
	if(figure > max){
		return ( max );
	}else if(figure < min){
		return (min);
	}else{
		return (figure);
	}
}
