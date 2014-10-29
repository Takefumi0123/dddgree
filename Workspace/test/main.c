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

#include "init_GPIOs.h"

#define	PORT_GREEN_LED	GPIOD
#define	PORT_ORANGE_LED	GPIOD
#define	PORT_RED_LED	GPIOD
#define	PORT_BLUE_LED	GPIOD
#define	PIN_GREEN_LED	GPIO_Pin_12
#define PIN_ORANGE_LED	GPIO_Pin_13
#define	PIN_RED_LED		GPIO_Pin_14
#define	PIN_BLUE_LED	GPIO_Pin_15
#define	PORT_PUSH_SW	GPIOA
#define	PIN_PUSH_SW		GPIO_Pin_0
#define PWM_PERIOD		8399

#define	ADD_TIMER_COUNT	0.005

float	g_timer_count = 0;

void myDelay(void);
void SysTick_Handler(void);
void init_systick(float	time);
float Limit_ul(float max,float min,float figure);
int	main(void);
void TM_TIMER_Init();
void TM_PWM(float duty_led_green , float duty_led_orange , float duty_led_red , float duty_led_blue , float duty_motor);
void TM_PWM_INIT();

void SysTick_Handler(void){
	g_timer_count = g_timer_count + ADD_TIMER_COUNT;
}

void init_systick(float	time){
	SystemCoreClockUpdate();

	if(SysTick_Config(SystemCoreClock*time)){

	}
}

void myDelay(){
	uint32_t ii;
	ii=0;
	//1,000,000回ループを回る
	while(ii<100000000){
		ii++;
	}
}

float Limit_ul(float max,float min,float figure)
{
	if(figure > max){
		return ( max );
	}else if(figure < min){
		return ( min );
	}else{
		return (figure);
	}
}

int main(void)
{
	int		i = 0,
			down_flug = 0,
			up_flug = 1;

	SystemInit();
	RCC_Configuration();
	Init_GPIOs();
	init_systick(ADD_TIMER_COUNT);
	TM_TIMER_Init();

	GPIO_SetBits(GPIOD,GPIO_Pin_1);
	GPIO_ResetBits(GPIOD,GPIO_Pin_0);
	TM_PWM_INIT();
	while(1)
	{
		if(g_timer_count >= 1.0){
			g_timer_count = 0.0;
			if(i>8){
				down_flug = 1;
				up_flug = 0;
			}else if(i==0){
				down_flug = 0;
				up_flug = 1;
			}
			if(up_flug==1)
				i++;
			if(down_flug==1)
				i--;
//	        TIM_SetCompare4(TIM4,PWM_PERIOD*i*5*0.01);
//	        TIM_SetCompare3(TIM4,PWM_PERIOD*i*10*0.01);
//	        TIM_SetCompare2(TIM4,PWM_PERIOD*i*20*0.01);
//	        TIM_SetCompare1(TIM4,PWM_PERIOD*i*30*0.01);
	        TIM_SetCompare1(TIM3,PWM_PERIOD*i*10*0.01);
		}

		if(GPIO_ReadInputDataBit(PORT_PUSH_SW, PIN_PUSH_SW)){
			//ボタンが押されて１になっていたら以下を実行
			GPIO_SetBits(PORT_GREEN_LED,PIN_GREEN_LED); //LED点灯
			myDelay();
			GPIO_ResetBits(PORT_GREEN_LED,PIN_GREEN_LED); //LED消灯
			myDelay();
			GPIO_SetBits(PORT_ORANGE_LED,PIN_ORANGE_LED); //LED点灯
			myDelay();
			GPIO_ResetBits(PORT_ORANGE_LED,PIN_ORANGE_LED); //LED消灯
			myDelay();
			GPIO_SetBits(PORT_RED_LED,PIN_RED_LED); //LED点灯
			myDelay();
			GPIO_ResetBits(PORT_RED_LED,PIN_RED_LED); //LED消灯
			myDelay();
			GPIO_SetBits(PORT_BLUE_LED,PIN_BLUE_LED); //LED点灯
			myDelay();
			GPIO_ResetBits(PORT_BLUE_LED,PIN_BLUE_LED); //LED消灯
			myDelay();
		}
	}
}

void TM_TIMER_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_BaseStruct.TIM_Prescaler = 0;

    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_BaseStruct.TIM_Period = PWM_PERIOD; // 10kHz PWM
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

void TM_PWM_INIT() {
    TIM_OCInitTypeDef TIM_OCStruct;

    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    TIM_OCStruct.TIM_Pulse = PWM_PERIOD * 0.0;
    TIM_OC1Init(TIM4, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM4, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM4, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM4, &TIM_OCStruct);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OC1Init(TIM3, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
}
