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
	float	duty_led_green = 0.0,
			duty_led_orange = 0.0,
			duty_led_red = 0.0,
			duty_led_blue = 0.0,
			duty_motor = 90.0;
	SystemInit();
	RCC_Configuration();
	Init_GPIOs();
	init_systick(ADD_TIMER_COUNT);
	TM_TIMER_Init();
	TM_PWM(duty_led_green , duty_led_orange , duty_led_red , duty_led_blue, duty_motor);

	GPIO_SetBits(GPIOD,GPIO_Pin_0);
	GPIO_ResetBits(GPIOD,GPIO_Pin_1);

	while(1)
	{
		if(g_timer_count >= 1.0){
			g_timer_count = 0.0;

			duty_led_blue += 5.0;
			duty_led_green += 10.0;
			duty_led_orange += 20.0;
			duty_led_red += 30.0;
//			duty_motor += 10.0;
			TM_PWM(duty_led_green , duty_led_orange , duty_led_red , duty_led_blue,duty_motor);
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
	  uint16_t PrescalerValue = 0;
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
/*
    timer_tick_frequency = 84000000 / (0 + 1) = 84000000
*/
    TIM_BaseStruct.TIM_Prescaler = 0;
    /* Count up */
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
/*
    Set timer period when it have reset
    First you have to know max value for timer
    In our case it is 16bit = 65535
    To get your frequency for PWM, equation is simple

    PWM_frequency = timer_tick_frequency / (TIM_Period + 1)

    If you know your PWM frequency you want to have timer period set correct

    TIM_Period = timer_tick_frequency / PWM_frequency - 1

    In our case, for 10Khz PWM_frequency, set Period to

    TIM_Period = 84000000 / 10000 - 1 = 8399

    If you get TIM_Period larger than max timer value (in our case 65535),
    you have to choose larger prescaler and slow down timer tick frequency
*/
    TIM_BaseStruct.TIM_Period = PWM_PERIOD; /* 10kHz PWM */
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
    /* Initialize TIM4 */
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
    /* Start count on TIM4 */
    TIM_Cmd(TIM4, ENABLE);



    /* Compute the prescaler value */
    PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 28000000) - 1;

    /* Time base configuration */
    TIM_BaseStruct.TIM_Period = 1000;
    TIM_BaseStruct.TIM_Prescaler = 0;
    TIM_BaseStruct.TIM_ClockDivision = 0;
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);

    /* TIM3 enable counter */
    TIM_Cmd(TIM3, ENABLE);
}

void TM_PWM(float duty_led_green , float duty_led_orange , float duty_led_red , float duty_led_blue,float duty_motor) {
    TIM_OCInitTypeDef TIM_OCStruct;

    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    duty_led_green = Limit_ul(99,0,duty_led_green);
    TIM_OCStruct.TIM_Pulse = ((PWM_PERIOD + 1) * duty_led_green) / 100 - 1;
    TIM_OC1Init(TIM4, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    duty_led_orange = Limit_ul(99,0,duty_led_orange);
    TIM_OCStruct.TIM_Pulse = ((PWM_PERIOD + 1) * duty_led_orange) / 100 - 1;
    TIM_OC2Init(TIM4, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    duty_led_red = Limit_ul(99,0,duty_led_red);
    TIM_OCStruct.TIM_Pulse = ((PWM_PERIOD + 1) * duty_led_red) / 100 - 1;
    TIM_OC3Init(TIM4, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    duty_led_blue = Limit_ul(99,0,duty_led_blue);
    TIM_OCStruct.TIM_Pulse = ((PWM_PERIOD + 1) * duty_led_blue) / 100 - 1;
    TIM_OC4Init(TIM4, &TIM_OCStruct);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    duty_motor = Limit_ul(99,0,duty_motor);
    TIM_OCStruct.TIM_Pulse = ((PWM_PERIOD + 1) * duty_motor) / 100 - 1;
    TIM_OC1Init(TIM3, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
}
