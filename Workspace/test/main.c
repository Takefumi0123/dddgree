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

#define	ADD_TIMER_COUNT	0.005

float	g_timer_count = 0;

void myDelay(void);
void SysTick_Handler(void);
void init_systick(float	time);
int	main(void);

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
	while(ii<1000000){
		ii++;
	}
}

int main(void)
{
	Init_GPIOs();
	init_systick(ADD_TIMER_COUNT);
//ボタンを押している間LEDを点滅する無限ループ
	while(1)
	{
		if(g_timer_count >= 1){
			g_timer_count = 0;
			GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
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
