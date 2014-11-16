#ifndef	INIT_H
#define	INIT_H

#include "stm32f4xx_conf.h"
#include "misc.h"

#define PWM_PERIOD		100-1
#define OUTPUT	0
#define INPUT	1
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)	//ADC3の変換結果レジスタのアドレス

void Init_GPIOs (void);
void RCC_Configuration(void);
void Init_Systick(float	time);
void Init_Timer(void);
void Init_Pwm(void);
void Init_USART(void);
void Init_io_port(uint16_t mode,GPIO_TypeDef *port,uint16_t pin);
void Init_Encoder(void);
void NVIC_Configuration(int irqn);
void ADC3_DMA_Config(uint16_t ADC3ConvertedValue[2]);

#endif
