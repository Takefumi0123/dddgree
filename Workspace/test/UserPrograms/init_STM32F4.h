#ifndef	INIT_H
#define	INIT_H

#include "stm32f4xx_conf.h"
#define PWM_PERIOD		100
#define OUTPUT	0
#define INPUT	1

void Init_GPIOs (void);
void RCC_Configuration(void);
void Init_Systick(float	time);
void Init_Timer(void);
void Init_Pwm(void);
void Init_USART(void);
void Init_io_port(uint16_t mode,GPIO_TypeDef *port,uint16_t pin);

#endif
