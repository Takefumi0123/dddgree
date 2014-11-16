#include "init_STM32F4.h"

//-----------------------------------------------------------------------------------------
// STM32F4 Discoveryの場合、基板内部で接続している信号があるため、コネクタ外部で使用できない信号が多数あります。
//-----------------------------------------------------------------------------------------
// To initialize the I/O ports
//-----------------------------------------------------------------------------------------
void  Init_GPIOs (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//Configuration Clocks
	RCC_Configuration();
	//--------------------------------------------------------------------------------------------
	// 使用しないピンをPull down入力に指定する。
	//--------------------------------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	//-----------------------------------------------------------------------------------
	// GPIO PORT A
	//-----------------------------------------------------------------------------------
	// PA0 : User SW

	// PA1 : PRST(Reset Network)

	// PA2 : UART2_TxD
	// PA3 : UART2_RxD

	// PA4 : D/A 0
	// PA5 : D/A 1

	// PA6 : PWM0
	// PA7 : PWM1

	// PA8 : NC

	// PA9 : UART1_TxD
	// PA10 : UART1_RxD

	// PA13 : SWDIO
	// PA14 : SWCLK

	// PA15 : SCS(SPI1_NSS)
	//-----------------------------------------------------------------------------------
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
								| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
								| GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11
								| GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//-----------------------------------------------------------------------------------
	// GPIO PORT B
	//-----------------------------------------------------------------------------------
	// PB0 : PWM2
	// PB1 : PWM3

	// PB2 : NC

	// PB3 : SCLK(SPI1_SCK)
	// PB4 : NC(SPI1_MISO)
	// PB5 : SI(SPI1_MOSI)

	// PB6 : I2C1_SCL
	// PB7 : I2C1_SDA

	// PB8 : NC
	// PB9 : NC

	// PB10 : UART3_TxD
	// PB11 : UART3_RxD

	// PB12 : SPI2_NSS
	// PB13 : SPI2_SCK
	// PB14 : SPI2_MISO
	// PB15 : SPI2_MOSI
	//-----------------------------------------------------------------------------------
	//-----------------------------------------------------------------------------------
	// GPIO PORT C
	//-----------------------------------------------------------------------------------
	// PC0 : ADC_IN10
	// PC1 : ADC_IN11
	// PC2 : ADC_IN12
	// PC3 : ADC_IN13
	// PC4 : ADC_IN14
	// PC5 : ADC_IN15

	// PC6 : CPU Status LED

	// PC7 : Detect SDC

	// PC8 : SDIO_D0
	// PC9 : SDIO_D1
	// PC10 : SDIO_D2
	// PC11 : SDIO_D3
	// PC12 : SDIO_CK

	// PC13 : NC

	// PC14 : OSC32_IN
	// PC15 : OSC32_OUT
	//-----------------------------------------------------------------------------------

	//PWMモード設定
	//PC6 PC7
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//-----------------------------------------------------------------------------------
	// GPIO PORT D
	//-----------------------------------------------------------------------------------
	// PD0 : Port0_bit0
	// PD1 : Port0_bit1

	// PD2 : SDIO_CMD

	// PD3 : Port0_bit3
	// PD4 : Port0_bit4
	// PD5 : Port0_bit5
	// PD6 : Port0_bit6
	// PD7 : Port0_bit7

	// PD8 : Port1_bit0
	// PD9 : Port1_bit1
	// PD10 : Port1_bit2
	// PD11 : Port1_bit3
	// PD12 : Port1_bit4
	// PD13 : Port1_bit5
	// PD14 : Port1_bit6
	// PD15 : Port1_bit7
	//-----------------------------------------------------------------------------------
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
								| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
								| GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//LEDの設定
	//設定するピンを指定する（４つのLED）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//-----------------------------------------------------------------------------------
	// GPIO PORT E
	//-----------------------------------------------------------------------------------
	// PE0 : LCD_DB0
	// PE1 : LCD_DB1
	// PE2 : LCD_DB2
	// PE3 : LCD_DB3
	// PE4 : LCD_DB4
	// PE5 : LCD_DB5
	// PE6 : LCD_DB6
	// PE7 : LCD_DB7

	// PE8 : LCD_RS
	// PE9 : LCD_RW
	// PE10 : LCD_E

	// PE11 : LCD_CS1
	// PE12 : LCD_CS2

	// PE13 : LCD_RST

	// PE14 : Port0_bit2

	// PE15 : DISP(LCD I/F2)
	//-----------------------------------------------------------------------------------

	//GPIOAのPIN2を出力に設定
	//PA2	シリアル送信
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    //PA0	PA1をエンコーダポートに設定
	GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	//PINをオルタネィテブファンクションに割り当て
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
}

// Configures the different system clocks.
void RCC_Configuration(void){
	/* Enable the GPIOs Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB
			| RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD
			| RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOH, ENABLE);

	/* Enable PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_USART2, ENABLE);

	/* Enable SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG , ENABLE);
}

void Init_Systick(float	time){
	SystemCoreClockUpdate();

	if(SysTick_Config(SystemCoreClock*time)){

	}
}

void Init_Timer(void) {
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_BaseStruct.TIM_Prescaler = 20;
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseStruct.TIM_Period = PWM_PERIOD; // 10kHz PWM
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);

    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

void Init_Pwm(void) {
    TIM_OCInitTypeDef TIM_OCStruct;

    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OCStruct.TIM_Pulse = PWM_PERIOD * 0.0;
    TIM_OC1Init(TIM4, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM4, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM4, &TIM_OCStruct);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM4, &TIM_OCStruct);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC1Init(TIM3, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void Init_USART(void){
	//USART初期化用構造体を作る
	USART_InitTypeDef USART_InitStructure;

	//USART2を38400bps,8bit,ストップビット1,パリティなし,フロー制御なし,送受信有効に設定
	USART_InitStructure.USART_BaudRate = 38400;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	//USART2を有効化
	USART_Cmd(USART2, ENABLE);
}

void Init_Encoder(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_PrescalerConfig(TIM2,0,TIM_PSCReloadMode_Update);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);

	TIM_Cmd(TIM2,ENABLE);

	NVIC_Configuration(TIM2_IRQn);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
}

//割り込み関数の許可
void NVIC_Configuration(int irqn){
	NVIC_InitTypeDef	NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = irqn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void ADC3_DMA_Config(uint16_t ADC3ConvertedValue[2]){
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  //必要なペリフェラルにクロック供給開始
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

  //DMAの設定
  //送り元のアドレスを固定し、送り先をADC3ConvertedValueにしてアドレスをインクリメント、送るデータ数を2つに
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 2;	//２チャンネルの変換結果を送るからデータ数は２（チャネルを増やしたらここを変える）
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//送り先アドレスをインクリメント
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  //GPIOCの指定の入力をアナログに設定
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;	//PC1とPC2を設定
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  //ADC3の基本設定
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  //ADC3の変換モード設定。入力2本をスキャン変換モードで逐次変換
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//スキャン変換モードに
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 2;	//入力を２本（チャネルを増やしたらここを変える）
  ADC_Init(ADC3, &ADC_InitStructure);

  //ADC3のアナログ入力を定義する
  //ADC_Channel_11がPA1,ADC_Channel_12がPA2。マニュアル(UM1472)のピン配表を見るとわかる
  //"ADC123_IN11"という表記は、ADC1,ADC2,ADC3で使える入力でchannel_11に割り当てられる、ということ
  //使うチャネルを増やすには、ここに追記する。チャネル指定の次のパラメータは変換シーケンスの順序
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_3Cycles);

  //変換結果がDMA転送されるごとに、ADCは次の変換を開始するように設定
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

  //ADC3のDMAを使えるようにする
  ADC_DMACmd(ADC3, ENABLE);

  //ADC3を使えるようにする
  ADC_Cmd(ADC3, ENABLE);
}
