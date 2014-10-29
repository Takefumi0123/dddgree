#include "init_GPIOs.h"


//-----------------------------------------------------------------------------------------
// STM32F4 Discovery�̏ꍇ�A������Őڑ����Ă���M�������邽�߁A�R�l�N�^�O���Ŏg�p�ł��Ȃ��M������������܂��B
//-----------------------------------------------------------------------------------------
// To initialize the I/O ports
//-----------------------------------------------------------------------------------------
void  Init_GPIOs (void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//Configuration Clocks
	RCC_Configuration();
	//--------------------------------------------------------------------------------------------
	// �g�p���Ȃ��s����Pull down���͂Ɏw�肷��B
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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//�������p�\���̂Ƀp�����[�^���Z�b�g���Ă������߁A�������񏉊��l�ɖ߂�
	GPIO_StructInit(&GPIO_InitStructure);
	//�ݒ肷��s�����w�肷��i�X�C�b�`�̃s���E�A�N�e�B�u�n�C�j
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
								| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
								| GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11
								| GPIO_Pin_12 | GPIO_Pin_15;
	//�w�肵���s������͂Ɏw�肷��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//�v���A�b�v���g�p���Ȃ�
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO�̃X�s�[�h��100MHz�i�ō����j�ɃZ�b�g����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	//PORTA�ݒ���͏I���B�����GPIOA��ݒ肷��
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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
								| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
								| GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11
								| GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

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
	//PORTC�ɃN���b�N�̋������J�n
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	//�������p�\���̂Ƀp�����[�^���Z�b�g���Ă������߁A�������񏉊��l�ɖ߂�
	GPIO_StructInit(&GPIO_InitStructure);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	//�������p�\���̂Ƀp�����[�^���Z�b�g���Ă������߁A�������񏉊��l�ɖ߂�
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	//PORTA�ݒ���͏I���B�����GPIOA��ݒ肷��
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
	//PORTD�ɃN���b�N�̋������J�n
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//�������p�\���̂Ƀp�����[�^���Z�b�g���Ă������߁A�������񏉊��l�ɖ߂�
	GPIO_StructInit(&GPIO_InitStructure);
	//�ݒ肷��s�����w�肷��i�S��LED�j
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
								| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
								| GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	//�w�肵���s�����o�͂Ɏw�肷��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//�o�̓|�[�g�̃^�C�v���v�b�V���v���Ɏw�肷��
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//�v���A�b�v���g�p���Ȃ�
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO�̃X�s�[�h��100MHz�i�ō����j�ɃZ�b�g����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	//PORTD�ݒ���͏I���B�����GPIOD��ݒ肷��
	GPIO_Init(GPIOD, &GPIO_InitStructure);

//LED�̐ݒ�
	//�ݒ肷��s�����w�肷��i�S��LED�j
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	//�w�肵���s�����o�͂Ɏw�肷��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	//PORTD�ݒ���͏I���B�����GPIOD��ݒ肷��
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
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
								| GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
								| GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11
								| GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//--------------------------------------------------------------------------------------------


	//----------------------------------------------------
	// Initialize for input User Switch
	//----------------------------------------------------
	// Configure User Button pin as input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);


	//---------------------------------------------------------------------
	// USB FS OFF : �d��ON���A��UOFF
	//---------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//---------------------------------------------------------------------


	//------------------------------------------------------------------------------------------
	// DiscoveryKit
	//------------------------------------------------------------------------------------------

	//------------------------------------------------------------------------------------------
	// DiscoveryKit�̂Ƃ��R�l�N�^�ɐڑ�����Ă���O���M�����g�p���邽�ߓ����̃f�o�C�X�����Z�b�g��Ԃɂ���B
	//------------------------------------------------------------------------------------------
	// Reset Audio IC
	//------------------------------------------------------------------------------------------
	// PD4 : I/O_bit4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	// Reset Audio IC
	GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);
	//------------------------------------------------------------------------------------------



	//-----------------------------------------------------------
	// DiscoveryKit : OTG_FS_PowerSwitch
	//-----------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//----------------------------------------
	// OTG_FS_PowerSwitch OFF
	//----------------------------------------
	GPIO_SetBits(GPIOC, GPIO_Pin_0);
	//----------------------------------------


	//-----------------------------------------------------------
	// PD5 :Detect OTG_FS_OverCurrent
	//-----------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


  GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);
}

// Configures the different system clocks.
void RCC_Configuration(void)
{
	/* Enable the GPIOs Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB
			| RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD
			| RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOH, ENABLE);

	/* Enable PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Enable SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG , ENABLE);
}
