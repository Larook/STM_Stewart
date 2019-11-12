/*
 * enable_things.c
 *
 *  Created on: 13.02.2019
 *      Author: PiK
 */
#include "stm32f10x.h"
#include "main.h"

//#define ADC_CHANNELS	5 // // X Y Z  panelX panelY

//uint16_t adc_value[ADC_CHANNELS]; //tablica z wynikami adc z DMA

void init_timer_touch() {

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//	TIM_TimeBaseInitTypeDef tim_read;
//	NVIC_InitTypeDef nvic_read;

	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 64000 - 1;
	tim.TIM_Period = 100 - 1; // bylo na 10 i smigalo jak nie bylo ruchu
	TIM_TimeBaseInit(TIM2, &tim);

	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_5;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &gpio); // dioda

	// Przygotuj piny do oblsugi odczytywanie wspolrzednych panelu dotykowego w przerwaniu
	// przygotuj do odczytania X
	GPIO_SetBits(GPIOA, GPIO_Pin_5);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &gpio);
	GPIO_SetBits(GPIOC, GPIO_Pin_0); // blue Vcc
	GPIO_ResetBits(GPIOC, GPIO_Pin_2); // white GND

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &gpio);  // red float

	gpio.GPIO_Pin = GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio); // black read
	//X_TouchPanel = getX_touchPanel();  // black read

	//wlacz przerwania timera
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void init_ADC_DMA() {
//	GPIO_InitTypeDef gpio;
	DMA_InitTypeDef dma;
	ADC_InitTypeDef adc;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* INICJALIZACJA ADC z DMA */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_StructInit(&gpio); // Pin 10 zasila Joystick/Potencjometr
	gpio.GPIO_Pin = GPIO_Pin_4;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &gpio);
	GPIO_SetBits(GPIOB, 4); // wl zasilania ADC

	GPIO_StructInit(&gpio); // Pin 3 GND Joystick/Potencjometr
	gpio.GPIO_Pin = GPIO_Pin_5;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &gpio);
	GPIO_ResetBits(GPIOB, 5);

	GPIO_StructInit(&gpio); // PC10 wejscie przycisku z Joysticka
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &gpio);

	GPIO_StructInit(&gpio); // Pin 12 swieci DIODA
	gpio.GPIO_Pin = GPIO_Pin_12;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &gpio);
	GPIO_SetBits(GPIOA, 12); // wl zasilania ADC

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_2;  // Pin 2 zasila ADC Potencjometru
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &gpio);
	GPIO_SetBits(GPIOA, 2); // wl zasilania ADC

	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4; // PA0, PA1, PA4 wejscia ADC1
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &gpio);

	//dodanie pinow do odczytu ADC z touchpanel
	gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0; // PC1, PC2 wejscia ADC1 touchPanel
	gpio.GPIO_Mode = GPIO_Mode_AIN;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &gpio);

	DMA_StructInit(&dma);
	dma.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	dma.DMA_MemoryBaseAddr = (uint32_t) adc_value;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	dma.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma.DMA_BufferSize = ADC_CHANNELS;
	dma.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(DMA1_Channel1, &dma);
	DMA_Cmd(DMA1_Channel1, ENABLE);

	ADC_StructInit(&adc);
	adc.ADC_ScanConvMode = ENABLE; // enable dla wielu kanalow
	adc.ADC_ContinuousConvMode = ENABLE;
	adc.ADC_NbrOfChannel = ADC_CHANNELS;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_Init(ADC1, &adc);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_239Cycles5);

	// touchPanel - jak to dobrac??? Patrzylem na koniec z CubeMX
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 4,
	ADC_SampleTime_239Cycles5); //PC1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 5,
	ADC_SampleTime_239Cycles5); //PC2-12  PC0-10

	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);

	while (ADC_GetResetCalibrationStatus(ADC1))
		;

	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1))
		;

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

void init_ButtonInterrupt() {
	//Przycisk PC10 odpowiada EXTI10 w rejestrze AFIO_EXTICR3
	EXTI_InitTypeDef exti;

	EXTI_StructInit(&exti);
	exti.EXTI_Line = EXTI_Line10;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource10);
	NVIC_InitTypeDef nvicB;

	nvicB.NVIC_IRQChannel = EXTI15_10_IRQn;
	nvicB.NVIC_IRQChannelPreemptionPriority = 0x00;
	nvicB.NVIC_IRQChannelSubPriority = 0x00;
	nvicB.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicB);
}

void init_USART2() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

//	GPIO_InitTypeDef gpio;
	USART_InitTypeDef uart;

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio);

	USART_StructInit(&uart);
	uart.USART_BaudRate = 9600;
	USART_Init(USART2, &uart);

	USART_Cmd(USART2, ENABLE);
}

void init_I2C() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

//	GPIO_InitTypeDef gpio;
	I2C_InitTypeDef i2c;

	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // SCL, SDA
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);

	I2C_StructInit(&i2c);
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_ClockSpeed = 100000;
	I2C_Init(I2C1, &i2c);

	I2C_Cmd(I2C1, ENABLE);

	// init VCC and GND for LCD i2c
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio); // PA10 VCC
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &gpio);
	GPIO_SetBits(GPIOA, 10); // wl zasilania LCD i2c - PA10

	GPIO_StructInit(&gpio); // PC4 GND LCD
	gpio.GPIO_Pin = GPIO_Pin_4;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &gpio);
	GPIO_ResetBits(GPIOC, 4); // masa

}
