/*
 * servos_platform.c
 *
 *  Created on: 13.02.2019
 *      Author: PiK
 */
#include "stm32f10x.h"

void init_timers_for_6servos() {
	GPIO_InitTypeDef gpio;
	TIM_TimeBaseInitTypeDef tim;
	TIM_OCInitTypeDef channel;
	NVIC_InitTypeDef nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

	/* WYBRANIE PINOW NA CHANELE TIMEROW*/
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // TIM3: CH1(6) i CH2(7)
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9; //TIM3: CH3(0) i CH4(1); TIM4: CH3(8) i CH4(9)
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &gpio);

	/*USTAWIENIA TIMEROW I CHANNELOW*/
	//caly okres ma trwac 20ms , taktowanie zegara 64MHz
	//czyli (Period-1)*(PSC-1)=1280000 bo 72MHz/1280kHz=50Hz
	//czyli PSC =64, a Period = 20000
	//TIM_SetCompare[nrKanalu](TIM[nrTimera], [iloscProbek]);
	/*TIM3 */
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 64 - 1;
	tim.TIM_Period = 20000 - 1;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &tim);

	TIM_OCStructInit(&channel);
	channel.TIM_OCMode = TIM_OCMode_PWM1; //wypelnienie jedynka 1
	channel.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC1Init(TIM3, &channel);
	TIM_OC2Init(TIM3, &channel);
	TIM_OC3Init(TIM3, &channel);
	TIM_OC4Init(TIM3, &channel);
	TIM_Cmd(TIM3, ENABLE);

	nvic.NVIC_IRQChannel = TIM3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	/*TIM4 */
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 64 - 1;
	tim.TIM_Period = 20000 - 1;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	tim.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &tim);

	//TIM_SetCompare1(TIM4, X);
	//porownuje zawartosc rejestrow timera z wartoscia X

	TIM_OCStructInit(&channel);
	channel.TIM_OCMode = TIM_OCMode_PWM1; //wypelnienie jedynka 1
	channel.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM4, &channel);
	TIM_OC4Init(TIM4, &channel);
	TIM_Cmd(TIM4, ENABLE);

	nvic.NVIC_IRQChannel = TIM4_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

}
//--------------------------koniec serva
