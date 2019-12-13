/*
 * main.h
 *
 *  Created on: 23.09.2019
 *      Author: PiK
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f10x.h"

//#include "lsm303d.h"
//#include "enable_things.h"
//#include "servos_platform.h"
//#include "kinematics.h"
#define ADC_CHANNELS	5

uint16_t adc_value[ADC_CHANNELS]; //tablica z wynikami adc z DMA

GPIO_InitTypeDef gpio; // jedne globalne gpio
TIM_TimeBaseInitTypeDef tim;
TIM_OCInitTypeDef channel;
NVIC_InitTypeDef nvic;

struct sTouchPanel {
	// zmienne potrzebne do kalibracji panelu dotykowego
	float alpha_x;
	float beta_x;
	float delta_x;

	float alpha_y;
	float beta_y;
	float delta_y;
} touchPanel;

struct sEnvironment {

	// joystick i potencjometr
	int16_t PlatformX;
	int16_t PlatformY;
	int16_t PlatformZ;

	// IMU
	int16_t Roll;
	int16_t Pitch;
	int16_t Yaw;

	// touchpanel
	uint16_t X_TouchPanel;
	uint16_t Y_TouchPanel;

	// podane w [mm]
	float X_Real;
	float Y_Real;

	float next_angle_Roll;
	float next_angle_Pitch;

	// katy, ktore trzeba nadac zeby talerz byl rzeczywiscie poziomo
	float roll_level_bias;
	float pitch_level_bias;

} env;



//struct Environments env; // w tym srodowisku uzupelnij zmierzone pomiary

double degToRad(double degree);

/*
 // ------------- Deklaracja funkcji
 double getLegLenghtforTranslationServoX(int servo, int tx, int ty, int tz);
 double getLegLenghtforTranslationServoX_RPY(int servo, int tx, int ty, int tz,
 double Roll, double Pitch, double Yaw);
 double getAlphaFromLegLengthServoX(int servo, double LegLength);
 void movePlatformFromTranslation(int tx, int ty, int tz);
 double getHeightHomePosistion(int servo);
 double getAlphaHomePosistion(int servo);

 int AngleToTicks(double angle);
 int getServXAngleOfPosY(int serwo, double pos);

 void moveServXPosY(int serwo, double pos);
 */

int getXJoystick(int max);
int getYJoystick(int max);
int getZPotentiometer();

int getY_touchPanel();
int getY_touchPanel();

#endif /* MAIN_H_ */