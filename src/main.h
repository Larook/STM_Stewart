/*
 * main.h
 *
 *  Created on: 23.09.2019
 *      Author: PiK
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "lsm303d.h"
#include "enable_things.h"
#include "servos_platform.h"
//#include "kinematics.h"

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

#endif /* MAIN_H_ */
