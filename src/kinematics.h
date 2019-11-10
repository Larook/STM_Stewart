/*
 * kinematics.h
 *
 *  Created on: 24.09.2019
 *      Author: PiK
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "main.h"

/*        STALE MECHANICZNE PLATFORMY      */
//definiowane parametry mechaniczne [mm]
#define h_Arm 35
#define d_Arm 129
#define h0 123
//double l_Arm; // zmienna
double Alphak;

//punkty polozenia globalnego ukl wspolrzednych, dane w Excelu
//C:\Users\PiK\Desktop\Elektronika\Stewart_Platform

//NOTACJA ZGODNA Z OPRACOWANIEM NA https://www.xarg.org/paper/inverse-kinematics-of-a-stewart-platform/
//pkty osi serwa
#define SkX [6] = { -50.73, 50.73, 89.45, 39.09, -39.09, -89.03 };
#define SkY [6] = { -74, -74, 6.93, 80.3, 80.3, -6.3 };
#define SkZ [6] = { 0.0 };

// ------------------------------------------------------------
double getLegLenghtforTranslationServoX(int servo, double tx, double ty,
		double tz);

double getLegLenghtforTranslationServoX_RPY(int servo, double tx, double ty,
		double tz, double Roll, double Pitch, double Yaw);
double getAlphaFromLegLengthServoX(int servo, double LegLength);

void movePlatformFromTranslation(double tx, double ty, double tz);

void movePlatformFromTranslation_RPY(double tx, double ty, double tz,
		double Roll, double Pitch, double Yaw);

//------------Home Position proby------------------------
//zwraca nam na jakiej wysokosci powinny byc punkty P aby platforma byla w HomePosition
double getHeightHomePosistion(int servo);
double getAlphaHomePosistion(int servo);

// pozycje wzgledne serwa
int getServXAngleOfPosY(int serwo, double pos);
void moveServXPosY(int serwo, double pos);

// trajektorie
void moveCircle(double radius, double T_round, struct sEnvironment);

#endif /* KINEMATICS_H_ */
