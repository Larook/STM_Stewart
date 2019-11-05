/*
 * kinematics.c
 *
 *  Created on: 24.09.2019
 *      Author: PiK
 */
#include <fastmath.h>
#include "kinematics.h"

/*			Zmienne 					*/
//k¹t beta ustawienia serwa na p³aszczyŸnie XY
double Betak[6] = { 0, 180, 120, 300, 240, 60 };

//pkty snapa przy serwie
double Bk[6][6][6] = { { { -21 }, { 21 }, { 68 }, { 49 }, { -49 }, { -68 } }, {
		{ -71 }, { -71 }, { 19 }, { 48 }, { 48 }, { 19 } }, { { 0 }, { 0 },
		{ 0 }, { 0 }, { 0 }, { 0 } } };
double BkX[6] = { -21, 21, 68, 49, -49, -68 };
double BkY[6] = { -71, -71, 19, 48, 48, 19 };
double BkZ[6] = { 0 };

//pkty snapow na platformie
double Pk[6][6][6] = { { { -56 }, { 56 }, { 62 }, { 4 }, { -4 }, { -62 } }, { {
		-41 }, { -41 }, { -28 }, { 69 }, { 69 }, { -28 } }, { { 123 }, { 123 },
		{ 123 }, { 123 }, { 123 }, { 123 } } };

double PkX[6] = { -56, 56, 62, 4, -4, -62 };
double PkY[6] = { -41, -41, -28, 69, 69, -28 };
double PkZ[6] = { 123, 123, 123, 123, 123, 123 };

/*
 double PkX[6] = { -51, 51, 62, 5, -5, -62 };
 double PkY[6] = { -39, -39, -28, 70, 70, -28 };
 double PkZ[6] = { 123, 123, 123, 123, 123, 123 };
 */

//dlugosc nog platformy (od oski serwa do snapa platformy)
double LkX[6] = { 0 };
double LkY[6] = { 0 };
double LkZ[6] = { 0 };

//------------ Sterowanie przez zadanie dlugosci nogom -----------------
//robimy translacje -> zmiana dlugosci nog -> zmiana kata alfa -> ustawienie serwa

double getLegLenghtforTranslationServoX(int servo, double tx, double ty,
		double tz) {
	//wpisujesz o ile ma sie przesunac punkt srodkowy platformy, przy zachowaniu katow Eulera=0
	//potem rozbudowac o katy Roll Pitch Yaw
	LkX[servo] = PkX[servo] - BkX[servo] + tx;
	LkY[servo] = PkY[servo] - BkY[servo] + ty;
	LkZ[servo] = PkZ[servo] - BkZ[servo] + tz;

	double l = 1.0
			* sqrt(
					LkX[servo] * LkX[servo] + LkY[servo] * LkY[servo]
							+ LkZ[servo] * LkZ[servo]);	//dlugosc nogi na podstawie wspolrzednych
	return l;
}

double getLegLenghtforTranslationServoX_RPY(int servo, double tx, double ty,
		double tz, double Roll, double Pitch, double Yaw) {
	//wpisujesz o ile ma sie przesunac punkt srodkowy platformy, przy zachowaniu katow Eulera=0
	//potem rozbudowac o katy Roll Pitch Yaw
	double sR = sin(Roll * 0.017); // *0.017 przeliczenie na radiany
	double cR = cos(Roll * 0.017);
	double sP = sin(Pitch * 0.017);
	double cP = cos(Pitch * 0.017);
	double sY = sin(Yaw * 0.017);
	double cY = cos(Yaw * 0.017);

	LkX[servo] = PkX[servo] * (cY * cP) + PkY[servo] * (-sY * cR + cY * sP * sR)
			+ PkZ[servo] * (sY * sR + cY * sP * cR) - BkX[servo] + tx;
	LkY[servo] = PkX[servo] * (sY * cP) + PkY[servo] * (cY * cR + sY * sP * sR)
			+ PkZ[servo] * (-1 * cY * sR + sY * sP * cR) - BkY[servo] + ty;
	LkZ[servo] = PkX[servo] * (-1 * sP) + PkY[servo] * (cP * sR)
			+ PkZ[servo] * (cP * cR) - BkZ[servo] + tz;
	double l = sqrt(
			LkX[servo] * LkX[servo] + LkY[servo] * LkY[servo]
					+ LkZ[servo] * LkZ[servo]); //dlugosc nogi na podstawie wspolrzednych
	return l;
}

//majac dlugosc nogi odpowiednia dla danego przesuniecia trzeba wiedziec pod jakim katem obrocic serwo, aby noga miala te dlugosc
double getAlphaFromLegLengthServoX(int servo, double LegLength) {

	double L = LegLength * LegLength - d_Arm * d_Arm + h_Arm * h_Arm;
	double M = 2 * h_Arm * (PkZ[servo] - BkZ[servo]); // moze te katy beta itp zapisac w radianach?
	double N = 2
			* (cos(Betak[servo]) * (PkX[servo] - BkX[servo])
					+ sin(Betak[servo] * (PkY[servo] - BkY[servo])));
	double Alpha = asin(L / (sqrt(M * M + N * N))) - atan2(N, M);
	Alpha = Alpha * 57.3; // rads to degrees
	return Alpha;
}

//mozemy teraz obliczyc w jakim kacie powinno byc serwo, aby dlugosc nogi byla w zadanym miejscu
//teraz trzeba ogarnac sterowanie polozeniem platformy za pomoca dlugosci nog
void movePlatformFromTranslation(double tx, double ty, double tz) {
	for (int i = 0; i < 6; i++) {
		moveServXPosY(i + 1,
				getAlphaFromLegLengthServoX(i,
						getLegLenghtforTranslationServoX(i, tx, ty, tz)));
	}
}

void movePlatformFromTranslation_RPY(double tx, double ty, double tz,
		double Roll, double Pitch, double Yaw) {
	for (int i = 0; i < 6; i++) {
		moveServXPosY(i + 1,
				getAlphaFromLegLengthServoX(i,
						getLegLenghtforTranslationServoX_RPY(i, tx, ty, tz,
								Roll, Pitch, Yaw)));
	}
}

//------------Home Position proby------------------------
//zwraca nam na jakiej wysokosci powinny byc punkty P aby platforma byla w HomePosition
double getHeightHomePosistion(int servo) { //WYCHODZI ZDECYDOWANIE ZA MALA WSOKOSC h0, chociaz 126 to niby ok
	//pozycja home to taka, gdzie bok orczyka serwa i pret ramienia tworza kat prosty (platforma lekko uniesiona)
	//w HomePosition qk = [xp, yp, zp+heightHome]^T

//	double l1 = d_Arm*d_Arm+h_Arm*h_Arm;
//	double l1 = pow((PkX[servo]-BkX[servo]),2)+pow((PkY[servo]-BkY[servo]),2) + pow(PkZ[0]+HomePosition,2) ;
	servo--; //zeby nr serwa do timerow pasowal do nr serwa tablicy
	double HeightHome = sqrt(
			d_Arm * d_Arm + h_Arm * h_Arm - pow((PkX[servo] - BkX[servo]), 2)
					- pow((PkY[servo] - BkY[servo]), 2)) - PkZ[servo];
	printf("HeightHome %d = %f \n\r", servo + 1, HeightHome);
	return HeightHome;
}

double getAlphaHomePosistion(int servo) { // JESZCZE RAZ ZROBIC ZWYMIAROWANIE Bk i Pk
	//A JAK NIE ZADZIALA TO WALIC TO I ZAJAC SIE ROTACJA
	// Zadzialalo tylko dla serwa nr 1, ktore bylo wymiarowane

	//pod jakim katem sa obrocone serwa aby tworzyc kat 90st
	servo--; //zeby nr serwa do timerow pasowal do nr serwa tablicy
	double LHome = 2 * h_Arm * h_Arm;
	double MHome = 2 * h_Arm * (PkX[servo] - BkX[servo]);
	double NHome = 2 * h_Arm * (PkZ[servo] + getHeightHomePosistion(servo));

	double AlphaHome = asin(LHome / sqrt(MHome * MHome + NHome * NHome))
			- atan2(MHome, NHome); //bylo -atan(MHome/NHome)
	AlphaHome = AlphaHome * 57.3; //rad to degrees

	printf("AlphaHome %d = %f \n\r", servo + 1, AlphaHome);
	return AlphaHome; // obrocic serwo na ten kat zeby przemiescic Pi w nowe miejsce
}

// ------------------ poruszanie serwa na zadane pozycje wzgledne:

int getServXAngleOfPosY(int serwo, double pos) { //USED
//wpisuje ktore serwo na jaka pozycje ma sie ruszyc,
//i zwraca kat wymagany do pozycji do ktorej sie ma ustawic
// kat potem zmieniany jest na ticki ktore ida do wypelnienia pwm timera
	int posToAngle;
	if ((serwo == 1) | (serwo == 3) | (serwo == 5)) {
		//zadane 0 - ma odpowiadac katowi 90 AngleToTicks(angle)
		if ((pos >= 0) & (pos <= 90)) {
			posToAngle = pos + 90;
			return posToAngle; //zwraca kat na jaki ma sie obrocic
		}
	} else if ((serwo == 2) | (serwo == 4) | (serwo == 6)) {
		// zadane 0 - odp. katowi 180
		if ((pos >= 0) & (pos <= 90)) {
			posToAngle = 180 - pos;
			return posToAngle;
		}
	}
	return 0;
}

void moveServXPosY(int serwo, double pos) { //USED NAJBARDZIEJ
//piszesz ktore serwo i na jaka pozycje ma sie ruszyc
// wybieram serwo i chce mu zadac pozycje, ale pozycje trzeba zamienic w kat a kat w ticksy
	if ((pos >= -36) & (pos <= 29)) { // przyjmuje katy od -35 do 30
		switch (serwo) {
		case 1:
			// pozycje ustawia przez odczytanie jakie ma byc wypelnienie zeby ustawic TeSerwo na TaPozycje
			TIM_SetCompare1(TIM3,
					AngleToTicks(getServXAngleOfPosY(serwo, pos + 35))); //dodany offset zeby zadane 0 stawialo serwo w pozycji horyzontalnej
			break;
		case 2:
			TIM_SetCompare2(TIM3,
					AngleToTicks(getServXAngleOfPosY(serwo, pos + 35)));
			break;
		case 3:
			TIM_SetCompare3(TIM3,
					AngleToTicks(getServXAngleOfPosY(serwo, pos + 30)));
			break;
		case 4:
			TIM_SetCompare4(TIM3,
					AngleToTicks(getServXAngleOfPosY(serwo, pos + 30)));
			break;
		case 5:
			TIM_SetCompare3(TIM4,
					AngleToTicks(getServXAngleOfPosY(serwo, pos + 32)));
			break;
		case 6:
			TIM_SetCompare4(TIM4,
					AngleToTicks(getServXAngleOfPosY(serwo, pos + 37)));
			break;
		default:
			break;
		}
	} else {
		printf("\tZle zadany kat\n\r");
	}
}

/*		funkcje ruchu		*/
void moveCircle(double radius, double T_round, double tx, double ty, struct Environments environment) {
	printf("\nNakurwiam salto");
	int step = 0;

	double phi = 0.0; // dodac ew poczatkowe przewidzenie punktu koncowego i na jego podstawie wziac startowy
	double x = 0.0;
	double y = 0.0;
	int ms_t = 50; // ms 1 stepa
	int ms_oper = 46; // 43 ms na operacje

	// trzeba zadawac staly RPM
	for (step; phi < 6.28; step++) {
		phi += 6.283 * ms_t / (T_round * 1000);
		x = radius * cos(1*phi);
		y = radius * sin(1*phi);

		printf("\nstep %3d | kat=%3.2f[st]  x=%.1f  y=%.1f", step, phi * 57.3,
				x, y);

		movePlatformFromTranslation_RPY(1*x+tx, 1*y+ty, environment.PlatformZ, 0*4 * sin(1*phi),
				0*4 * cos(1*phi), 0*5* sin(phi));

		/*
		 * movePlatformFromTranslation_RPY(1*x+tx, 1*y+ty, -9 + 9 * sin(3*phi), 0*4 * sin(1*phi),
				0*4 * cos(1*phi), 0*5* sin(phi));
		 */
		delay_ms(ms_t - ms_oper);
	}
	/*
	 while (phi < 360*0.0175) {
	 x = radius * cos(phi);
	 y = radius * sin(phi);
	 printf("\nkrok %3d  czas %3f  |  kat=%6.2f[st]  x=%3d  y=%3d",step, step*0.1 ,phi* 57.3, x, y);
	 //phi = phi+ 1/steps_per_round *360*57.3;
	 phi += 360*0.0175/steps_per_round;

	 movePlatformFromTranslation_RPY(x, y, z ,0,0,0);
	 delay_ms(1);
	 step++;
	 }
	 */

}

