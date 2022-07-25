/*
 * PID.c
 *
 *  Created on: 19.11.2019
 *      Author: PiK
 *
 * Utworzenie struktury regulatora PID. Wykorzystane b�d� dwa takie obiekty struktury, jedna do kontroli osi X, a druga do osi Y
 *
 */

#include "stm32f10x.h"
#include "main.h"
#include "PID.h"

float get_PID_output(sPID_controller *controller, int Ts) { // trzeba dobrac Ts do przerwania

//	float y_prev = controller->y_out; // wartosc poprzednia do okreslenia , czy wchodzic do anti-windupu
	/* Updateowanie poprzednich wartosci*/

	controller->yi_p = controller->yi; // poprzednio obliczone yi
	controller->yd_p = controller->yd;

	controller->y_out_pp = controller->y_out_p;
	controller->y_out_p = controller->y_out;

	/* Saturacja i anti-windup */
	// jesli chce przekroczyc granice, to na to nie pozwol, nie calkuj
	if (((controller->y_out_p > controller->limitHigh) && (controller->e_in > 0))
			|| ((controller->y_out_p < controller->limitLow)
					&& (controller->e_in < 0))) {
		printf("Anti-Windup! Przekroczono limit calkowania");
		controller->yi = controller->yi_p;

	} else { // normalnie oblicz yi_out (calkuj)

		//Bez sampling time
		controller->yi += controller->Ki * controller->e_in; // I

////		 Z wpisaniem sampling Time (stare - chyba zle + y_out_p)
//		controller->yi = controller->Ki * controller->e_in
//				+ controller->y_out_p; // I

//		controller->yi = controller->Ki * controller->e_in * Ts
//				+ controller->y_out_p; // I  bylo wczesniej yi_p
	}
	/* Po obliczeniu I licz dalej wyjscie P i D */

	controller->yp = controller->Kp * controller->e_in; // P
//	controller->yd = controller->Kd * (controller->yd - controller->yd_p) / Ts; // D

	controller->yd = controller->Kd * (controller->e_in - controller->e_in_p); // D = error - last error

	controller->y_out = controller->yp + controller->yi + controller->yd;

	controller->e_in_p = controller->e_in;
	return controller->y_out;
}

float get_angle_from_PID_output(sPID_controller *controller,
		float g_inertia_const, float max_change) {
	// x.. = const * rad
	// moze ograniczyc do tego, zeby kolejny rzucony kat nie byl wyzszy o 1 stopien od poprzedniego?

// z wrzuceniem sampling time
//	float angle_rad = (controller->y_out - 2 * controller->y_out_p
//			+ controller->y_out_pp) / Ts / Ts / g_inertia_const;

	controller->next_angle_p = controller->next_angle; // poprzedni kat

	float angle_deg = ((controller->y_out - 2 * controller->y_out_p
			+ controller->y_out_pp) / g_inertia_const) * 57.3;

//	// jesli obliczony, jest wiekszy od poprzedniego o 1, to zmien go tylko o 1
//	if (abs(angle_deg - controller->next_angle_p) > 1) {
//		printf("\n\r DUZA ZMIANA \n\r");
//		if (angle_deg < 0) {
//			controller->next_angle = controller->next_angle_p - max_change;
//		} else {
//			controller->next_angle = controller->next_angle_p + max_change;
//		}
//	} else {
//		controller->next_angle = angle_deg; // obecny kat
//	}

	// surowe podejscie:
	controller->next_angle = angle_deg / 10;
	return controller->next_angle;

//	return angle_deg;
}

void set_PID_params(sPID_controller *controller, float limitHigh,
		float limitLow, float Kp, float Ki, float Kd) {
	/* Konstruktor do obiekut PID_controller: PID, limitHigh, limitLow, Kp, Ki, Kd */

	controller->limitHigh = limitHigh;
	controller->limitLow = limitLow;

	/* wzmocnienia czesci Prop, calk i rozn */
	controller->Kp = Kp;
	controller->Ki = Ki;
	controller->Kd = Kd;

	controller->e_in = 0; // wejscie
	controller->e_in_p = 0;

	controller->yi_p = 0; // poprzednia wartosc wyjscia calkujacego
	controller->yi = 0; // wyjscie calkujace
	controller->yp = 0; // wyjscie proporcjonalne
	controller->yd = 0; // wyjscie rozniczkujace
	controller->yd_p = 0;
	controller->y_out = 0; // wyjscie jako suma PID
	controller->y_out_p = 0; // y(n-1)
	controller->y_out_pp = 0; // y(n-2)

	controller->next_angle = 0;
	controller->next_angle_p = 0;
}
