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
	if (((controller->y_out_p > controller->limitHigh) && (controller->x_in > 0))
			|| ((controller->y_out_p < controller->limitLow) && (controller->x_in < 0))) {
		controller->yi = controller->yi_p;

	} else { // normalnie oblicz yi_out (calkuj)

		controller->yi = controller->Ki * controller->x_in * Ts
				+ controller->yi_p; // I
	}
	/* Po obliczeniu I licz dalej wyjscie P i D */

	controller->yp = controller->Kp * controller->x_in; // P
	controller->yd = controller->Kd * (controller->yd - controller->yd_p) / Ts; // D

	controller->y_out = controller->yp + controller->yi + controller->yd;

	return controller->y_out;
}

float get_angle_from_PID_output(sPID_controller *controller,
		float g_inertia_const, float Ts) {
	// x.. = const * rad
	float angle_rad = (controller->y_out - 2 * controller->y_out_p
			- controller->y_out_pp) / Ts / Ts / g_inertia_const;

	return angle_rad;
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

	controller->x_in = 0; // wejscie

	controller->yi_p = 0; // poprzednia wartosc wyjscia calkujacego
	controller->yi = 0; // wyjscie calkujace
	controller->yp = 0; // wyjscie proporcjonalne
	controller->yd = 0; // wyjscie rozniczkujace
	controller->yd_p = 0.1;
	controller->y_out = 0; // wyjscie jako suma PID
	controller->y_out_p = 0; // y(n-1)
	controller->y_out_pp = 0; // y(n-2)
}
