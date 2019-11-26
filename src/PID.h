/*
 * PID.h
 *
 *  Created on: 19.11.2019
 *      Author: PiK
 */
#include "main.h"

#ifndef PID_H_
#define PID_H_

typedef struct {
	/* parametry do anty-windupu */
	float limitHigh;
	float limitLow;

	/* wzmocnienia czesci Prop, calk i rozn */
	float Kp;
	float Ki;
	float Kd;

	float x_in; // wejscie

	float yi_p; // poprzednia wartosc wyjscia calkujacego
	float yi; // wyjscie calkujace

	float yp; // wyjscie proporcjonalne

	float yd; // wyjscie rozniczkujace
	float yd_p;

	float y_out; // wyjscie jako suma PID
	float y_out_p; // y(n-1)
	float y_out_pp; // y(n-2)

} sPID_controller;

sPID_controller PIDx; // globalny regulator PIDx
sPID_controller PIDy;

float get_PID_output(sPID_controller *controller, int Ts);
float get_angle_from_PID_output(sPID_controller *controller,
		float g_inertia_const, float Ts);
void set_PID_params(sPID_controller *controller, float limitHigh,
		float limitLow, float Kp, float Ki, float Kd);

#endif /* PID_H_ */
