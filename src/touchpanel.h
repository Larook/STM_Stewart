/*
 * touchpanel.h

 *
 *  Created on: 06.11.2019
 *      Author: PiK
 */
/*
 * Przechowuje stale potrzebne do zamiany pomiaru z 12bit ADC na wspolrzedne platformy
 * kalibracja na podstawie:
 * www.ti.com/lit/an/slyt277/slyt277.pdf
 *
 */
#include "main.h"

#ifndef TOUCHPANEL_H_
#define TOUCHPANEL_H_

/*
 * gdy nie ma dotyku		X = 1470		Y = 2870  ADC przy ustawieniu przerwania Period = 10
 * pomiary TPx wykonane sa 1cm od krawedzi
 * 					____________
 * 					|TP1	TP2 |
 * 					| 			|
 * 					|			|
 * 					|	TP4		|
 * 					|	  '		|
 * 					|			|___
 * 					|			|___ 4 wires
 * 					|TP0_____TP3|
 *
 * 	Przypisanie wspolrzednych TP_adc do wsp rzeczywistych
 * 	TP0 (622,401)		->Rzeczywiste	->		RP0 (-71, -50)
 * 	TP1 (647,3670)		->Rzeczywiste	->		RP1 (71, -50)
 * 	TP2 (3498, 3686)	->Rzeczywiste	->		RP2 (71, 50)
 * 	TP3 (3512,375)		->Rzeczywiste	->		RP3 (-71, 50)
 * 	TP4 (2040, 2006)	->Rzeczywiste	->		RP4 (0,0)
 *
 * 	Potem trzeba dac rzeczywiste wsp tym punktom (kalibracja)
 */
setTouchPoint(struct sPoint *struct_pointer, float tx, float ty, float rx,
		float ry);
setPointsArray(struct sPoint *arrayOfPoints[4]);
set_calibration_matrix_5_points(struct sTouchPanel *panel_pointer,
		struct sPoint *points_pointer[4]);
init_touchPointsCalibration(struct sTouchPanel *panel_pointer);
getRealTouch(int16_t touchX, int16_t touchY, struct sTouchPanel *panel_pointer);

#endif /* TOUCHPANEL_H_ */
