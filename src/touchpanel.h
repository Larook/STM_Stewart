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
 * 	TP0 (622,401)
 * 	TP1 (647,3670)
 * 	TP2 (3498, 3686)
 * 	TP3 (3512,375)
 * 	TP4 (2040, 2006)
 *
 * 	Potem trzeba dac rzeczywiste wsp tym punktom (kalibracja)
 */

#endif /* TOUCHPANEL_H_ */
