/*
 * touchpanel.c
 *
 *  Created on: 06.11.2019
 *      Author: PiK
 */
#include "main.h"

#define NUMBER_CALIBRATION_POINTS 5

/*
 * Trzeba zrobic to co bylo w Pythonie
 * 5 punktow zapisac do listy punktow
 */

struct sPoint {
	// wspolrzedne z pomiaru ADC
	float x_touch;
	float y_touch;

	// wsporzedne w ukladzie rzeczywistym
	float x_real;
	float y_real;
} tp0, tp1, tp2, tp3, tp4;

struct sTouchPanel {
	// zmienne potrzebne do kalibracji:
	float alpha_x;
	float beta_x;
	float delta_x;

	float alpha_y;
	float beta_y;
	float delta_y;
} touchPanel;
//*p_touchPanel = &touchPanel; // zmienna wskaznikowa *p trzyma adres obiektu struktury

void set_calibration_matrix_5_points(struct sTouchPanel *panelCalibration, 5pointsList,) { // podajemy obiekt, a bierze go jako wskaznik
	panelCalibration->alpha_x = 0; // jesli zmienna wskaznikowa zawiera adres zmiennej strukturalnej, to wyluskanie ->
	panelCalibration->beta_x = 0;
	panelCalibration->delta_x = 0;

	panelCalibration->alpha_y = 0;
	panelCalibration->beta_y = 0;
	panelCalibration->delta_y = 0;
	;
}
