/*
 * touchpanel.c
 *
 *  Created on: 06.11.2019
 *      Author: PiK
 */
#include "main.h"
#include "stm32f10x.h"

#define NUMBER_CALIBRATION_POINTS 5

/*
 * Trzeba zrobic to co bylo w Pythonie
 * 5 punktow zapisac do listy punktow
 * Majac liste punktow - oblicz 6 wspolczynnikow kalibracji - zapisz je w strukturze
 * struct sTouchPanel touchPanel
 * i wykorzystuj ja do obliczania prawdziwej wartosci dotyku - getRealTouch
 */

struct sPoint {
	// wspolrzedne z pomiaru ADC
	float x_touch;
	float y_touch;
	// wsporzedne w ukladzie rzeczywistym
	float x_real;
	float y_real;
} tp0, tp1, tp2, tp3, tp4;

//struct sTouchPanel {
//	// zmienne potrzebne do kalibracji panelu dotykowego
//	float alpha_x;
//	float beta_x;
//	float delta_x;
//
//	float alpha_y;
//	float beta_y;
//	float delta_y;
//} touchPanel;
//*p_touchPanel = &touchPanel; // zmienna wskaznikowa *p trzyma adres obiektu struktury

/* gwiazdka wskazuje na adres struktury (podajemy adres struktury i wsp)
 czyli wykonujac te funkcje, musimy podac nie strukture, ale adres do niej &tp0 */

struct sPoint getTouchPoint(struct sPoint *struct_pointer, float tx, float ty,
		float rx, float ry) {
	/* uzupelnij wsp X i Y punktu do kalibracji */
	struct_pointer->x_touch = tx;
	struct_pointer->y_touch = ty;

	struct_pointer->x_real = rx;
	struct_pointer->y_real = ry;

	return *struct_pointer;
}

/* Tworzy liste 5 punktow */
void setPointsArray(struct sPoint *arrayOfPoints[4]) {
	/*
	 * 	TP0 (622,401)		->Rzeczywiste	->		RP0 (-71, -50)
	 * 	TP1 (647,3670)		->Rzeczywiste	->		RP1 (71, -50)
	 * 	TP2 (3498, 3686)	->Rzeczywiste	->		RP2 (71, 50)
	 * 	TP3 (3512,375)		->Rzeczywiste	->		RP3 (-71, 50)
	 * 	TP4 (2040, 2006)	->Rzeczywiste	->		RP4 (0,0)
	 */
//	arrayOfPoints[0]->x_real= 622;
	*arrayOfPoints[0] = getTouchPoint(&tp0, 622, 401, -71, -50);
	*arrayOfPoints[1] = getTouchPoint(&tp1, 647, 3670, 71, -50);
	*arrayOfPoints[2] = getTouchPoint(&tp2, 3489, 3686, 71, 50);
	*arrayOfPoints[3] = getTouchPoint(&tp3, 3512, 375, -71, 50);
	*arrayOfPoints[4] = getTouchPoint(&tp4, 2040, 2006, 0, 0);

}

void set_calibration_matrix_5_points(struct sTouchPanel *panel_pointer,
		struct sPoint *points_pointer[4]) { // podajemy obiekt, a bierze go jako wskaznik
	/* Oblicz wsp macierzy kalibracji*/
	int n = 5; // liczba punktow do kalibracji
	float a, b, c, d, e, x1, x2, x3, y1, y2, y3 = 0;
	float delta, delta_x1, delta_x2, delta_x3, delta_y1, delta_y2, delta_y3 = 0;
	float alpha_x, beta_x, delta_x, alpha_y, beta_y, delta_y = 0;

//calculating for every point
	for (int i = 0; i < n; i++) {
// print(touch_list[i].x, touch_list[i].y, disp_list[i].x, disp_list[i].y)
		// touch
		a += points_pointer[i]->x_touch * points_pointer[i]->x_touch;
		b += points_pointer[i]->y_touch * points_pointer[i]->y_touch;
		c += points_pointer[i]->x_touch * points_pointer[i]->y_touch;
		d += points_pointer[i]->x_touch;
		e += points_pointer[i]->x_touch;

		// real
		x1 += points_pointer[i]->x_touch * points_pointer[i]->x_real;
		x2 += points_pointer[i]->y_touch * points_pointer[i]->x_real;
		x3 += points_pointer[i]->x_real;

		y1 += points_pointer[i]->x_touch * points_pointer[i]->y_real;
		y2 += points_pointer[i]->y_touch * points_pointer[i]->y_real;
		y3 += points_pointer[i]->y_real;
	}

	delta = n * (a * b - c * c) + 2 * c * d * e - a * e * e - b * d * d;
	delta_x1 = n * (x1 * b - x2 * c) + e * (x2 * d - x1 * e)
			+ x3 * (c * e - b * d);
	delta_x2 = n * (x2 * a - x1 * c) + d * (x1 * e - x2 * d)
			+ x3 * (c * d - a * e);
	delta_x3 = x3 * (a * b - c * c) + x1 * (c * e - b * d)
			+ x2 * (c * d - a * e);

	delta_y1 = n * (y1 * b - y2 * c) + e * (y2 * d - y1 * e)
			+ y3 * (c * e - b * d);
	delta_y2 = n * (y2 * a - y1 * c) + d * (y1 * e - y2 * d)
			+ y3 * (c * d - a * e);
	delta_y3 = y3 * (a * b - c * c) + y1 * (c * e - b * d)
			+ y2 * (c * d - a * e);

	alpha_x = delta_x1 / delta;
	beta_x = delta_x2 / delta;
	delta_x = delta_x3 / delta;

	alpha_y = delta_y1 / delta;
	beta_y = delta_y2 / delta;
	delta_x = delta_y3 / delta;

	panel_pointer->alpha_x = alpha_x; // jesli zmienna wskaznikowa zawiera adres zmiennej strukturalnej, to wyluskanie ->
	panel_pointer->beta_x = beta_x;
	panel_pointer->delta_x = delta_x;

	panel_pointer->alpha_y = alpha_y;
	panel_pointer->beta_y = beta_y;
	panel_pointer->delta_y = delta_y;
}

void init_touchPointsCalibration(struct sTouchPanel *panel_pointer) {
	/* Na podstawie podanych 5 punktow oblicz macierz kalibracji */
	struct sPoint pointsArray[4];
	setPointsArray(&pointsArray[4]);
	set_calibration_matrix_5_points(&touchPanel, &pointsArray[4]);
	printf(
			"\nWspolczynniki macierzy kalibracji:\n %f \t%f \t%f \t%f \n %f \t%f \t%f \t%f",
			panel_pointer->alpha_x, panel_pointer->beta_x,
			panel_pointer->delta_x, panel_pointer->alpha_y,
			panel_pointer->beta_y, panel_pointer->delta_y);
}

void getRealTouch(struct sPoint *point, struct sTouchPanel *panel_pointer) {
	/* Oblicza wsp rzeczywiste na podstawie macierzy kalibracji */
	point->y_real = 1;
	point->x_real = 1;
}

