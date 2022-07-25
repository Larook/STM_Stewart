/*
 * lsm303d.c
 *
 *  Created on: 08.02.2019
 *      Author: PiK
 */

#include "stm32f10x.h"
#include "lsm303d.h"
#include <fastmath.h>
#include <stdio.h>

#define LSM303D_ADDR			0x3a

//zmienne globalne z wartosciami
double roll_M, pitch_M, yaw_M, roll_A, pitch_A, yaw_A;
//double Hx, Hy, Hz, He;
//double H_wyp;
//double Ax, Ay, Az;
//int16_t m_x, m_y, m_z; //m_xh;

void check_i2c_LSM303D() {
	printf("Wyszukiwanie akcelerometru...\n");

	uint8_t who_am_i = 0;
	lsm_read(0x0f, &who_am_i, sizeof(who_am_i));

	if (who_am_i == 0x49) {
		printf("Znaleziono akcelerometr LSM303D\n");
	} else {
		printf("Nie LSM303D \n Odpowied� uk�adu (0x%02X)\n", who_am_i);
	}
}
//---------------------- odczyty RPY ---------------------
float getRollIMU() {
	int16_t a_y = lsm_read_value(LSM303D_OUT_Y_A);
	int16_t a_z = lsm_read_value(LSM303D_OUT_Z_A);

	double Ay = a_y * 2.0f / 32678.0f;
	double Az = a_z * 2.0f / 32678.0f;

	float roll_A = (atan2(Ay, Az) * 57.2958); //kondycjonowanie

	return roll_A ; // = 180 - roll_A;

}
float getPitchIMU() { //po co powtarzac te pomiary?
	// moze w 1 metodzie zrobic wszystkie pomiary, a potem w jakims 32bitowym rejestrze to podzielic na 3?
	//potem wyslac dane do maina i w mainie obliczac RPY?

	int16_t a_x = lsm_read_value(LSM303D_OUT_X_A);
	int16_t a_y = lsm_read_value(LSM303D_OUT_Y_A);
	int16_t a_z = lsm_read_value(LSM303D_OUT_Z_A);
	//printf("X = %d   Y = %d   Z = %d\n", a_x, a_y, a_z);

	//PRZYSPIESZENIE z akcelerometru
	double Ax = a_x * 2.0f / 32678.0f;
	double Ay = a_y * 2.0f / 32678.0f;
	double Az = a_z * 2.0f / 32678.0f;

	float pitch_A = (atan2((-Ax), sqrt(Ay * Ay + Az * Az))) * 57.2958;
	return pitch_A;
}
float getYawIMU() {
	int16_t m_x = lsm_read_value(LSM303D_OUT_X_M);
	int16_t m_y = lsm_read_value(LSM303D_OUT_Y_M);

	//UWAGA byLo -1 , czyli Hx = 1-* m_x * 2.00f / 32678.00f
	double Hx = m_x * 2.00f / 32678.00f; // te -1 sprawiaja, ze acc i magn maja pomiary w fazie
	double Hy = m_y * 2.00f / 32678.00f; //*2 zeby byla ilosc w Gaussach

	float yaw_M = atan2(Hy, Hx) * 57.2958 + 180;
	yaw_M -= 225; // sam ustalam gdzie jest polnoc
	return yaw_M;
}

void get_Accelerometer() {
	int16_t a_x = lsm_read_value(LSM303D_OUT_X_A);
	int16_t a_y = lsm_read_value(LSM303D_OUT_Y_A);
	int16_t a_z = lsm_read_value(LSM303D_OUT_Z_A);
	//		printf("X = %d   Y = %d   Z = %d\n", a_x, a_y, a_z);

	//PRZYSPIESZENIE z akcelerometru
	double Ax = a_x * 2.0f / 32678.0f;
	double Ay = a_y * 2.0f / 32678.0f;
	double Az = a_z * 2.0f / 32678.0f;

// HighPassFilter w acc moze tylko odfiltrowac wplyw przyspieszenia ziemskiego, tak ze zostaje przysp "wzgledne"

	double roll_A = (atan2(Ay, Az) * 57.2958); //kondycjonowanie
	if (roll_A > 0) {
		roll_A = 180 - roll_A;

	} else if (roll_A < 0) {
		roll_A = -180 - roll_A;
	}

	double pitch_A = (atan2((-Ax), sqrt(Ay * Ay + Az * Az))) * 57.2958;

	printf("roll_A = %.2f \t", roll_A);
	printf("pitch_A = %.2f \t", pitch_A);

}
void get_Magnetometer2() {
// zakres +/- 4Gauss
	//pole magnetyczne Ziemi od 30 do 60 mikro Tesli
	//1T =10e4Gauss
	//45e-6 T = 45e-2Gauss=0,45Gauss

	// na rejestrach HIGH tez sa warto�ci, ale jakies randomowe

	int16_t m_x = lsm_read_value(LSM303D_OUT_X_M);
//	m_xh = lsm_read_value(LSM303D_OUT_X_M_H);
	int16_t m_y = lsm_read_value(LSM303D_OUT_Y_M);
	int16_t m_z = lsm_read_value(LSM303D_OUT_Z_M);

	//UWAGA byLo -1 , czyli Hx = 1-* m_x * 2.00f / 32678.00f
	double Hx = m_x * 4.00f / 32678.00f; // te -1 sprawiaja, ze acc i magn maja pomiary w fazie
//	double Hx_high = m_xh * 2.00f / 32678.00f;
	double Hy = m_y * 4.00f / 32678.00f; //*2 zeby byla ilosc w Gaussach
	double Hz = m_z * 4.00f / 32678.00f;

	double H_wyp = sqrt(Hx * Hx + Hy * Hy + Hz * Hz);

	double roll_M = atan2(Hy, Hz) * 57.2958;
	double pitch_M = (atan2((-Hx), sqrt(Hy * Hy + Hz * Hz))) * 57.2958 + 90;
	double yaw_M = atan2(Hy, Hx) * 57.2958 + 180;
	yaw_M -= 225; // sam ustalam gdzie jest polnoc

	printf("H_x= %.4f \t H_y= %.4f \t H_z=%.4f \t H_wyp= %.4f \n", Hx, Hy, Hz,
			H_wyp);

//	printf("rejestr Low = %s \n", m_x);
//	printf("rejestr High = %s \n", m_xh);
//	printf("yaw_M = %.2f \n", yaw_M);

}

void get_Magnetometer() {
// zakres +/- 2Gauss
	//pole magnetyczne Ziemi od 30 do 60 mikro Tesli
	//1T =10e4Gauss
	//45e-6 T = 45e-2Gauss=0,45Gauss

	// na rejestrach HIGH tez sa warto�ci, ale jakies randomowe

	int16_t m_x = lsm_read_value(LSM303D_OUT_X_M);
//	m_xh = lsm_read_value(LSM303D_OUT_X_M_H);
	int16_t m_y = lsm_read_value(LSM303D_OUT_Y_M);
	int16_t m_z = lsm_read_value(LSM303D_OUT_Z_M);

	//UWAGA byLo -1 , czyli Hx = 1-* m_x * 2.00f / 32678.00f
	double Hx = m_x * 2.00f / 32678.00f; // te -1 sprawiaja, ze acc i magn maja pomiary w fazie
//	double Hx_high = m_xh * 2.00f / 32678.00f;
	double Hy = m_y * 2.00f / 32678.00f; //*2 zeby byla ilosc w Gaussach
	double Hz = m_z * 2.00f / 32678.00f;

	double H_wyp = sqrt(Hx * Hx + Hy * Hy + Hz * Hz);

	double roll_M = atan2(Hy, Hz) * 57.2958;
	double pitch_M = (atan2((-Hx), sqrt(Hy * Hy + Hz * Hz))) * 57.2958 + 90;
	double yaw_M = atan2(Hy, Hx) * 57.2958 + 180;
	yaw_M -= 225; // sam ustalam gdzie jest polnoc

//	printf("H_x= %.4f \t H_y= %.4f \t H_z=%.4f \t H_wyp= %.4f \n", Hx, Hy, Hz,
//			H_wyp);

//	printf("rejestr Low = %s \n", m_x);
//	printf("rejestr High = %s \n", m_xh);
	printf("yaw_M = %.2f \n", yaw_M);

}

void set_default_Accelerometer() {
	printf("Wlaczam odczyt akcelerometru na 50Hz\n");
	lsm_write_reg(LSM303D_CTRL1, 0x50 | 0x07); // f akcelerometru =50Hz | wlacz aX,aY,aZ
	delay_ms(100);
	printf("Teraz:  CRL1 = 0x%x \n", lsm_read_reg(LSM303D_CTRL1));
}

void set_default_Magnetometer() {
	printf("Wlaczam odczyt temp i magne\n");
	// CTRL5 wlaczenie odczytu temperatury | czest pracy i rozdzielczosc magnetometru
	lsm_write_reg(LSM303D_CTRL5, 0x80 | 0x03 << 5 | 0x04 << 2); // wlaczony odczyt temperatury| high resolution | f magnetometru=50Hz
	delay_ms(100);
	printf("Teraz:  CRL5 = 0x%x \n", lsm_read_reg(LSM303D_CTRL5));

	// IG_CFG1 6-direction detection function enabled ?? moze niepotrzebne
	lsm_write_reg(LSM303D_IG_CFG1, 1 << 6);
	printf("Teraz:  IF_CFG1 = 0x%x \n", lsm_read_reg(LSM303D_IG_CFG1));

	// INT_CTRL_M wlaczenie przerwan ??? chyba niepotrzebne
	lsm_write_reg(LMS303D_INT_CTRL_M, 7 << 5 | 8); // na poczatku bylo 0xe8
	printf("Teraz:  LMS303D_INT_CTRL_M = 0x%x \n",
			lsm_read_reg(LMS303D_INT_CTRL_M));

	// CTRL6 Ustawianie skali na +/- 2gauss
	lsm_write_reg(LSM303D_CTRL6, 0x00); // bylo 0x20
	printf("Teraz:  LSM303D_CTRL6 = 0x%x \n", lsm_read_reg(LSM303D_CTRL6));

	//CTRL7 Ustawienie ci�g�ego trybu pracy
	lsm_write_reg(LSM303D_CTRL7, 0x00); //bylo 0x3
	printf("Teraz:  LSM303D_CTRL7 = 0x%x \n", lsm_read_reg(LSM303D_CTRL7));
}

//sprawdzenie czy bajtyH tez maja dane
void set_pomiary_Magnetometer() {
	printf("Wlaczam odczyt temp i magne\n");
	// CTRL5 wlaczenie odczytu temperatury | czest pracy i rozdzielczosc magnetometru
	lsm_write_reg(LSM303D_CTRL5, 0x80 | 0x03 << 5 | 0x04 << 2); // wlaczony odczyt temperatury| high resolution | f magnetometru=50Hz
	delay_ms(100);
	printf("Teraz:  CRL5 = 0x%x \n", lsm_read_reg(LSM303D_CTRL5));

	// IG_CFG1 6-direction detection function enabled ?? moze niepotrzebne
	lsm_write_reg(LSM303D_IG_CFG1, 1 << 6);
	printf("Teraz:  IF_CFG1 = 0x%x \n", lsm_read_reg(LSM303D_IG_CFG1));

	// INT_CTRL_M wlaczenie przerwan ??? chyba niepotrzebne
	lsm_write_reg(LMS303D_INT_CTRL_M, 7 << 5 | 8); // na poczatku bylo 0xe8
	printf("Teraz:  LMS303D_INT_CTRL_M = 0x%x \n",
			lsm_read_reg(LMS303D_INT_CTRL_M));

	// CTRL6 Ustawianie skali na +/- 4gauss
	lsm_write_reg(LSM303D_CTRL6, 1 << 6); // bylo 0x20
	printf("Teraz:  LSM303D_CTRL6 = 0x%x \n", lsm_read_reg(LSM303D_CTRL6));

	//CTRL7 Ustawienie ci�g�ego trybu pracy
	lsm_write_reg(LSM303D_CTRL7, 0x00); //bylo 0x3
	printf("Teraz:  LSM303D_CTRL7 = 0x%x \n", lsm_read_reg(LSM303D_CTRL7));
}

void get_Temperature() {
	int16_t temp = lsm_read_value(LSM303D_TEMP_OUT);
	printf("Temp = %d\n", temp);
}

//******funkcje sluzace do uzyskania wymiany danych************
void lsm_write_reg(uint8_t reg, uint8_t value) {
	lsm_write(reg, &value, sizeof(value));
}

uint8_t lsm_read_reg(uint8_t reg) {
	uint8_t value = 0;
	lsm_read(reg, &value, sizeof(value));
	return value;
}
int16_t lsm_read_value(uint8_t reg) {
	int16_t value = 0;
	lsm_read(reg, &value, sizeof(value));
	return value;
}

void lsm_set_reg(uint8_t reg) {
	I2C_GenerateSTART(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
		;
	I2C_Send7bitAddress(I2C1, LSM303D_ADDR, I2C_Direction_Transmitter);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
			!= SUCCESS)
		;

	I2C_SendData(I2C1, 0x80 | reg); // dlaczego dodajemy 0x80?
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS)
		;
}

void lsm_write(uint8_t reg, const void* data, int size) {
	int i;
	const uint8_t* buffer = (uint8_t*) data;

	lsm_set_reg(reg);
	for (i = 0; i < size; i++) {
		I2C_SendData(I2C1, buffer[i]);
		while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING)
				!= SUCCESS)
			;
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
}

//				ktory rejestr czytasz, jaka dana, jak duzy
//czyta 0x0f, bo to jest rejestr WHO_AM_I
void lsm_read(uint8_t reg, void* data, int size) {
	int i;
	uint8_t* buffer = (uint8_t*) data;

	lsm_set_reg(reg); // !

	I2C_GenerateSTART(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
		;

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_Send7bitAddress(I2C1, LSM303D_ADDR, I2C_Direction_Receiver);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
			!= SUCCESS)
		;

	for (i = 0; i < size - 1; i++) {
		while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS)
			;
		buffer[i] = I2C_ReceiveData(I2C1);
	}
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS)
		;
	buffer[i] = I2C_ReceiveData(I2C1);
}
