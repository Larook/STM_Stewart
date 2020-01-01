/*------------TO DO:6

 -OK poruszanie srodka po okregu na doublach
 -OK dodac polozenie Z do okregu
 -dodac wyswietlacz I2c
 -ulozyc lepiej przewody
 -zbadac RPY, dlaczego srodek nie zostaje w miejscu

 */

#include <fastmath.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "main.h"
#include "usart2.h"  // dodanie komunikacji po usarcie
#include "lsm303d.h"  // podlaczanie i odczytywanie akcelerometru
#include "enable_things.h"  // wlaczenie peryferiow
#include "servos_platform.h"  // sterowanie serwami
#include "kinematics.h"  // stale mechaniczne budowy platformy i obliczanie katow serw dla zadanych polozen
#include "lcd_i2c.h"
#include "touchpanel.h"
#include "PID.h"

volatile uint32_t timer_ms = 0;

/* GLOBALNE WSKAZNIKI DO STRUKTUR*/
struct sEnvironment* ptr_env = &env; // wskaznik do adresu environment

struct sTouchPanel* ptr_touchPanel = &touchPanel;

struct sPID_controller* ptr_PIDx = &PIDx;
//int x_ref = 100;

struct sPID_controller* ptr_PIDy = &PIDy;

int8_t prevState = 1; // najpierw byl wylaczony
int8_t ball_on_plate = 0;

void SysTick_Handler() {
	if (timer_ms)
		timer_ms--;
}

void delay_ms(int time) {
	timer_ms = time;
	while (timer_ms)
		;
}

double degToRad(double degree) {
	return degree * 57.297;
}

int AngleToTicks(double angle) {
	/*funkcja zamienia zadany kat do doswiadczalne wyznaczonej liczby Tickow*/
//prosta dobrze wyrownuje 90-180 STOPNI, wykres w matlabie
	float a = 13.19; //a prostej
	float b = 115; //b prostej
	int ticks = a * angle + b; //prosta
	return ticks;
}

int main(void) {
	/*
	 * ZEBY ODCZYTAC PRAWIDLOWO WSP X I Y MUSZE ZMIENIC GPIO.MODE NA PINACH
	 * DZIALA! W PRZERWANIACH!!!
	 * Teraz trzeba implementowac PID
	 * a takze szukac w necie jak wykorzystac dokladnie PID do sterowania RPY
	 */

	SysTick_Config(SystemCoreClock / 1000);

	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
					| RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

	init_ADC_DMA();
	init_timers_for_6servos();
	init_ButtonInterrupt();
	init_USART2();
	init_I2C();

	printf("Start PRORGAMU \n\r");
	printf(
			"Kalibracja panelu dotykowego na podstawie zmierzonych wczesniej wspolrzednych");
	delay_ms(100);

//	init_touchPointsCalibration(&touchPanel); // kalibruje panel i wyswietla jego parametry
	setTouchPanelCalibration(&touchPanel, 0.00, 4.32, -8765.75, 3.48, 0,
			-7176.76); // ale nadal korzystam ze wsp obliczonych w pythonie
	delay_ms(100);

	printf("Po inilizacji peryferiow \n\r");
	printf("Ustawiam defaulty IMU \n\r");
	set_default_Magnetometer();
	set_default_Accelerometer();
	check_i2c_LSM303D();

//	lcd_send_4bit(0xff); // zapal
//	lcd_send_4bit(0x00); // zgas
	printf("po wyslaniu \n\r");

	// stale pozwalajace ustawic platforme w rzeczywistym poziomie
	ptr_env->roll_level_bias = 1.4;
	ptr_env->pitch_level_bias = -0.6;

	delay_ms(200);

//	Proba z regulatorem PD
//	set_PID_params(&PIDx, 1100, -1100, 0.00000001, 0, 0.000019); calkiem niezle
//	set_PID_params(&PIDx, 1100, -1100, 0.0000020, 1820, 0.000019); lepiej
//	set_PID_params(&PIDx, 1100, -1100, 0.0000013, 0xde, 0.000020); jeszcze lepiej
//	set_PID_params(&PIDx, 1100, -1100, 0.0000013, 0xaefe, 0.000018); //konstruktor do regulatora PID osi X D=0.0001
//	set_PID_params(&PIDx, 1100, -1100, 0.0075, 0.019, 0.015); //po zmianie na kod bez Ts
//	set_PID_params(&PIDx, 1100, -1100, 0.0021, 0.036, 0.01); //po zmianie na kod bez Ts
	set_PID_params(&PIDx, 1100, -1100, 0.0022, 0.047, 0.012); //po zmianie na kod bez Ts
	set_PID_params(&PIDy, 1100, -1100, 0.0022, 0.047, 0.012); //po zmianie na kod bez Ts

//	Niby ok, ale nadal jest spore opoznienie, spory lag - albo duzo szybciej przerwania, albo zmienic podejscie troche
//	set_PID_params(&PIDx, 1100, -1100, 0.2, 1.8, 0.3); //konstruktor do regulatora PID osi X
//	set_PID_params(&PIDy, 1100, -1100, 0.2, 1.8, 0.3); //konstruktor do regulatora PID osi Y

// Raczej zle - kulka w osi y duzo szybciej niz totalnie wolna w osi x
//	set_PID_params(&PIDx, 1000, -1000, 0.2, 0.018, 0); //konstruktor do regulatora PID osi X
//		set_PID_params(&PIDy, 100, -100, 1.5, 0, 0); //konstruktor do regulatora PID osi Y

	init_timer_touch(); // wlaczenie przerwan do sczytywania danych z sensorow
	//------------------------------

	while (1) {
//		moveCircle(20, 7, env); // nowe
//		moveCircle(20, 7, env); // nowe
//
//		moveZ(20, 7, env);
//		moveZ(20, 7, env);
//
//		moveY(20, 7, env);
//		moveY(20, 7, env);
//
//		moveRP(8, 1);
//		moveRP(8, 1);
//
//		moveCircleZ(10, 7, env);
//		moveCircleZ(10, 7, env);
//		moveCircleZ(10, 7, env);
//		moveCircleZ(10, 7, env);
//
//		moveWave(10, 10, env);
//		moveWave(10, 10, env);
//		moveWave(10, 10, env);
//		moveWave(10, 10, env);
//		delay_ms(1000);
	}

}

// -------------------- Przerwanie do sczytanie wsp panelu dotykowego -----------
void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5)) {
			/* odczytaj X i przygotuj do Y */

			ptr_env->X_TouchPanel = adc_value[3]; // black read		getX_touchPanel()

			GPIO_ResetBits(GPIOA, GPIO_Pin_5);

			//reset
			GPIO_StructInit(&gpio);
			gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
			gpio.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_Init(GPIOC, &gpio);
			GPIO_ResetBits(GPIOC, GPIO_Pin_1); // black GND
			GPIO_SetBits(GPIOC, GPIO_Pin_3); // red Vcc

			GPIO_StructInit(&gpio);
			gpio.GPIO_Pin = GPIO_Pin_2;
			gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_Init(GPIOC, &gpio); // white float

			gpio.GPIO_Pin = GPIO_Pin_0;
			gpio.GPIO_Mode = GPIO_Mode_AIN;
			gpio.GPIO_Speed = GPIO_Speed_2MHz;
			GPIO_Init(GPIOC, &gpio); // blue read

			ptr_env->PlatformX = -1 * (adc_value[0] - 2048) * 25 / 2048; // getXJoystick
			ptr_env->PlatformY = -1 * (adc_value[1] - 2088) * 25 / 2048; // getYJoystick
			ptr_env->PlatformZ = -11 - (adc_value[2] * 0.06 - 40); // getZPotentiometer

			ptr_env->Roll = -1 * getRollIMU(); //cos jakby troche nietak
			ptr_env->Pitch = -1 * getPitchIMU(); //-8 do 8
			ptr_env->Yaw = 0; //-1 * (getYawIMU() + 17); //-5 do 5

		} else {
			/* odczyt Y i przygotuj do X */

			ptr_env->Y_TouchPanel = adc_value[4]; // blue read		getY_touchPanel()
			GPIO_SetBits(GPIOA, GPIO_Pin_5);

			//przygotuj do X
			GPIO_StructInit(&gpio);
			gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
			gpio.GPIO_Mode = GPIO_Mode_Out_PP;
			GPIO_Init(GPIOC, &gpio);
			GPIO_SetBits(GPIOC, GPIO_Pin_0); // blue Vcc
			GPIO_ResetBits(GPIOC, GPIO_Pin_2); // white GND

			GPIO_StructInit(&gpio);
			gpio.GPIO_Pin = GPIO_Pin_3;
			gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_Init(GPIOC, &gpio);  // red float

			gpio.GPIO_Pin = GPIO_Pin_1;
			gpio.GPIO_Mode = GPIO_Mode_AIN;
			gpio.GPIO_Speed = GPIO_Speed_2MHz;
			GPIO_Init(GPIOC, &gpio); // black read

			// zmienna wskaznikowa pod adres funkcji
			int16_t* real_ptr = getPtrRealTouchArray(ptr_env->X_TouchPanel,
					ptr_env->Y_TouchPanel, ptr_touchPanel);

			// przypisz tablice z pomiarami do sEnvironment
			ptr_env->X_Real = -1 * real_ptr[0];
			ptr_env->Y_Real = real_ptr[1];

//			printf(
//					"X: sense = %d   real = %f\t\t Y: sense = %d   real = %f\n\r",
//					ptr_env->X_TouchPanel, ptr_env->X_Real,
//					ptr_env->Y_TouchPanel, ptr_env->Y_Real);

			/* Jesli wcisniety przycisk - rob balans pilki */
			if (!ball_on_plate) {

				/* TESTOWANIE PID: 1-error[x], 2-PID, 3-PIDout->angle, 4-angle->move*/
				PIDx.e_in = ptr_env->PlatformX - ptr_env->X_Real; // joystick - zmierzone
				PIDy.e_in = ptr_env->PlatformY - ptr_env->Y_Real; // joystick - zmierzone

//				PIDx.x_in = 100 - PIDx.y_o
				get_PID_output(&PIDx, 0.05); // ms przerwania niby, ale nie ma porownania
				get_PID_output(&PIDy, 0.05);

				//			ptr_env->next_angle_Pitch = -1 * get_angle_from_PID_output(&PIDx, 0.7,
				//								50.0) * 57.3 + ptr_env -> pitch_level_bias; // osia X steruje Pitch   5.3
				//						ptr_env->next_angle_Roll = get_angle_from_PID_output(&PIDy, 0.7, 50.0) * 57.3 + ptr_env -> roll_level_bias; // osia Y steruje Roll 0.45

				// 0.05s = 50ms = okres przerwan

				ptr_env->next_angle_Pitch = -1
						* get_angle_from_PID_output(&PIDx, 35, 0.05) * 57.3; // osia X steruje Pitch   5.3   0,169721656 bylo 9.71 a potem 9.48, a potem 35

				ptr_env->next_angle_Roll = get_angle_from_PID_output(&PIDy, 35,
						0.05) * 57.3; // osia Y steruje Roll 0.45   0,169721656

				//zrob ruch - najlepiej ze struktury, ktora sie updateuje w przerwaniach
				movePlatformFromTranslation_RPY(0, 0, ptr_env->PlatformZ,
						ptr_env->next_angle_Roll + ptr_env->roll_level_bias,
						ptr_env->next_angle_Pitch + ptr_env->pitch_level_bias,
						0);

				//			movePlatformFromTranslation_RPY(0, 0, ptr_env->PlatformZ, // levelowanie dobrego poziomu platformy
				//					ptr_env->roll_level_bias, ptr_env->pitch_level_bias, 0);
//				printf(
//						"\nX = %d   \t Y = %d   \t Z = %d   \t Xpanel_r = %f   \t Ypanel_r = %f   \t\t Roll = %f, error_r = %f   \t Pitch = %f, error_p = %f   \n\r",
//						ptr_env->PlatformX, ptr_env->PlatformY,
//						ptr_env->PlatformZ, ptr_env->X_Real, ptr_env->Y_Real,
//						ptr_env->next_angle_Roll, PIDx.e_in, ptr_env->next_angle_Pitch, PIDy.e_in);

				printf("X: yp=%.3f \t yi=%.3f \t yd=%.5f \n\r",PIDx.yp, PIDx.yi, PIDx.yd);
//				printf(
//						"Z = %d   \t Xpanel_r = %f   \t Ypanel_r = %f   \t\t Roll = %f, error_r = %.0f   \t Pitch = %f, error_p = %.0f   \n\r",
//						ptr_env->PlatformZ, ptr_env->X_Real, ptr_env->Y_Real,
//						ptr_env->next_angle_Roll, PIDx.e_in,
//						ptr_env->next_angle_Pitch, PIDy.e_in);
			}

			/* Jesli przycisk nie wcisniety - rob ruch na podstawie joysticka*/
			else {
				movePlatformFromTranslation_RPY(ptr_env->PlatformX,
						ptr_env->PlatformY, ptr_env->PlatformZ,
						ptr_env->Roll + ptr_env->roll_level_bias,
						ptr_env->Pitch + ptr_env->pitch_level_bias, 0);

//				movePlatformFromTranslation_RPY(ptr_env->PlatformX,
//						ptr_env->PlatformY, ptr_env->PlatformZ,
//						ptr_env->roll_level_bias, ptr_env->pitch_level_bias, 0);

//				movePlatformFromTranslation_RPY(ptr_env->PlatformX, ptr_env->PlatformY, ptr_env->PlatformZ,
//										0, 0, 0);
				printf(
						"\nX = %d   \t Y = %d   \t Z = %d   \t Xpanel_r = %f   \t Ypanel_r = %f   \t\t Roll = %f   \t Pitch = %f  \n\r",
						ptr_env->PlatformX, ptr_env->PlatformY,
						ptr_env->PlatformZ, ptr_env->X_Real, ptr_env->Y_Real,
						ptr_env->Roll, ptr_env->Pitch);
			}

//			printf(
//					"\nX = %d   \t Y = %d   \t Z = %d   \t Xpanel_r = %f   \t Ypanel_r = %f   \t\t Roll = %d   \t Pitch = %d   \t Yaw = %d\n\r",
//					ptr_env->PlatformX, ptr_env->PlatformY, ptr_env->PlatformZ,
//					ptr_env->X_Real, ptr_env->Y_Real, ptr_env->Roll,
//					ptr_env->Pitch, ptr_env->Yaw);

		}
	}
}

//--------------------- Przerwanie od przycisku ----------------
void EXTI15_10_IRQHandler() {
	if (EXTI_GetITStatus(EXTI_Line10)) {
//		int X_TouchPanel = getX_touchPanel();
//		int Y_TouchPanel = abs(getY_touchPanel() - 4095); //getY_touchPanel();//abs(getY_touchPanel()-4095);
//
//		printf("X_TouchPanel %d 	Y_TouchPanel %d", X_TouchPanel, Y_TouchPanel);

		//wykonuj program w zaleznosci od przycisku
		if ((GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10) == 0)
				& (prevState % 2 == 0)) {
//			delay_ms(50);
			prevState++;
			GPIO_SetBits(GPIOA, GPIO_Pin_12);
			ball_on_plate = 1; // wcisniecie przycisku -> program z RPY

		} else if ((GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10) == 0)
				& (prevState % 2 == 1)) { //} & (isPressed == 1) ){
//			delay_ms(50);
			prevState--;
			GPIO_ResetBits(GPIOA, GPIO_Pin_12);
			ball_on_plate = 0; //domyslnie program bez RPY
		}

		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}

void check_lcd() {
	//Table 12 4-Bit Operation, 8-Digit � 1-Line Display Example with Internal Reset
	// zamiast H wyszly 2 linie zapalone - trzeba przesledzic gdzie wychodza sciezki z i2c na disp
	// step 2
	lcd_send_4bit(0x02);
	delay_ms(40);
	// step 3
	lcd_send_two_4bit(0x02, 0x03);
	delay_ms(40);
	// step 4
	lcd_send_two_4bit(0x00, 0x0e);
	delay_ms(40);
	// step 5
	lcd_send_two_4bit(0x00, 0x07);
	delay_ms(40);
	// step 6 (data - H)
	lcd_send_two_4bit(0x24, 0x28);
	delay_ms(40);

//	while (1){
//		lcd_set_reg(toWrite);
//		delay_ms(300);
//		printf("%x \n",toWrite);
//		toWrite ++;
//	}
//	while(1){
//		int input =
//		lcd_set_reg(input);
//	}
//	for (int i = 0; i<5; i++){
//		lcd_set_reg(DISP_ON_CURSOR_OFF);
////		lcd_send_data(10);
//		delay_ms(1000);
//		lcd_set_reg(DISP_OFF_CURSOR_OFF);
//		delay_ms(1000);
//	}
}

