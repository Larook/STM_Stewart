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

volatile uint32_t timer_ms = 0;
struct sEnvironment* env_pointer = &env; // wskaznik do adresu environment
struct sTouchPanel* panel_pointer = &touchPanel;

int8_t prevState = 1; // najpierw byl wylaczony
int8_t doRPY = 0;

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
	setTouchPanelCalibration(&touchPanel, 0.00, 4.32, -8765.75, 3.48, 0, -7176.76); // ale nadal korzystam ze wsp obliczonych w pythonie
	delay_ms(100);

	printf("Po inilizacji peryferiow \n\r");
	printf("Ustawiam defaulty IMU \n\r");
	set_default_Magnetometer();
	set_default_Accelerometer();
	check_i2c_LSM303D();

//	lcd_send_4bit(0xff); // zapal
//	lcd_send_4bit(0x00); // zgas
	printf("po wyslaniu \n\r");

	delay_ms(200);
	init_timer_touch(); // wlaczenie przerwan do sczytywania danych z sensorow
	//------------------------------

	while (1) {
		/*dodac potencjometry ktore by sterowaly wysokoscia platformy */
		/* ogarnac funkcje ktora zwieksza pos o jakas wartosc w gore lub w dol	*/
		// mozna sprobowac na rejestrach zmieniac piny, zeby bylo szybciej
		moveCircle(6, 3, env); // nowe
//		delay_ms(7);
	}

}

// -------------------- Przerwanie do sczytanie wsp panelu dotykowego -----------
void TIM2_IRQHandler() {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5)) {
			/* odczytaj X i przygotuj do Y */

			env_pointer->X_TouchPanel = adc_value[3]; // black read		getX_touchPanel()

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

			env_pointer->PlatformX = -1 * (adc_value[0] - 2048) * 25 / 2048; // getXJoystick
			env_pointer->PlatformY = -1 * (adc_value[1] - 2088) * 25 / 2048; // getYJoystick
			env_pointer->PlatformZ = -11 - (adc_value[2] * 0.06 - 40); // getZPotentiometer

			env_pointer->Roll = -1 * getRollIMU(); //cos jakby troche nietak
			env_pointer->Pitch = -1 * getPitchIMU(); //-8 do 8
			env_pointer->Yaw = 0; //-1 * (getYawIMU() + 17); //-5 do 5

		} else {
			/* odczyt Y i przygotuj do X */

			env_pointer->Y_TouchPanel = adc_value[4];  // blue read		getY_touchPanel()
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
			uint8_t* real_ptr = getPtrRealTouchArray(env_pointer->X_TouchPanel,
					env_pointer->Y_TouchPanel, panel_pointer);

			// przypisz tablice z pomiarami do sEnvironment
			env_pointer->X_Real = -1 * real_ptr[0];
			env_pointer->Y_Real = real_ptr[1];

//			printf("Xpanel_r = %d\t\t Ypanel_r = %d\n\r", env_pointer->X_Real,
//					env_pointer->Y_Real);

			printf(
					"\nX = %d   \t Y = %d   \t Z = %d   \t Xpanel_r = %d   \t Ypanel_r = %d   \t\t Roll = %d   \t Pitch = %d   \t Yaw = %d\n\r",
					env_pointer->PlatformX, env_pointer->PlatformY,
					env_pointer->PlatformZ, env_pointer->X_Real,
					env_pointer->Y_Real, env_pointer->Roll, env_pointer->Pitch,
					env_pointer->Yaw);

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
			doRPY = 1; // wcisniecie przycisku -> program z RPY

		} else if ((GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10) == 0)
				& (prevState % 2 == 1)) { //} & (isPressed == 1) ){
//			delay_ms(50);
			prevState--;
			GPIO_ResetBits(GPIOA, GPIO_Pin_12);
			doRPY = 0; //domyslnie program bez RPY
		}

		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}

//---------------------- Odczyty z przetwornikow --------------
//int getXJoystick(int max) {
//	int Vx = (adc_value[0] - 2048) * max / 2048; // dla polozenia normalnego zwraca 0
//	return Vx;
//}
//int getYJoystick(int max) {
//	int Vy = (adc_value[1] - 2088) * max / 2048;
//	return Vy;
//}
//int getZPotentiometer() { //normalnie daje adc od 0 do 880
//	int Vz = adc_value[2] * 0.06 - 40; // teraz od -40 do 12
//	return Vz;
//}
//
//int getX_touchPanel() { // PC1
//	int Vx = adc_value[3];
//	return Vx;
//}
//
//int getY_touchPanel() { // PC0
//	int Vy = adc_value[4];
//	return Vy;
//}


void check_lcd() {
	//Table 12 4-Bit Operation, 8-Digit ´ 1-Line Display Example with Internal Reset
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

