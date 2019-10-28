/*------------TO DO:6

 -OK poruszanie srodka po okregu na doublach
 -OK dodac polozenie Z do okregu
 -dodac wyswietlacz I2c
 -ulozyc lepiej przewody
 -zbadac RPY, dlaczego srodek nie zostaje w miejscu

 */

//#include <math.h>
#include <fastmath.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "main.h"  //
#include "usart2.h"  // dodanie komunikacji po usarcie
#include "lsm303d.h"  // podlaczanie i odczytywanie akcelerometru
#include "enable_things.h"  // wlaczenie peryferiow
#include "servos_platform.h"  // sterowanie serwami
#include "kinematics.h"  // stale mechaniczne budowy platformy i obliczanie katow serw dla zadanych polozen
#include "lcd_i2c.h"

volatile uint32_t timer_ms = 0;

int8_t prevState = 1; // najpierw byl wylaczony
int8_t doRPY = 0;
int16_t X_TouchPanel = 0;
int16_t Y_TouchPanel = 0;

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
	//  TO JEST BRANCH "touchscreen_now"
	// Trzeba podlaczyc touchscreen i dodac piny adc, a jak trzeba to channele

	// jak dziala ten IC ktory jest doczepiony - w jaki sposob wysyla informacje do LCD

	//https://forbot.pl/blog/kurs-stm32-8-dma-czyli-bezposredni-dostep-do-pamieci-id8465

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
	printf("Po inilizacji peryferiow \n\r");
	printf("Ustawiam defaulty IMU \n\r");
	set_default_Magnetometer();

	set_default_Accelerometer();

	check_i2c_LSM303D();

//	lcd_send_4bit(0xff); // zapal
//	lcd_send_4bit(0x00); // zgas
	printf("po wyslaniu \n\r");
	delay_ms(200);
	//------------------------------

	while (1) {
		/*dodac potencjometry ktore by sterowaly wysokoscia platformy */
		/* ogarnac funkcje ktora zwieksza pos o jakas wartosc w gore lub w dol	*/

		// pobierz dane
		int PlatformX = -1 * getXJoystick(25);
		int PlatformY = -1 * getYJoystick(25);
		int PlatformZ = -11 - getZPotentiometer(); //getZPotentiometer();
//		moveCircle(6, 3, PlatformX, PlatformY, PlatformZ);

		int Roll = -1 * getRollIMU(); //cos jakby troche nietak
		int Pitch = -1 * getPitchIMU(); //-8 do 8
		int Yaw = 0; //-1 * (getYawIMU() + 17); //-5 do 5

//		delay_ms(4);
		X_TouchPanel = getX_touchPanel();
//		delay_ms(4);
		Y_TouchPanel = getY_touchPanel();//abs(getY_touchPanel()-4095);

		printf(
				"X = %d   \t Y = %d   \t Z = %d   \t Xpanel = %5d   \t Ypanel = %5d   \t\t Roll = %d   \t Pitch = %d   \t Yaw = %d\n\r",
				PlatformX, PlatformY, PlatformZ, X_TouchPanel, Y_TouchPanel,
				Roll, Pitch, Yaw);

//		while (1) {
//		moveCircle(6, 3,PlatformX,PlatformY PlatformZ);
//
//	}

		/*
		 // bez RPY
		 if (doRPY == 0) {
		 movePlatformFromTranslation(PlatformX, PlatformY, PlatformZ);
		 // z RPY
		 } else {
		 // bylo Roll * 12.0 / 90.0, Pitch * 12.0 / 90.0, Yaw * 20.0 / 90.0);
		 movePlatformFromTranslation_RPY(PlatformX, PlatformY, PlatformZ,
		 Roll * 70.0 / 90.0, Pitch * 70.0 / 90.0, Yaw=0);
		 } //
		 */

		delay_ms(7);
	}

}
//---------------------- Odczyty z przetwornikow --------------

int getXJoystick(int max) {
	int Vx = (adc_value[0] - 2048) * max / 2048; // dla polozenia normalnego zwraca 0
	return Vx;
}
int getYJoystick(int max) {
	int Vy = (adc_value[1] - 2088) * max / 2048;
	return Vy;
}
int getZPotentiometer() { //normalnie daje adc od 0 do 880
	int Vz = adc_value[2] * 0.06 - 40; // teraz od -40 do 12
	return Vz;
}

int getX_touchPanel() { // PC0
	int Vx = adc_value[3];
	return Vx;
}

int getY_touchPanel() { // PC1
	int Vy = adc_value[4];
	return Vy;
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
