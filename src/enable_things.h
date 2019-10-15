/*
 * enable_things.h
 *
 *  Created on: 13.02.2019
 *      Author: PiK
 */

#ifndef ENABLE_THINGS_H_
#define ENABLE_THINGS_H_

#define ADC_CHANNELS	3

uint16_t adc_value[ADC_CHANNELS]; //tablica z wynikami adc z DMA

void init_ADC_DMA();
void init_timers_for_6servos();
void init_ButtonInterrupt();
void init_USART2();
void init_I2C();

void set_default_Magnetometer();
void set_default_Accelerometer();
void check_i2c_LSM303D();


#endif /* ENABLE_THINGS_H_ */
