/*
 * lsm303d.h
 *
 *  Created on: 08.02.2019
 *      Author: PiK
 */

#ifndef LSM303D_H_
#define LSM303D_H_

#include <stdint.h>

#define LSM303D_TEMP_OUT			0x05
#define LSM303D_STATUS_M			0x07
#define LSM303D_OUT_X_M				0x08
#define LSM303D_OUT_X_M_H			0x09
#define LSM303D_OUT_Y_M				0x0a
#define LSM303D_OUT_Y_M_H			0x0B
#define LSM303D_OUT_Z_M				0x0c
#define LSM303D_OUT_Z_M_H			0x0D
#define LSM303D_WHO_AM_I			0x0f
#define LSM303D_CTRL0				0x1f
#define LSM303D_CTRL1				0x20
#define LSM303D_CTRL2				0x21
#define LSM303D_CTRL3				0x22
#define LSM303D_CTRL4				0x23
#define LSM303D_CTRL5				0x24
#define LSM303D_CTRL6				0x25
#define LSM303D_CTRL7				0x26
#define LSM303D_STATUS				0x27
#define LSM303D_OUT_X_A				0x28
#define LSM303D_OUT_Y_A				0x2a
#define LSM303D_OUT_Z_A 			0x2c
#define LSM303D_IG_CFG1				0x30
#define LMS303D_INT_CTRL_M 			0x12
#define LMS303D_OFFSET_X_L_M 		0x16
#define LMS303D_OFFSET_Y_L_M 		0x18
#define LMS303D_OFFSET_Z_L_M 		0x1A

//funkcje czyt zap
extern void lsm_write(uint8_t reg, const void* data, int size);
extern void lsm_read(uint8_t reg, void* data, int size);

extern void lsm_write_reg(uint8_t reg, uint8_t value);
extern uint8_t lsm_read_reg(uint8_t reg);
extern int16_t lsm_read_value(uint8_t reg);

float getRollIMU(); //cos jakby troche nietak
float getPitchIMU(); //-8 do 8
float getYawIMU(); //-5 do 5

#endif /* LSM303D_H_ */
