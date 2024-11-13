/*
 * mpu6050.h
 *
 *  Created on: May 20, 2023
 *      Author: TOLGA
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "main.h"


#define MPU6050           0x68
#define MPU6050_ADDR       MPU6050 << 1
#define whoAmIReg         0x75 // to read 0x68 is exist or not
#define powerManagmentReg 0x6B
#define sampleRateDiv 	  0x19 // sampleRate = gyroRate / (1 + sampleDiv)
#define gyroConf		  0x1B
#define accelConf		  0x1C
#define accelMeasure      0x3B
#define gyroMeasure       0x43

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim3;

typedef enum{
	degS250  = 0,
	degS500  = 1,
	degS1000 = 2,
	degS2000 = 3
}gyroScale_t;

typedef enum{
	g2  = 0,
	g4  = 1,
	g8  = 2,
	g16 = 3
}accelScale_t;


void MPU6050_Read_Temperature(void);
void MPU6050Config(void);
void MPU6050AccelRead(void);
void MPU6050GyroAverage(void);
void MPU6050AccelScale(accelScale_t scale);
void MPU6050GyroScale(gyroScale_t scale);
void MPU6050powerOn(void);
void MPU6050Init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


































#endif /* INC_MPU6050_H_ */
