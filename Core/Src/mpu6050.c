 /*
 * mpu6050.c
 *
 *  Created on: May 20, 2023
 *      Author: TOLGA
 */

#include "mpu6050.h"
uint8_t whoAreYou;
uint8_t MemData;
uint8_t buffer[2], tuffer[6], cuffer[6];   // It holds the raw data.
int16_t gyro_raw[3] , acc_raw[3];
uint32_t elapsed_time = 0; // zamanı izlemek için kullanılır
bool is_angle_low = false;  // açının düşük olup olmadığını izlemek için kullanılır
// It holds the Actual Result.
float gyro_cal[3];
int16_t acc_total_vector;
float angle_pitch_gyro , angle_roll_gyro;
float angle_pitch_acc, angle_roll_acc;
float angle_pitch , angle_roll;
uint32_t low_angle_start_time;
int16_t raw_temp;
float temp;
int i ;
float prevtime , prevtime1 , time1, elapsedtime1 , prevtime2 , time2 , elapsedtime2;

float gyro_X;
float gyro_Y;
float gyro_Z;

HAL_StatusTypeDef set_gyro;

void MPU6050Init(void){
	HAL_I2C_Mem_Read(
				&hi2c1,
				MPU6050_ADDR,
				whoAmIReg,
				1,
				&whoAreYou,
				1,
				HAL_MAX_DELAY
				);
}



void MPU6050powerOn(void){
	MemData = 0x00;
	HAL_I2C_Mem_Write(
			&hi2c1,
			MPU6050_ADDR,
			powerManagmentReg,
			1,
			&MemData,
			1,
			HAL_MAX_DELAY
			);
}

/*void mpu6050Sampling(void){
	MemData = 0x07;
	HAL_I2C_Mem_Write(
			&hi2c1,
			MPU6050_ADDR,
			sampleRateDiv,
			1,
			&MemData,
			1,
			HAL_MAX_DELAY
			);
}
*/

void MPU6050GyroScale(gyroScale_t scale){
	MemData  = 0x00 | (scale << 3);
		HAL_I2C_Mem_Write(
			&hi2c1,
			MPU6050_ADDR,
			gyroConf,
			1,
			&MemData,
			1,
			HAL_MAX_DELAY
			);
}

void MPU6050AccelScale(accelScale_t scale){
	MemData = 0x00 | (scale << 3);
		HAL_I2C_Mem_Write(
			&hi2c1,
			MPU6050_ADDR,
			accelConf,
			1,
			&MemData,
			1,
			HAL_MAX_DELAY
			);
}

//void mpu6050GyroRead(void)
//{
//	mpu6050GyroAverage();


//}
void MPU6050GyroAverage(void)
{
	for(i = 0; i<2000; i++)
	{
		prevtime2 = time2;
		time2 = HAL_GetTick();
		elapsedtime2 = (time2-prevtime2)*1000;

		cuffer[0] = gyroMeasure; // 0X43
		HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cuffer, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, cuffer, 6, HAL_MAX_DELAY);

		gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);   // X axis
		gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);   // Y axis
		gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);   // Z axis

		gyro_cal[0] += gyro_raw[0];            /*used to hold the addition (showing the x-axis) */
		gyro_cal[1] += gyro_raw[1];            /*used to hold the addition (showing the x-axis) */
		gyro_cal[2] += gyro_raw[2];            /*used to hold the addition (showing the x-axis) */

		HAL_Delay(3);
	}
	gyro_cal[0] /= 2000;                     /* Divide by 2000 to get the average value */
	gyro_cal[1] /= 2000;                     /* Divide by 2000 to get the average value */
	gyro_cal[2] /= 2000;                     /* Divide by 2000 to get the average value */

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);  /*Tested the STM32F407 led to test whether the function works */
	HAL_Delay(1000);

}

void MPU6050AccelRead(void)
{
	prevtime1 = time1;
	time1 = HAL_GetTick();
	elapsedtime1 = (time1 - prevtime1) * 1000;  /*Dummy variable used to control delay */
	tuffer[0] = 0x3B;  //accelMeasure
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, tuffer, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, tuffer, 6, HAL_MAX_DELAY);

	acc_raw[0] = (tuffer[0] << 8 | tuffer[1]);  /*x */
	acc_raw[1] = (tuffer[2] << 8 | tuffer[3]);  /*y */
	acc_raw[2] = (tuffer[4] << 8 | tuffer[5]);  /*z */

	cuffer[0] = 0x43;  // gyroMeasure
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cuffer, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, cuffer, 6, HAL_MAX_DELAY);

	//Gyro Raw Values
	gyro_raw[0] = (cuffer[0] << 8  | cuffer[1] );
	gyro_raw[1] = (cuffer[2] << 8  | cuffer[3] );
	gyro_raw[2] = (cuffer[4] << 8  | cuffer[5] );

	gyro_raw[0] -= gyro_cal[0];
	gyro_raw[1] -= gyro_cal[1];
	gyro_raw[2] -= gyro_cal[2];

	angle_pitch_gyro += gyro_raw[0]  * 0.0000611;
	angle_roll_gyro  += gyro_raw[1]  * 0.0000611;

	angle_pitch_gyro += angle_roll_gyro  * sin(gyro_raw[2] * 0.000001066);
	angle_roll_gyro += angle_pitch_gyro  * sin(gyro_raw[2] * 0.000001066);

	acc_total_vector = sqrt((acc_raw[0]*acc_raw[0])+ (acc_raw[1] * acc_raw[1]) + (acc_raw[2] * acc_raw[2])) ;
	angle_pitch_acc = asin((float)acc_raw[1]/acc_total_vector) * 57.296;  // radians to degrees conversion
	angle_roll_acc = asin((float) acc_raw[0]/acc_total_vector) * -57.296; // radians to degrees conversion
	if (angle_pitch_acc < 65 && angle_pitch_acc >= 25) {
	        if (!is_angle_low) {
	            // açı yeni düşmüşse, zamanlayıcıyı başlatın
	            is_angle_low = true;
	            low_angle_start_time = HAL_GetTick();
	        } else {
	            // açı hala düşükse ve 10 saniye (10000 ms) geçtiyse, buzzer'ı başlatın
	            if (HAL_GetTick() - low_angle_start_time > 10000) {
	                for (i = 0; i < 200; i++) {
	                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	                }
	            }
	            // elapsed_time değişkenini güncelleyin
	            elapsed_time = HAL_GetTick() - low_angle_start_time;
	        }
	    } else {
	        // açı düşük değilse, durumları sıfırlayın
	        is_angle_low = false;
	        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	        // elapsed_time değişkenini sıfırlayın
	        elapsed_time = 0;
	    }

	if(set_gyro)
		{
		  angle_pitch = angle_pitch_gyro * 0.9996 + angle_pitch_acc * 0.0004;  //x axis
		  angle_roll = angle_roll_gyro * 0.9996 + angle_roll_acc * 0.0004;     // y axis
		}
	else
	{
	  angle_pitch = angle_pitch_acc;
	  set_gyro = true;
	}

	while((HAL_GetTick()- prevtime) * 1000 < 4000);
	prevtime = HAL_GetTick();



}

void MPU6050Config(void){
	// is valid Condition true 0x68
	MPU6050Init();
	if(whoAreYou == MPU6050)
	{
	// power on
		MPU6050powerOn();
	// sampling data ratio
		//mpu6050Sampling();
	// gyro scale   (RAW)
		MPU6050GyroScale(degS500);
	// accel scale  (RAW)
		MPU6050AccelScale(g8);
	}
}

void MPU6050_Read_Temperature()
{

buffer[0] = 0x41;
HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buffer, 1, HAL_MAX_DELAY);
HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buffer, 2, HAL_MAX_DELAY);
raw_temp = (buffer[0] << 8 | buffer[1]);
temp = (raw_temp / 340.0)  + 36.53;

}
