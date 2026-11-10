/*
 * BNO055.h
 *
 *  Created on: Jan 30, 2026
 *      Author: LE CONG TRUONG
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "stm32f4xx_hal.h"

// ===== I2C =====
extern I2C_HandleTypeDef hi2c1;
#define BNO055_I2C &hi2c1
#define BNO055_ADDR (0x28 << 1)

// ===== Register =====
#define BNO055_CHIP_ID      0x00
#define BNO055_OPR_MODE     0x3D
#define BNO055_PWR_MODE     0x3E
#define BNO055_SYS_TRIGGER  0x3F
#define BNO055_UNIT_SEL     0x3B

#define BNO055_EULER_H_LSB  0x1A
#define BNO055_GYRO_DATA_X_LSB  0x14
#define BNO055_ACCEL_DATA_X_LSB 0x08

// ===== Mode =====
#define CONFIG_MODE 0x00
#define NDOF_MODE   0x0C

#define BNO055_CALIB_STAT   0x35  // Thanh ghi kiểm tra trạng thái hiệu chuẩn
#define BNO055_CALIB_DATA   0x55  // Bắt đầu 22 byte dữ liệu hiệu chuẩn

#define BNO055_CALIB_DATA   0x55

void BNO055_WriteCalibProfile(uint8_t *buffer);
uint8_t BNO055_GetCalibStatus(void);
void BNO055_ReadCalibProfile(uint8_t *buffer);
void BNO055_WriteCalibProfile(uint8_t *buffer);
void BNO055_Init(void);
void BNO055_ReadEuler(float *yaw);
void BNO055_ReadGyro(float *gx, float *gy, float *gz);
void BNO055_ReadAccel(float *ax, float *ay, float *az);


#endif /* INC_BNO055_H_ */
