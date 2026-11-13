/*
 * INA219.h
 *
 *  Created on: Nov 13, 2026
 *      Author: Asus
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "main.h"

#define INA219_ADDRESS  0x80

void INA219_Init(I2C_HandleTypeDef *hi2c);
float INA219_GetBusVoltage_V(void);
float INA219_GetCurrent_mA(void);

#endif /* INC_INA219_H_ */
