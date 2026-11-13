/*
 * INA219.c
 *
 *  Created on: Nov 13, 2026
 *      Author: Asus
 */


#include "INA219.h"

extern I2C_HandleTypeDef hi2c3;
static I2C_HandleTypeDef *ina219_i2c;

static void INA219_WriteRegister(uint8_t reg, uint16_t value) {
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF;
    data[1] = value & 0xFF;
    HAL_I2C_Mem_Write(ina219_i2c, INA219_ADDRESS, reg, 1, data, 2, 100);
}

static uint16_t INA219_ReadRegister(uint8_t reg) {
    uint8_t data[2];
    HAL_I2C_Mem_Read(ina219_i2c, INA219_ADDRESS, reg, 1, data, 2, 100);
    return ((data[0] << 8) | data[1]);
}

void INA219_Init(I2C_HandleTypeDef *hi2c) {
    ina219_i2c = hi2c;

    INA219_WriteRegister(0x05, 4096);

    uint16_t config = 0x399F;
    INA219_WriteRegister(0x00, config);
}

float INA219_GetBusVoltage_V(void) {
    uint16_t value = INA219_ReadRegister(0x02);
    return ((value >> 3) * 4) * 0.001f;
}

float INA219_GetCurrent_mA(void) {
    int16_t value = (int16_t)INA219_ReadRegister(0x04);
    return (float)value / 10.0f;
}
