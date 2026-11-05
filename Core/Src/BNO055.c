/*
 * BNO055.c
 *
 *  Created on: Jan 30, 2026
 *      Author: LE CONG TRUONG
 */


#include "BNO055.h"

typedef struct {
    float value;
    float alpha;
} LPF_t;

static LPF_t lpf_gx;
static LPF_t lpf_gy;
static LPF_t lpf_gz;

static void LPF_Init(LPF_t *f, float fc, float dt)
{
    f->value = 0.0f;
    f->alpha = dt / (dt + 1.0f / (2.0f * 3.1415926f * fc));
}

static float LPF_Update(LPF_t *f, float input)
{
    f->value += f->alpha * (input - f->value);
    return f->value;
}

void BNO055_FilterInit(float dt)
{
    float fc_gyro = 40.0f;   // chuẩn drone DIY

    LPF_Init(&lpf_gx, fc_gyro, dt);
    LPF_Init(&lpf_gy, fc_gyro, dt);
    LPF_Init(&lpf_gz, fc_gyro, dt);
}


static void bno055_write(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(BNO055_I2C, BNO055_ADDR, reg,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

static void bno055_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(BNO055_I2C, BNO055_ADDR, reg,
                     I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

void BNO055_ReadEuler(float *yaw)
{
    uint8_t buf[2];
    int16_t raw_h;

    bno055_read(BNO055_EULER_H_LSB, buf, 2);

    raw_h = (int16_t)((buf[1] << 8) | buf[0]); // Heading (Yaw)

    *yaw   = raw_h / 16.0f;
}

void BNO055_ReadGyro(float *gx, float *gy, float *gz)
{
    uint8_t buf[6];
    int16_t raw_x, raw_y, raw_z;

    // Đọc 6 byte gyro
    bno055_read(BNO055_GYRO_DATA_X_LSB, buf, 6);

    // LITTLE-ENDIAN
    raw_x = (int16_t)((buf[1] << 8) | buf[0]);
    raw_y = (int16_t)((buf[3] << 8) | buf[2]);
    raw_z = (int16_t)((buf[5] << 8) | buf[4]);

    // Scale: 1 dps = 16 LSB
    *gx = raw_x / 16.0f;
    *gy = raw_y / 16.0f;
    *gz = raw_z / 16.0f;
//    float gx_raw = raw_x / 16.0f;
//    float gy_raw = raw_y / 16.0f;
//    float gz_raw = raw_z / 16.0f;
//
//    *gx = LPF_Update(&lpf_gx, gx_raw);
//    *gy = LPF_Update(&lpf_gy, gy_raw);
//    *gz = LPF_Update(&lpf_gz, gz_raw);

}


void BNO055_Init(void)
{
    uint8_t id = 0;

    HAL_Delay(700); // chờ sensor ổn định
    bno055_read(BNO055_CHIP_ID, &id, 1);

    if (id != 0xA0) return; // BNO055 ID

    // CONFIG MODE
    bno055_write(BNO055_OPR_MODE, CONFIG_MODE);
    HAL_Delay(30);

    // Normal power
    bno055_write(BNO055_PWR_MODE, 0x00);
    HAL_Delay(10);

    // Unit: degree, dps, m/s2
    bno055_write(BNO055_UNIT_SEL, 0x00);

    // NDOF mode (sensor fusion)
    bno055_write(BNO055_OPR_MODE, NDOF_MODE);
    HAL_Delay(30);
}
