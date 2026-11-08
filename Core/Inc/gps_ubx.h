/*
 * gps_ubx.h
 *
 *  Created on: Nov 8, 2026
 *      Author: Asus
 */

#ifndef INC_GPS_UBX_H_
#define INC_GPS_UBX_H_

#include <stdint.h>
#include <stdbool.h>

#pragma pack(push, 1)
typedef struct {
    uint32_t iTOW;      // Thời gian GPS (ms)
    uint16_t year;      // Năm (UTC)
    uint8_t month;      // Tháng
    uint8_t day;        // Ngày
    uint8_t hour;       // Giờ
    uint8_t min;        // Phút
    uint8_t sec;        // Giây
    uint8_t valid;      // Cờ hợp lệ
    uint32_t tAcc;      // Độ chính xác thời gian
    int32_t nano;       // Phần nano giây
    uint8_t fixType;    // Trạng thái: 0=No Fix, 2=2D, 3=3D
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;      // Số lượng vệ tinh
    int32_t lon;        // Kinh độ (10^-7 độ)
    int32_t lat;        // Vĩ độ (10^-7 độ)
    int32_t height;     // Chiều cao Elipxoit (mm)
    int32_t hMSL;       // Chiều cao mực nước biển (mm)
    uint32_t hAcc;      // Độ chính xác mặt phẳng ngang (mm)
    uint32_t vAcc;      // Độ chính xác chiều dọc (mm)
    int32_t velN;       // Vận tốc trục Bắc (mm/s)
    int32_t velE;       // Vận tốc trục Đông (mm/s)
    int32_t velD;       // Vận tốc trục Xuống (mm/s)
    int32_t gSpeed;     // Tốc độ mặt đất (mm/s)
    int32_t headMot;    // Góc hướng di chuyển (10^-5 độ)
    uint32_t sAcc;      // Độ chính xác vận tốc (mm/s)
    uint32_t headAcc;   // Độ chính xác góc hướng (10^-5 độ)
    uint16_t pDOP;      // Hệ số pha loãng vị trí (0.01)
    uint8_t flags3;
    uint8_t reserved1[5];
    int32_t headVeh;    // Góc hướng mũi xe
    int16_t magDec;
    uint16_t magAcc;
} __attribute__((packed)) UBX_NAV_PVT_t;
#pragma pack(pop)

extern UBX_NAV_PVT_t gps_pvt_data;
extern double ugv_lat;       // Vĩ độ thực tế
extern double ugv_lon;       // Kinh độ thực tế
extern float ugv_speed;     // Vận tốc (m/s)
extern float ugv_heading;   // Góc hướng đầu xe (độ)
extern uint8_t ugv_sat_num; // Số lượng vệ tinh
extern uint8_t ugv_fix;     // Trạng thái Fix

void UBX_Init(void);
bool UBX_ProcessByte(uint8_t byte);

#endif /* INC_GPS_UBX_H_ */
