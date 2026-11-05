/*
 * gps.h
 *
 *  Created on: Sep 28, 2026
 *      Author: Asus
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define GPS_DMA_BUFFER_SIZE       512U
#define GPS_RING_BUFFER_SIZE      2048U
#define GPS_LINE_BUFFER_SIZE      160U


/* =========================
 * GPS DATA STRUCTURE
 * ========================= */
typedef struct
{
    double latitude;
    double longitude;

    float altitude;
    float geoid_height;

    uint8_t fix;
    uint8_t satellites;

    float hdop;

    float speed_knots;
    float speed_kmh;
    float speed_mps;

    float course_deg;

    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    uint8_t day;
    uint8_t month;
    uint8_t year;

    bool position_valid;
    bool data_valid;

} gps_data_t;


/* =========================
 * GLOBAL GPS DATA
 * ========================= */

/* Cho phép Live Expressions nhìn thấy biến này */
extern gps_data_t gps_data;


/* =========================
 * FUNCTIONS
 * ========================= */

HAL_StatusTypeDef gps_init(UART_HandleTypeDef *huart);

void gps_process(void);

gps_data_t *gps_get_data(void);

double gps_nmea_to_degrees(double nmea_value);


/* HAL UART callback */
void GPS_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* INC_GPS_H_ */
