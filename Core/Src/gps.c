/*
 * gps.c
 *
 *  Created on: Sep 28, 2026
 *      Author: Asus
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gps.c
  * @brief   GPS NMEA parser using USART3 + DMA
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


/* Private defines -----------------------------------------------------------*/

#define GPS_RING_MASK   (GPS_RING_BUFFER_SIZE - 1U)


/* Private variables ---------------------------------------------------------*/

/*
 * Global GPS data.
 *
 * IMPORTANT:
 * Do NOT add "static" here.
 * This allows Live Expressions to access:
 *
 *     gps_data.latitude
 *     gps_data.longitude
 *     gps_data.fix
 *     ...
 */
gps_data_t gps_data;

/* DMA buffer */
static uint8_t gps_dma_buffer[GPS_DMA_BUFFER_SIZE];

/* Software ring buffer */
static uint8_t gps_ring_buffer[GPS_RING_BUFFER_SIZE];

static volatile uint16_t gps_ring_head = 0;
static volatile uint16_t gps_ring_tail = 0;

/* NMEA line buffer */
static char gps_line_buffer[GPS_LINE_BUFFER_SIZE];
static uint16_t gps_line_index = 0;

/* UART handle */
static UART_HandleTypeDef *gps_uart = NULL;

/* Last DMA position */
static uint16_t gps_dma_old_pos = 0;


/* Private function prototypes -----------------------------------------------*/

static void gps_ring_push(uint8_t data);
static int gps_ring_pop(uint8_t *data);

static void gps_process_line(char *line);

static void gps_parse_gga(char *sentence);
static void gps_parse_rmc(char *sentence);

static bool gps_check_checksum(char *sentence);

static uint8_t gps_hex_to_byte(char c);

static void gps_update_time(const char *utc);
static void gps_update_date(const char *date);


/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize GPS.
  */
HAL_StatusTypeDef gps_init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    gps_uart = huart;

    /* Clear GPS data */
    memset(&gps_data, 0, sizeof(gps_data));

    /* Clear buffers */
    memset(gps_dma_buffer, 0, sizeof(gps_dma_buffer));
    memset(gps_ring_buffer, 0, sizeof(gps_ring_buffer));
    memset(gps_line_buffer, 0, sizeof(gps_line_buffer));

    gps_ring_head = 0;
    gps_ring_tail = 0;

    gps_line_index = 0;

    gps_dma_old_pos = 0;

    /*
     * Start UART reception using DMA.
     *
     * IMPORTANT:
     * USART3 RX DMA must be configured correctly in CubeMX.
     */
    return HAL_UARTEx_ReceiveToIdle_DMA(
        gps_uart,
        gps_dma_buffer,
        GPS_DMA_BUFFER_SIZE
    );
}


/**
  * @brief  Process received GPS data.
  *
  * Call continuously in main while loop:
  *
  * while (1)
  * {
  *     gps_process();
  * }
  */
void gps_process(void)
{
    uint8_t data;

    while (gps_ring_pop(&data))
    {
        /* Ignore CR */
        if (data == '\r')
        {
            continue;
        }

        /*
         * Wait for beginning of NMEA sentence.
         */
        if (gps_line_index == 0)
        {
            if (data != '$')
            {
                continue;
            }

            gps_line_buffer[gps_line_index++] = '$';
            continue;
        }

        /*
         * Prevent buffer overflow.
         */
        if (gps_line_index >= (GPS_LINE_BUFFER_SIZE - 1U))
        {
            gps_line_index = 0;
            memset(gps_line_buffer, 0, sizeof(gps_line_buffer));

            continue;
        }

        gps_line_buffer[gps_line_index++] = (char)data;

        /*
         * End of NMEA sentence.
         */
        if (data == '\n')
        {
            gps_line_buffer[gps_line_index] = '\0';

            gps_process_line(gps_line_buffer);

            gps_line_index = 0;

            memset(gps_line_buffer, 0, sizeof(gps_line_buffer));
        }
    }
}


/**
  * @brief  Get GPS data pointer.
  */
gps_data_t *gps_get_data(void)
{
    return &gps_data;
}


/**
  * @brief  Convert NMEA coordinate to decimal degrees.
  *
  * Example:
  *
  * 1605.1234
  *
  * becomes:
  *
  * 16.08539...
  */
double gps_nmea_to_degrees(double nmea_value)
{
    int degrees;
    double minutes;

    degrees = (int)(nmea_value / 100.0);

    minutes = nmea_value - ((double)degrees * 100.0);

    return ((double)degrees + minutes / 60.0);
}


/* Private functions ---------------------------------------------------------*/


/**
  * @brief Push one byte into software ring buffer.
  */
static void gps_ring_push(uint8_t data)
{
    uint16_t next_head;

    next_head = (uint16_t)(
        (gps_ring_head + 1U) % GPS_RING_BUFFER_SIZE
    );

    /*
     * If buffer is full, discard oldest byte.
     */
    if (next_head == gps_ring_tail)
    {
        gps_ring_tail = (uint16_t)(
            (gps_ring_tail + 1U) % GPS_RING_BUFFER_SIZE
        );
    }

    gps_ring_buffer[gps_ring_head] = data;

    gps_ring_head = next_head;
}


/**
  * @brief Pop one byte from software ring buffer.
  *
  * @return 1 if data available, 0 otherwise.
  */
static int gps_ring_pop(uint8_t *data)
{
    if (data == NULL)
    {
        return 0;
    }

    if (gps_ring_tail == gps_ring_head)
    {
        return 0;
    }

    *data = gps_ring_buffer[gps_ring_tail];

    gps_ring_tail = (uint16_t)(
        (gps_ring_tail + 1U) % GPS_RING_BUFFER_SIZE
    );

    return 1;
}


/**
  * @brief Process one complete NMEA sentence.
  */
static void gps_process_line(char *line)
{
    if (line == NULL)
    {
        return;
    }

    /*
     * Check checksum first.
     */
    if (!gps_check_checksum(line))
    {
        return;
    }

    /*
     * GGA:
     *
     * $GNGGA
     * $GPGGA
     */
    if ((strncmp(line, "$GNGGA", 6) == 0) ||
        (strncmp(line, "$GPGGA", 6) == 0))
    {
        gps_parse_gga(line);
    }

    /*
     * RMC:
     *
     * $GNRMC
     * $GPRMC
     */
    else if ((strncmp(line, "$GNRMC", 6) == 0) ||
             (strncmp(line, "$GPRMC", 6) == 0))
    {
        gps_parse_rmc(line);
    }
}


/**
  * @brief Check NMEA checksum.
  *
  * Example:
  *
  * $GNGGA,...*5A
  */
static bool gps_check_checksum(char *sentence)
{
    char *star;
    uint8_t checksum = 0;
    uint8_t received_checksum;
    char *p;

    if (sentence == NULL)
    {
        return false;
    }

    if (sentence[0] != '$')
    {
        return false;
    }

    star = strchr(sentence, '*');

    /*
     * No checksum.
     */
    if (star == NULL)
    {
        return false;
    }

    /*
     * Calculate XOR from after '$'
     * until before '*'.
     */
    p = &sentence[1];

    while (p < star)
    {
        checksum ^= (uint8_t)(*p);
        p++;
    }

    /*
     * Need two hexadecimal characters.
     */
    if ((star[1] == '\0') || (star[2] == '\0'))
    {
        return false;
    }

    received_checksum =
        (uint8_t)((gps_hex_to_byte(star[1]) << 4) |
                  gps_hex_to_byte(star[2]));

    return (checksum == received_checksum);
}


/**
  * @brief Convert hexadecimal ASCII character to value.
  */
static uint8_t gps_hex_to_byte(char c)
{
    if ((c >= '0') && (c <= '9'))
    {
        return (uint8_t)(c - '0');
    }

    if ((c >= 'A') && (c <= 'F'))
    {
        return (uint8_t)(c - 'A' + 10U);
    }

    if ((c >= 'a') && (c <= 'f'))
    {
        return (uint8_t)(c - 'a' + 10U);
    }

    return 0;
}


/**
  * @brief Parse GGA sentence.
  *
  * Format:
  *
  * $GNGGA,
  * UTC,
  * latitude,
  * N/S,
  * longitude,
  * E/W,
  * fix,
  * satellites,
  * HDOP,
  * altitude,
  * M,
  * geoid,
  * M,
  * ...
  */
static void gps_parse_gga(char *sentence)
{
    char *token;
    char *saveptr = NULL;

    uint8_t field = 0;

    double latitude_nmea = 0.0;
    double longitude_nmea = 0.0;

    char latitude_dir = 0;
    char longitude_dir = 0;

    float altitude = 0.0f;
    float geoid_height = 0.0f;

    uint8_t fix = 0;
    uint8_t satellites = 0;

    float hdop = 0.0f;

    token = strtok_r(sentence, ",", &saveptr);

    while (token != NULL)
    {
        switch (field)
        {
            case 1:
                /*
                 * UTC time
                 */
                gps_update_time(token);
                break;

            case 2:
                /*
                 * Latitude
                 */
                if (strlen(token) > 0)
                {
                    latitude_nmea = atof(token);
                }
                break;

            case 3:
                /*
                 * N/S
                 */
                if (strlen(token) > 0)
                {
                    latitude_dir = token[0];
                }
                break;

            case 4:
                /*
                 * Longitude
                 */
                if (strlen(token) > 0)
                {
                    longitude_nmea = atof(token);
                }
                break;

            case 5:
                /*
                 * E/W
                 */
                if (strlen(token) > 0)
                {
                    longitude_dir = token[0];
                }
                break;

            case 6:
                /*
                 * Fix quality
                 *
                 * 0 = invalid
                 * 1 = GPS fix
                 * 2 = DGPS
                 * ...
                 */
                if (strlen(token) > 0)
                {
                    fix = (uint8_t)atoi(token);
                }
                break;

            case 7:
                /*
                 * Number of satellites
                 */
                if (strlen(token) > 0)
                {
                    satellites = (uint8_t)atoi(token);
                }
                break;

            case 8:
                /*
                 * HDOP
                 */
                if (strlen(token) > 0)
                {
                    hdop = (float)atof(token);
                }
                break;

            case 9:
                /*
                 * Altitude
                 */
                if (strlen(token) > 0)
                {
                    altitude = (float)atof(token);
                }
                break;

            case 11:
                /*
                 * Geoid height
                 */
                if (strlen(token) > 0)
                {
                    geoid_height = (float)atof(token);
                }
                break;

            default:
                break;
        }

        field++;

        token = strtok_r(NULL, ",", &saveptr);
    }


    /*
     * Convert latitude.
     */
    if (latitude_nmea != 0.0)
    {
        gps_data.latitude =
            gps_nmea_to_degrees(latitude_nmea);

        if (latitude_dir == 'S')
        {
            gps_data.latitude = -gps_data.latitude;
        }
    }


    /*
     * Convert longitude.
     */
    if (longitude_nmea != 0.0)
    {
        gps_data.longitude =
            gps_nmea_to_degrees(longitude_nmea);

        if (longitude_dir == 'W')
        {
            gps_data.longitude = -gps_data.longitude;
        }
    }


    /*
     * Update other values.
     */
    gps_data.fix = fix;

    gps_data.satellites = satellites;

    gps_data.hdop = hdop;

    gps_data.altitude = altitude;

    gps_data.geoid_height = geoid_height;


    /*
     * Position is considered valid
     * when GPS has a fix.
     */
    if ((fix > 0U) &&
        (latitude_nmea != 0.0) &&
        (longitude_nmea != 0.0))
    {
        gps_data.position_valid = true;
    }
    else
    {
        gps_data.position_valid = false;
    }


    /*
     * At least one valid GGA has been received.
     */
    if (fix > 0U)
    {
        gps_data.data_valid = true;
    }
}


/**
  * @brief Parse RMC sentence.
  *
  * Format:
  *
  * $GNRMC,
  * UTC,
  * status,
  * latitude,
  * N/S,
  * longitude,
  * E/W,
  * speed knots,
  * course,
  * date,
  * ...
  */
static void gps_parse_rmc(char *sentence)
{
    char *token;
    char *saveptr = NULL;

    uint8_t field = 0;

    char status = 0;

    float speed_knots = 0.0f;
    float course = 0.0f;


    token = strtok_r(sentence, ",", &saveptr);

    while (token != NULL)
    {
        switch (field)
        {
            case 1:
                /*
                 * UTC
                 */
                gps_update_time(token);
                break;

            case 2:
                /*
                 * Status:
                 *
                 * A = valid
                 * V = invalid
                 */
                if (strlen(token) > 0)
                {
                    status = token[0];
                }
                break;

            case 7:
                /*
                 * Speed over ground
                 * in knots.
                 */
                if (strlen(token) > 0)
                {
                    speed_knots = (float)atof(token);
                }
                break;

            case 8:
                /*
                 * Course over ground
                 */
                if (strlen(token) > 0)
                {
                    course = (float)atof(token);
                }
                break;

            case 9:
                /*
                 * Date
                 */
                gps_update_date(token);
                break;

            default:
                break;
        }

        field++;

        token = strtok_r(NULL, ",", &saveptr);
    }


    /*
     * Update speed.
     */
    gps_data.speed_knots = speed_knots;

    gps_data.speed_kmh =
        speed_knots * 1.852f;

    gps_data.speed_mps =
        gps_data.speed_kmh / 3.6f;


    /*
     * Course.
     */
    gps_data.course_deg = course;


    /*
     * RMC validity.
     */
    if (status == 'A')
    {
        gps_data.data_valid = true;
    }
}


/**
  * @brief Update UTC time.
  *
  * NMEA format:
  *
  * hhmmss.ss
  */
static void gps_update_time(const char *utc)
{
    if (utc == NULL)
    {
        return;
    }

    if (strlen(utc) < 6U)
    {
        return;
    }

    gps_data.hour =
        (uint8_t)((utc[0] - '0') * 10 +
                  (utc[1] - '0'));

    gps_data.minute =
        (uint8_t)((utc[2] - '0') * 10 +
                  (utc[3] - '0'));

    gps_data.second =
        (uint8_t)((utc[4] - '0') * 10 +
                  (utc[5] - '0'));
}


/**
  * @brief Update date.
  *
  * NMEA format:
  *
  * ddmmyy
  */
static void gps_update_date(const char *date)
{
    if (date == NULL)
    {
        return;
    }

    if (strlen(date) < 6U)
    {
        return;
    }

    gps_data.day =
        (uint8_t)((date[0] - '0') * 10 +
                  (date[1] - '0'));

    gps_data.month =
        (uint8_t)((date[2] - '0') * 10 +
                  (date[3] - '0'));

    gps_data.year =
        (uint8_t)((date[4] - '0') * 10 +
                  (date[5] - '0'));
}


/* HAL callbacks -------------------------------------------------------------*/


/**
  * @brief UART Receive-To-Idle callback.
  *
  * IMPORTANT:
  * STM32F4 HAL expects TWO parameters:
  *
  * HAL_UARTEx_RxEventCallback(
  *      UART_HandleTypeDef *huart,
  *      uint16_t Size
  * );
  */
void GPS_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t current_pos;
    uint16_t i;

    (void)Size;

    if (huart == NULL)
    {
        return;
    }

    /*
     * Only process USART3.
     */
    if (huart->Instance != USART3)
    {
        return;
    }

    /*
     * Get current DMA write position.
     *
     * DMA counter decreases as bytes arrive.
     */
    current_pos =
        (uint16_t)(
            GPS_DMA_BUFFER_SIZE -
            __HAL_DMA_GET_COUNTER(huart->hdmarx)
        );

    /*
     * DMA buffer has not wrapped.
     */
    if (current_pos >= gps_dma_old_pos)
    {
        for (i = gps_dma_old_pos;
             i < current_pos;
             i++)
        {
            gps_ring_push(gps_dma_buffer[i]);
        }
    }

    /*
     * DMA buffer wrapped around.
     */
    else
    {
        for (i = gps_dma_old_pos;
             i < GPS_DMA_BUFFER_SIZE;
             i++)
        {
            gps_ring_push(gps_dma_buffer[i]);
        }

        for (i = 0;
             i < current_pos;
             i++)
        {
            gps_ring_push(gps_dma_buffer[i]);
        }
    }

    /*
     * Save new DMA position.
     */
    gps_dma_old_pos = current_pos;

    /*
     * If using circular DMA, reception continues automatically.
     *
     * If your CubeMX configuration uses NORMAL mode instead,
     * restart DMA here.
     */
}


/**
  * @brief UART error callback.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if (huart->Instance != USART3)
    {
        return;
    }

    /*
     * Restart DMA reception after UART error.
     */
    gps_dma_old_pos = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(
        huart,
        gps_dma_buffer,
        GPS_DMA_BUFFER_SIZE
    );
}
