/*
 * gps_ubx.c
 *
 *  Created on: Nov 8, 2026
 *      Author: Asus
 */

#include "gps_ubx.h"
#include <string.h>

typedef enum {
    SYNC_1, SYNC_2, CLASS, ID, LEN_L, LEN_H, PAYLOAD, CK_A, CK_B
} UBX_State_t;

static UBX_State_t state;
static uint8_t msg_class, msg_id;
static uint16_t msg_len, payload_idx;
static uint8_t payload_buf[100];
static uint8_t ck_a, ck_b, calc_ck_a, calc_ck_b;

UBX_NAV_PVT_t gps_pvt_data;
double ugv_lat = 0.0;
double ugv_lon = 0.0;
float ugv_speed = 0.0f;
float ugv_heading = 0.0f;
uint8_t ugv_sat_num = 0;
uint8_t ugv_fix = 0;

void UBX_Init(void) {
    state = SYNC_1;
    memset(&gps_pvt_data, 0, sizeof(UBX_NAV_PVT_t));
}

static void calc_checksum(uint8_t byte) {
    calc_ck_a += byte;
    calc_ck_b += calc_ck_a;
}

bool UBX_ProcessByte(uint8_t byte) {
    switch (state) {
        case SYNC_1:
            if (byte == 0xB5) state = SYNC_2;
            break;

        case SYNC_2:
            if (byte == 0x62) {
                state = CLASS;
                calc_ck_a = 0;
                calc_ck_b = 0;
            } else {
                state = SYNC_1;
            }
            break;

        case CLASS:
            msg_class = byte;
            calc_checksum(byte);
            state = ID;
            break;

        case ID:
            msg_id = byte;
            calc_checksum(byte);
            state = LEN_L;
            break;

        case LEN_L:
            msg_len = byte;
            calc_checksum(byte);
            state = LEN_H;
            break;

        case LEN_H:
            msg_len |= (byte << 8);
            calc_checksum(byte);
            if (msg_len > 92) {
                state = SYNC_1;
            } else {
                payload_idx = 0;
                state = PAYLOAD;
            }
            break;

        case PAYLOAD:
            payload_buf[payload_idx++] = byte;
            calc_checksum(byte);
            if (payload_idx >= msg_len) {
                state = CK_A;
            }
            break;

        case CK_A:
            ck_a = byte;
            state = CK_B;
            break;

        case CK_B:
            ck_b = byte;
            state = SYNC_1;

            if (ck_a == calc_ck_a && ck_b == calc_ck_b) {
                if (msg_class == 0x01 && msg_id == 0x07) {

                    memcpy(&gps_pvt_data, payload_buf, sizeof(UBX_NAV_PVT_t));

                    ugv_lat     = (double)gps_pvt_data.lat / 10000000.0;
                    ugv_lon     = (double)gps_pvt_data.lon / 10000000.0;

                    ugv_speed   = (float)gps_pvt_data.gSpeed / 1000.0f;
                    ugv_heading = (float)gps_pvt_data.headMot / 100000.0f;
                    ugv_sat_num = gps_pvt_data.numSV;
                    ugv_fix     = gps_pvt_data.fixType;

                    return true;
                }
            }
            break;
    }
    return false;
}
