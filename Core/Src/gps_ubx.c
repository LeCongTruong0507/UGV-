/*
 * gps_ubx.c
 *
 *  Created on: Nov 8, 2026
 *      Author: Asus
 */

#include "gps_ubx.h"
#include <string.h>

// Định nghĩa các trạng thái của cỗ máy xử lý UBX
typedef enum {
    SYNC_1, SYNC_2, CLASS, ID, LEN_L, LEN_H, PAYLOAD, CK_A, CK_B
} UBX_State_t;

// Biến nội bộ của thư viện (Máy trạng thái)
static UBX_State_t state;
static uint8_t msg_class, msg_id;
static uint16_t msg_len, payload_idx;
static uint8_t payload_buf[100];
static uint8_t ck_a, ck_b, calc_ck_a, calc_ck_b;

// Khai báo thực thi các biến toàn cục (Đã khai báo extern ở file .h)
UBX_NAV_PVT_t gps_pvt_data;
double ugv_lat = 0.0;        // Dùng double (64-bit) để giữ chính xác 9 chữ số
double ugv_lon = 0.0;        // Dùng double (64-bit) để giữ chính xác 9 chữ số
float ugv_speed = 0.0f;
float ugv_heading = 0.0f;
uint8_t ugv_sat_num = 0;
uint8_t ugv_fix = 0;

// Hàm khởi tạo State Machine
void UBX_Init(void) {
    state = SYNC_1;
    memset(&gps_pvt_data, 0, sizeof(UBX_NAV_PVT_t));
}

// Hàm tính Checksum Fletcher-16 nội bộ
static void calc_checksum(uint8_t byte) {
    calc_ck_a += byte;
    calc_ck_b += calc_ck_a;
}

// Hàm giải mã cốt lõi (Bỏ vào ngắt UART RxCpltCallback, gọi từng byte 1)
// Trả về true nếu nhận xong và chuẩn Checksum 1 gói 92 byte payload
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
            // Bảo vệ chống tràn mảng: Gói PVT có payload cố định 92 byte
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
            state = SYNC_1; // Xong 1 gói, lập tức reset về chờ gói tiếp theo

            // Xác thực Checksum mạch tính so với Checksum module gửi
            if (ck_a == calc_ck_a && ck_b == calc_ck_b) {
                // Đảm bảo chính xác là gói UBX-NAV-PVT (Class 0x01, ID 0x07)
                if (msg_class == 0x01 && msg_id == 0x07) {

                    // Copy 92 byte nhị phân đặc ruột trực tiếp vào Struct
                    memcpy(&gps_pvt_data, payload_buf, sizeof(UBX_NAV_PVT_t));

                    // Quy đổi ra các số liệu thực tế để dùng cho thuật toán UGV
                    // CHÚ Ý: Ép kiểu (double) và chia cho 10000000.0 (chấm KHÔNG)
                    ugv_lat     = (double)gps_pvt_data.lat / 10000000.0;
                    ugv_lon     = (double)gps_pvt_data.lon / 10000000.0;

                    ugv_speed   = (float)gps_pvt_data.gSpeed / 1000.0f;
                    ugv_heading = (float)gps_pvt_data.headMot / 100000.0f;
                    ugv_sat_num = gps_pvt_data.numSV;
                    ugv_fix     = gps_pvt_data.fixType;

                    return true; // Báo hiệu cho main() biết đã có số liệu định vị mới nhất
                }
            }
            break;
    }
    return false;
}
