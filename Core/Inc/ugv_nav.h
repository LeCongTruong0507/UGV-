/*
 * ugv_nav.h
 *
 *  Created on: Nov 10, 2026
 *      Author: Asus
 */

#ifndef INC_UGV_NAV_H_
#define INC_UGV_NAV_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define BASE_SPEED 200         // Tốc độ chạy thẳng (Khoảng 45% công suất)
#define MAX_TURN_SPEED 300     // Giới hạn xung bù lúc đánh lái
#define Kp 0.5f                // Hệ số đánh lái (Tăng nếu xe ôm cua chậm)
#define WAYPOINT_RADIUS 1.5    // Khoảng cách chấp nhận đã đến đích (mét)

typedef struct {
    double lat;
    double lng;
    int stop_time;
} Waypoint_t;

extern Waypoint_t rover_mission[];
extern uint8_t total_waypoints;
extern char rover_status[20];
extern float yaw;
extern void Control_Motors(int16_t speed1, int16_t speed2);

extern uint8_t current_wp_index;

void UGV_Auto_Drive(void);

#endif /* INC_UGV_NAV_H_ */
