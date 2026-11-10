/*
 * ugv_nav.c
 *
 *  Created on: Nov 10, 2026
 *      Author: Asus
 */

#include "ugv_nav.h"
#include "gps_ubx.h"
#include <math.h>
#include <string.h>
#include "main.h"

#define EARTH_RADIUS 6371000.0
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

uint8_t current_wp_index = 0;
static double true_lat = 0, true_lng = 0;
double target_bearing;

static double Get_Distance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
               sin(dLon/2) * sin(dLon/2);
    return EARTH_RADIUS * (2 * atan2(sqrt(a), sqrt(1-a)));
}

static double Get_Bearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double y = sin(dLon) * cos(lat2 * DEG_TO_RAD);
    double x = cos(lat1 * DEG_TO_RAD) * sin(lat2 * DEG_TO_RAD) -
               sin(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * cos(dLon);
    double bearing = atan2(y, x) * RAD_TO_DEG;
    return (bearing < 0) ? (bearing + 360.0) : bearing;
}

void UGV_Auto_Drive(void) {
    // hết điểm WP || status ko phải MOVING
    if (total_waypoints == 0 || current_wp_index >= total_waypoints) {
        if (strcmp(rover_status, "MOVING") == 0) {
            strcpy(rover_status, "STOP");
            Control_Motors(0, 0);
            true_lat = 16.07480000;
            true_lng = 108.15100000;
        }
        return;
    }

//    if (ugv_speed > 0.1f || (true_lat == 0) ) {
    if (ugv_speed > 0.1f || 1 ) {
        true_lat = 16.07480000;
        true_lng = 108.15100000;
    }
//    if (true_lat == 0 || true_lng == 0) return;

    double target_lat = rover_mission[current_wp_index].lat;
    double target_lng = rover_mission[current_wp_index].lng;

    double distance_to_target = Get_Distance(true_lat, true_lng, target_lat, target_lng);
    if (distance_to_target < WAYPOINT_RADIUS) {
        current_wp_index++;
        HAL_Delay(2000);
        return;
    }

    target_bearing = Get_Bearing(true_lat, true_lng, target_lat, target_lng);
    double error_angle = target_bearing - (double)yaw;

    if (error_angle > 180.0) error_angle -= 360.0;
    else if (error_angle < -180.0) error_angle += 360.0;

    int16_t turn_speed = (int16_t)(error_angle * Kp);
    if(turn_speed > MAX_TURN_SPEED) turn_speed = MAX_TURN_SPEED;
    if(turn_speed < -MAX_TURN_SPEED) turn_speed = -MAX_TURN_SPEED;

    Control_Motors(BASE_SPEED + turn_speed, BASE_SPEED - turn_speed);
}
