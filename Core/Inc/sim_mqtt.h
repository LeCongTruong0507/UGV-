/*
 * sim_mqtt.h
 *
 *  Created on: Nov 13, 2026
 *      Author: Asus
 */

#ifndef INC_SIM_MQTT_H_
#define INC_SIM_MQTT_H_

#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

void Sim_Init(void);
void TCP_Publish_Telemetry(double lat, double lng, uint8_t bat, float spd, const char *status, float yaw_angle);
bool Send_AT_Command(char *cmd, const char *expected_response, uint32_t timeout_ms);
void Clear_SIM_Buffer(void);
bool SIM_WaitResponse(const char *expected_response, uint32_t timeout_ms);

#endif /* INC_SIM_MQTT_H_ */
