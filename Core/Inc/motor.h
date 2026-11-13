/*
 * motor.h
 *
 *  Created on: Nov 13, 2026
 *      Author: Asus
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include <stdlib.h>

void Control_Motors(int16_t speed1, int16_t speed2);
void Update_Robot_Motion(void);

#endif /* INC_MOTOR_H_ */
