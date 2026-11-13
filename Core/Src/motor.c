/*
 * motor.c
 *
 *  Created on: Nov 13, 2026
 *      Author: Asus
 */


#include "motor.h"

extern TIM_HandleTypeDef htim1;

extern uint32_t pwm_value[6];

int16_t left_motor  = 0;
int16_t right_motor = 0;

void Control_Motors(int16_t speed1, int16_t speed2) {
    speed1 = (speed1 > 900) ? 900 : (speed1 < -900 ? -900 : speed1);
    speed2 = (speed2 > 900) ? 900 : (speed2 < -900 ? -900 : speed2);

    if(abs(speed1) > 29) {
    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, (speed1 >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, abs(speed1));
    }
    else {
    	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
    if(abs(speed2) > 29) {
    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, (speed2 >= 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, abs(speed2));
    }
    else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}

void Update_Robot_Motion(void) {
	if (pwm_value[0] < 900 || pwm_value[1] < 900 || pwm_value[2] < 900)
	    {
	        Control_Motors(0, 0);
	        return;
	    }
    int32_t forward = 0;
    int32_t steer = 0;
    if (pwm_value[0] > 1200)
    {
        forward = ((int32_t)pwm_value[0] - 1200) * 5 / 7;

        if (forward > 500) forward = 500;
    }
    if (pwm_value[1] > 1560 || pwm_value[1] < 1541)
    {
        steer = ((int32_t)pwm_value[1] - 1551) * 1 / 4;

        if (steer > 300) steer = 300;
        if (steer < -300) steer = -300;
    }

     left_motor  = forward + steer;
     right_motor = forward - steer;
    if(pwm_value[2] > 1600) {
       Control_Motors(left_motor, right_motor);
    } else if(pwm_value[2] < 1400) {
    	Control_Motors(-left_motor, -right_motor);
    }
}
