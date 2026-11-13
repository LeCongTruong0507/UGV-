/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps_ubx.h"
#include "BNO055.h"
#include "ugv_nav.h"
#include "ina219.h"
#include "motor.h"
#include "sim_mqtt.h"
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern float yaw;
extern char rover_status[20];
extern uint8_t ugv_fix;
extern double ugv_lat;
extern double ugv_lon;
extern float ugv_speed;
extern bool is_tcp_connected;
extern uint8_t bat_level;
/* USER CODE END Variables */
osThreadId Task_MotionHandle;
osThreadId Task_TelemetryHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_Motion(void const * argument);
void StartTask_Telemetry(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_Motion */
  osThreadDef(Task_Motion, StartTask_Motion, osPriorityAboveNormal, 0, 256);
  Task_MotionHandle = osThreadCreate(osThread(Task_Motion), NULL);

  /* definition and creation of Task_Telemetry */
  osThreadDef(Task_Telemetry, StartTask_Telemetry, osPriorityNormal, 0, 512);
  Task_TelemetryHandle = osThreadCreate(osThread(Task_Telemetry), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTask_Motion */
/**
  * @brief  Function implementing the Task_Motion thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_Motion */
void StartTask_Motion(void const * argument)
{
  /* USER CODE BEGIN StartTask_Motion */
  /* Infinite loop */
  for(;;)
  {
      BNO055_ReadEuler(&yaw);
      if (strcmp(rover_status, "MOVING") == 0) {
          if (ugv_fix == 3) {
              UGV_Auto_Drive();
          } else {
              Control_Motors(0, 0);
          }
      } else {
          Update_Robot_Motion();
      }
      osDelay(20);
  }
  /* USER CODE END StartTask_Motion */
}

/* USER CODE BEGIN Header_StartTask_Telemetry */
/**
* @brief Function implementing the Task_Telemetry thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Telemetry */
void StartTask_Telemetry(void const * argument)
{
  /* USER CODE BEGIN StartTask_Telemetry */
  /* Infinite loop */
  for(;;)
  {
      if (is_tcp_connected) {
          float voltage = INA219_GetBusVoltage_V();

          float bat_percent = ((voltage - 11.1f) / (12.6f - 11.1f)) * 100.0f;
          if (bat_percent > 100.0f) bat_percent = 100.0f;
          if (bat_percent < 0.0f) bat_percent = 0.0f;
          bat_level = (uint8_t)bat_percent;

          TCP_Publish_Telemetry(
              (double)ugv_lat,
              (double)ugv_lon,
              bat_level,
              (float)ugv_speed,
              rover_status,
              (float)yaw
          );
      }
      osDelay(500);
  }
  /* USER CODE END StartTask_Telemetry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
