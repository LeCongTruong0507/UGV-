/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gps_ubx.h"
#include "BNO055.h"
#include "ugv_nav.h"
#include "INA219.h"
#include "motor.h"
#include "sim_mqtt.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_WAYPOINTS 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern DMA_HandleTypeDef hdma_usart2_rx;
extern I2C_HandleTypeDef hi2c3;

bool is_tcp_connected = false;

float yaw = 0.0f;

uint8_t rx_sim_buffer[256];
uint32_t last_telemetry_time = 0;
char rover_status[20] = "IDLE";
float target_speed = 0.0f;

uint8_t bat_level = 85;
float current_speed = 0.0f;

uint16_t counter = 0;

Waypoint_t rover_mission[MAX_WAYPOINTS];
uint8_t total_waypoints = 0;

double filtered_lat = 0.0;
double filtered_lng = 0.0;

uint32_t start_time[6];
uint16_t pwm_value[6];

uint8_t rx_gps_byte;

uint8_t calib_status = 0;
uint8_t calib_data[22] = {0};
uint8_t mag_status = 0;
uint8_t gyro_status = 0;

uint8_t bat_level;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
bool SIM_WaitResponse(const char* expected_response, uint32_t timeout_ms);
void Clear_SIM_Buffer(void);
bool Send_AT_Command(char *cmd, const char *expected_response, uint32_t timeout_ms);
void TCP_Publish_Telemetry(double lat, double lng, uint8_t bat, float spd, const char *status, float yaw_angle);
void Update_Robot_Motion(void);
void Control_Motors(int16_t speed1, int16_t speed2);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
//    gps_init(&huart3);
    UBX_Init();
    HAL_UART_Receive_IT(&huart3, &rx_gps_byte, 1);
    BNO055_Init();
    INA219_Init(&hi2c3);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_sim_buffer, sizeof(rx_sim_buffer));
      __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

        Sim_Init();

        HAL_TIM_Base_Start(&htim2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  BNO055_ReadEuler(&yaw);
	  if (strcmp(rover_status, "MOVING") == 0) {
	            if (ugv_fix == 3) {
	                UGV_Auto_Drive();
	            }
	        } else {
	            Update_Robot_Motion();
	        }

	        if (is_tcp_connected && (HAL_GetTick() - last_telemetry_time >= 500)) {
	            last_telemetry_time = HAL_GetTick();
	                 float voltage = INA219_GetBusVoltage_V();
	                      float bat_percent = ((voltage - 7.4f) / (12.6f - 7.4f)) * 100.0f; // lipo 3S
	                      if (bat_percent > 100.0f) bat_percent = 100.0f;
	                      if (bat_percent < 0.0f) bat_percent = 0.0f;
	                      bat_level = (uint8_t)bat_percent;

	                TCP_Publish_Telemetry(
	                    (double)ugv_lat,
	                    (double)ugv_lon,
	                    (uint8_t)bat_level,
						(float)ugv_speed,
	                    rover_status,
	                    (float)yaw
	                );
	        }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if(GPIO_Pin == GPIO_PIN_6)
    {
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
        {
            start_time[0] = __HAL_TIM_GET_COUNTER(&htim2);
        }
        else
        {
            pwm_value[0] = __HAL_TIM_GET_COUNTER(&htim2) - start_time[0];
        }
    }

    else if(GPIO_Pin == GPIO_PIN_7)
    {
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET)
        {
            start_time[1] = __HAL_TIM_GET_COUNTER(&htim2);
        }
        else
        {
            pwm_value[1] = __HAL_TIM_GET_COUNTER(&htim2) - start_time[1];
        }
    }
    else if(GPIO_Pin == GPIO_PIN_5)
        {
            if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET)
            {
                start_time[2] = __HAL_TIM_GET_COUNTER(&htim2);
            }
            else
            {
                pwm_value[2] = __HAL_TIM_GET_COUNTER(&htim2) - start_time[2];
            }
        }

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		        if (UBX_ProcessByte(rx_gps_byte)) {

		        }
		        HAL_UART_Receive_IT(&huart3, &rx_gps_byte, 1);
		    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
//    GPS_UART_RxEventCallback(huart, Size);
        if (huart->Instance == USART2) {
            for(uint16_t i = 0; i < Size; i++) {
                if(rx_sim_buffer[i] == 0x00) {
                    rx_sim_buffer[i] = ' ';
                }
            }
            rx_sim_buffer[Size < sizeof(rx_sim_buffer) ? Size : (sizeof(rx_sim_buffer) - 1)] = '\0';

            if (strstr((char*)rx_sim_buffer, "\"cmd\":\"CLEAR_MISSION\"") != NULL) {
                total_waypoints = 0;
                current_wp_index = 0;
                HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n>> DA XOA LO TRINH CU\r\n", 25, 100);
            }
            else if (strstr((char*)rx_sim_buffer, "\"cmd\":\"ADD_WP\"") != NULL) {

                        if (total_waypoints < MAX_WAYPOINTS) {

                            char *lat_p = strstr((char*)rx_sim_buffer, "\"lat\":");
                            char *lng_p = strstr((char*)rx_sim_buffer, "\"lng\":");
                            char *stop_p = strstr((char*)rx_sim_buffer, "\"stop\":");

                            if (lat_p != NULL && lng_p != NULL && stop_p != NULL) {
                                double p_lat = atof(lat_p + 6);

                                double p_lng = atof(lng_p + 6);

                                int p_stop = atoi(stop_p + 7);

                                rover_mission[total_waypoints].lat = p_lat;
                                rover_mission[total_waypoints].lng = p_lng;
                                rover_mission[total_waypoints].stop_time = p_stop;

                                total_waypoints++;

                                char dbg[100];
                                sprintf(dbg, "\r\n>> DA NHAN WP %d: Lat=%.8f, Lng=%.8f, Stop=%d(s)\r\n",
                                        total_waypoints, p_lat, p_lng, p_stop);
                                HAL_UART_Transmit(&huart4, (uint8_t*)dbg, strlen(dbg), 100);
                            }
                        } else {
                            HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n>> LOI: BO NHO WAYPOINT DA DAY!\r\n", 35, 100);
                        }
                    }
            else if (strstr((char*)rx_sim_buffer, "\"cmd\":\"START\"") != NULL) {
                strcpy(rover_status, "MOVING");
                HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n>> LENH: START\r\n", 18, 100);
            }
            else if (strstr((char*)rx_sim_buffer, "\"cmd\":\"STOP\"") != NULL) {
                strcpy(rover_status, "STOP");
                HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n>> LENH: STOP\r\n", 17, 100);
            }
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_sim_buffer, sizeof(rx_sim_buffer));
            __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
