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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    double lat;
    double lng;
    int stop_time;
} Waypoint_t;

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

int16_t left_motor  = 0;
int16_t right_motor = 0;

uint8_t rx_gps_byte;

uint8_t calib_status = 0;
uint8_t calib_data[22] = {0};
uint8_t mag_status = 0;
uint8_t gyro_status = 0;
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

void Clear_SIM_Buffer(void) {
    HAL_UART_AbortReceive(&huart2);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    memset(rx_sim_buffer, 0, sizeof(rx_sim_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_sim_buffer, sizeof(rx_sim_buffer));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

bool SIM_WaitResponse(const char* expected_response, uint32_t timeout_ms) {
    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (strstr((char*)rx_sim_buffer, expected_response) != NULL) {
            return true;
        }
    }
    return false;
}

bool Send_AT_Command(char *cmd, const char *expected_response, uint32_t timeout_ms) {
    Clear_SIM_Buffer();

    HAL_UART_Transmit(&huart4, (uint8_t*)">> STM32: ", 10, 100);
    HAL_UART_Transmit(&huart4, (uint8_t*)cmd, strlen(cmd), 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), 1000);

    if (expected_response == NULL) {
        HAL_Delay(timeout_ms);
        return true;
    }

    bool result = SIM_WaitResponse(expected_response, timeout_ms);

    if (strlen((char*)rx_sim_buffer) > 0) {
        HAL_UART_Transmit(&huart4, (uint8_t*)"<< SIM : ", 9, 100);
        HAL_UART_Transmit(&huart4, rx_sim_buffer, strlen((char*)rx_sim_buffer), 100);
        HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", 2, 100);
    }
    return result;
}

void TCP_Publish_Telemetry(double lat, double lng, uint8_t bat, float spd, const char *status, float yaw_angle) {
    char json_payload[150];
    char mqtt_packet[200];
    char at_send_cmd[30];

    char topic[] = "lct_pbl3_rover/telemetry";
    uint16_t topic_len = strlen(topic);

    sprintf(json_payload, "{\"lat\":%.8f,\"lng\":%.8f,\"bat\":%d,\"spd\":%.1f,\"status\":\"%s\",\"yaw\":%.1f}",
                lat, lng, bat, spd, status, yaw_angle);
    uint16_t json_len = strlen(json_payload);

    uint16_t remaining_length = 2 + topic_len + json_len;

    uint16_t packet_idx = 0;
    mqtt_packet[packet_idx++] = 0x30;

    uint32_t x = remaining_length;
    do {
        uint8_t encoded_byte = x % 128;
        x /= 128;
        if (x > 0) encoded_byte |= 128;
        mqtt_packet[packet_idx++] = encoded_byte;
    } while (x > 0);

    mqtt_packet[packet_idx++] = (topic_len >> 8) & 0xFF;
    mqtt_packet[packet_idx++] = topic_len & 0xFF;
    memcpy(&mqtt_packet[packet_idx], topic, topic_len);
    packet_idx += topic_len;

    memcpy(&mqtt_packet[packet_idx], json_payload, json_len);
    packet_idx += json_len;

    sprintf(at_send_cmd, "AT+CIPSEND=0,%d\r\n", packet_idx);

    if (Send_AT_Command(at_send_cmd, ">", 300)) {
        Clear_SIM_Buffer();

        HAL_UART_Transmit(&huart2, (uint8_t*)mqtt_packet, packet_idx, 100);

        SIM_WaitResponse("SEND OK", 200);

        HAL_UART_Transmit(&huart4, (uint8_t*)">> TCP_DATA_SENT\r\n", 17, 100);
    }
}
void Control_Motors(int16_t speed1, int16_t speed2)
{
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
void Update_Robot_Motion(void)
{
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
  /* USER CODE BEGIN 2 */
//    gps_init(&huart3);
    UBX_Init();
    HAL_UART_Receive_IT(&huart3, &rx_gps_byte, 1);
    BNO055_Init();

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_sim_buffer, sizeof(rx_sim_buffer));
      __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

      HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n=== KHOI DONG MODULE SIM (TCP MODE) ===\r\n", 42, 100);
      HAL_Delay(4000);

      Send_AT_Command("AT\r\n", "OK", 1000);
      Send_AT_Command("ATE0\r\n", "OK", 1000);
      Send_AT_Command("AT+CPIN?\r\n", "READY", 2000);
      Send_AT_Command("AT+CSQ\r\n", "OK", 1000);
      Send_AT_Command("AT+CEREG?\r\n", "0,1", 2000);

      Send_AT_Command("AT+CGDCONT=1,\"IP\",\"v-internet\"\r\n", "OK", 2000);
        Send_AT_Command("AT+CGACT=1,1\r\n", "OK", 3000);

        Send_AT_Command("AT+NETCLOSE\r\n", "OK", 3000);
        Send_AT_Command("AT+CDNSCFG=\"8.8.8.8\",\"8.8.4.4\"\r\n", "OK", 2000);

        Send_AT_Command("AT+NETOPEN\r\n", "+NETOPEN: 0", 10000);
        HAL_Delay(2000);

        // SỬ DỤNG LỆNH "AT+CIPOPEN" CHUẨN CỦA DÒNG 4G A7600C VÀ BẮT CỜ "+CIPOPEN: 0,0"
        is_tcp_connected = Send_AT_Command("AT+CIPOPEN=0,\"TCP\",\"broker.hivemq.com\",1883\r\n", "+CIPOPEN: 0,0", 10000);

        if (is_tcp_connected) {
                    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n>> KET NOI TCP THANH CONG!\r\n", 28, 100);

                    HAL_Delay(1000);

                    char at_connect_cmd[] = "AT+CIPSEND=0,14\r\n";
                    if (Send_AT_Command(at_connect_cmd, ">", 2000)) {
                        Clear_SIM_Buffer();
                        uint8_t mqtt_connect_packet[] = {0x10, 0x0C, 0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04, 0x02, 0x00, 0x3C, 0x00, 0x00};
                        HAL_UART_Transmit(&huart2, mqtt_connect_packet, sizeof(mqtt_connect_packet), 1000);

                        // Chờ phản hồi CONNACK từ Broker
                        SIM_WaitResponse("SEND OK", 3000);
                    }

                    HAL_Delay(1000);

                    char at_sub_cmd[] = "AT+CIPSEND=0,29\r\n";
                                if (Send_AT_Command(at_sub_cmd, ">", 2000)) {
                                    Clear_SIM_Buffer();

                                    uint8_t mqtt_sub_packet[29] = {
                                        0x82, 0x1B,             // Cờ Subscribe (0x82) & Độ dài phần còn lại (27 bytes = 0x1B)
                                        0x00, 0x01,             // Packet ID (1)
                                        0x00, 0x16,             // Độ dài Topic: 22 bytes (0x16)
                                        'l','c','t','_','p','b','l','3','_','r','o','v','e','r','/','c','o','m','m','a','n','d',
                                        0x00                    // Requested QoS: 0
                                    };

                                    HAL_UART_Transmit(&huart2, mqtt_sub_packet, sizeof(mqtt_sub_packet), 1000);

                                    SIM_WaitResponse("SEND OK", 3000);

                                    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n>> DA SUBSCRIBE TOPIC COMMAND!\r\n", 34, 100);
                                }
                } else {
                    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n>> LOI: KHONG THE MO KET NOI TCP!\r\n", 35, 100);
                }
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
	  gps_process();

/*	  if (gps_data.satellites >= 5) {

	            double raw_lat = (double)gps_data.latitude;
	            double raw_lng = (double)gps_data.longitude;

	            // nếu là lần đầu tiên bắt được GPS (biến vẫn = 0), gán thẳng giá trị
	            if (filtered_lat == 0.0 && raw_lat != 0.0) {
	                filtered_lat = raw_lat;
	                filtered_lng = raw_lng;
	            }
	            else if (raw_lat != 0.0) {
	                if (strcmp(rover_status, "IDLE") == 0 || strcmp(rover_status, "STOP") == 0) {

	                }
	                else {
	                    filtered_lat = (filtered_lat * 0.7) + (raw_lat * 0.3);
	                    filtered_lng = (filtered_lng * 0.7) + (raw_lng * 0.3);
	                }
	            }
	        }  */

	        if (is_tcp_connected && (HAL_GetTick() - last_telemetry_time >= 500)) {
	            last_telemetry_time = HAL_GetTick();

	            if (1) {
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
	        Update_Robot_Motion();
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
