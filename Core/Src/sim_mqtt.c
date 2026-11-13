/*
 * sim_mqtt.c
 *
 *  Created on: Nov 13, 2026
 *      Author: Asus
 */

#include "sim_mqtt.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_usart2_rx;

extern uint8_t rx_sim_buffer[256];
extern bool is_tcp_connected;


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

void Sim_Init(void) {
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
}

