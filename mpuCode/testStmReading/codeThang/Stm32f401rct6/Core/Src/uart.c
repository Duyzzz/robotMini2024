/*
 * uart.c
 *
 *  Created on: Oct 2, 2024
 *      Author: Admin
 */
#include "uart.h"

//UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

uint8_t received_command;

/*
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}
*/
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  // cấu hình ngắt
//  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

}


/*
int16_t Read_Compass(void){

	char cmd = 'z';           // Lệnh yêu cầu la bàn gửi dữ liệu
	uint8_t byte_h, byte_l;    // Byte cao và thấp
	int16_t value_compass;   // Giá trị nhận được từ la bàn

	HAL_UART_Transmit(&huart1, (uint8_t*)&cmd, 1, HAL_MAX_DELAY);

	HAL_UART_Receive(&huart1, &byte_h, 1, HAL_MAX_DELAY);

	HAL_UART_Receive(&huart1, &byte_l, 1, HAL_MAX_DELAY);

	value_compass = (byte_h << 8) | byte_l;

	return value_compass;

}*/













