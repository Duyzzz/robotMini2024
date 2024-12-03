/*
 * uart.h
 *
 *  Created on: Oct 2, 2024
 *      Author: Admin
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"


 void MX_USART1_UART_Init(void);  // Hàm khởi tạo UART1
 void MX_USART2_UART_Init(void);

 void UART1_Transmit(char *string);  // Hàm truyền dữ liệu UART1
 void UART1_Receive(uint8_t *buffer, uint16_t size);  // Hàm nhận dữ liệu UART1

 int16_t Read_Compass(void);

 void Write_UART2(char *string);
 uint8_t Read_UART2(uint8_t *buffer, uint16_t size);




#ifdef __cplusplus
}
#endif

#endif /* __UART1_H */

