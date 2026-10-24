#ifndef TEST_REMOTE_BSP_UART_H
#define TEST_REMOTE_BSP_UART_H
#include "usart.h"
typedef void (*UART_Callback)(uint8_t*, uint16_t);
void UART_Init(UART_HandleTypeDef* huart, UART_Callback callback);
#endif
