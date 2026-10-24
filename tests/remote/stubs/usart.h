#ifndef TEST_REMOTE_USART_H
#define TEST_REMOTE_USART_H
#include "stm32h7xx.h"
enum
{
    UART_WORDLENGTH_9B = 9,
    UART_STOPBITS_2 = 2,
    UART_PARITY_EVEN = 2,
    UART_MODE_RX = 1,
};
typedef struct
{
    uint32_t BaudRate;
    uint32_t WordLength;
    uint32_t StopBits;
    uint32_t Parity;
    uint32_t Mode;
} UART_InitTypeDef;
typedef struct
{
    UART_InitTypeDef Init;
    void* hdmarx;
} UART_HandleTypeDef;
#ifdef __cplusplus
extern "C"
{
#endif
    extern UART_HandleTypeDef huart5;
#ifdef __cplusplus
}
#endif
#endif
