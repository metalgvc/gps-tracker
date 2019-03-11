/*
 * uart_debug.c
 *
 *  Created on: Mar 10, 2019
 *      Author: metalgvc
 */

#include "main.h"
#include "uart_debug.h"

UART_HandleTypeDef dbguart;

void uartDebugInit(void)
{

	dbguart.Instance = USART2;
	dbguart.Init.BaudRate = 115200;
	dbguart.Init.WordLength = UART_WORDLENGTH_8B;
	dbguart.Init.StopBits = UART_STOPBITS_1;
	dbguart.Init.Parity = UART_PARITY_NONE;
	dbguart.Init.Mode = UART_MODE_TX_RX;
	dbguart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	dbguart.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&dbguart) != HAL_OK)
	{
		Error_Handler(__FILE__, __LINE__);
	}
}

void uartDebugSendStr(const char *str)
{
	HAL_UART_Transmit(&dbguart, (uint8_t*)str, sizeof(str)-1, HAL_MAX_DELAY);
}
