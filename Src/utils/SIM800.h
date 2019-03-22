/*
 * SIM800.h
 *
 *  Created on: Mar 11, 2019
 *      Author: metalgvc
 */

#ifndef UTILS_SIM800_H_
#define UTILS_SIM800_H_

#include "main.h"
#include "stdint.h"
#include "cmsis_os.h"
#include "stdio.h"

#define SIM800_BOOT_Pin GPIO_PIN_8
#define SIM800_BOOT_GPIO_Port GPIOA

// response from SIM800
typedef enum {
    AT_OK = 0,          // GSM module return OK
    AT_UNKNOWN = 1,     // unknown response from module
    AT_NULL = 2,        // initial value before response
    AT_TIMEOUT = 3,      // response timeout
    AT_EXT_COMMAND = 4  // external command
} ATResponse;

typedef struct {
    ATResponse status;  // response status
    uint8_t *response;  // response buff e.g. command
} ATResponse_t;

void SIM800Boot();
void SIM800init(UART_HandleTypeDef *huart);
ATResponse_t SIM800_sendAT(char* command, uint32_t responseTimeout);
ATResponse_t SIM800_waitResponse(uint32_t timeout);
ATResponse_t SIM800_parseResponse(uint8_t *respBuff);

static uint8_t _strcmp(const char *str1, char *str2);
static void initRxBuffer(UART_DMA_Rx_t *initBuff);
static void _clearReceiveBuff();

#endif /* UTILS_SIM800_H_ */
