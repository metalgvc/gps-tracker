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

#define SIM800_BOOT_PORT GPIOB
#define SIM800_BOOT_PIN GPIO_PIN_4

// response from SIM800
typedef enum {
    AT_OK = 0,          // GSM module return OK
    AT_UNKNOWN = 1,     // unknown response from module
    AT_NULL = 2,        // initial value before response
    AT_TIMEOUT = 3      // response timeout
} ATResponse;

void SIM800Boot(GPIO_TypeDef *bootPort, uint16_t bootPin);
void SIM800init(UART_HandleTypeDef *huart, GPIO_TypeDef *bootPort, uint16_t bootPin);
ATResponse SIM800_sendAT(char *command, UART_HandleTypeDef *huart, uint8_t responseLines);
ATResponse SIM800_getResponse(UART_HandleTypeDef *huart, uint8_t lines, uint32_t timeout);
ATResponse SIM800_parseResponse(uint8_t *respBuff);

static uint8_t _strcmp(const char *str1, char *str2);

#endif /* UTILS_SIM800_H_ */
