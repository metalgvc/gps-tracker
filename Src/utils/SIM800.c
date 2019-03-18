/*
 * SIM800.c
 *
 *  Created on: Mar 11, 2019
 *      Author: metalgvc
 */

#include "SIM800.h"

#define SIM800_TX_STATUS_READY 0
#define SIM800_TX_STATUS_BUSY 1

uint8_t TxStatus;
uint8_t isWaitingResponse;
ATResponse lastATResponse;
uint8_t respTmpBuff[256];


//UART_HandleTypeDef GSMUart;

/**
 * boot/init SIM800 module
 */
void SIM800init(UART_HandleTypeDef *huart, GPIO_TypeDef *bootPort, uint16_t bootPin)
{
    SIM800Boot(bootPort, bootPin);

    TxStatus = SIM800_TX_STATUS_READY;
    isWaitingResponse = 0;
    lastATResponse = AT_NULL;

    ATResponse resp = SIM800_sendAT("AT\r\n", huart, 1);

    if (resp == AT_OK) {
        // send PIN if exists

    } else {
        Error_Handler(__FILE__, __LINE__);
    }

}

/**
 * SIM800 boot pin -> GND on 2 seconds
 */
void SIM800Boot(GPIO_TypeDef *bootPort, uint16_t bootPin)
{
    // boot pin to GND
    HAL_GPIO_WritePin(bootPort, bootPin, GPIO_PIN_RESET);

    //uint32_t startTick = HAL_GetTick();

    // wait 2 seconds
    osDelay(2000);
    //while ( (HAL_GetTick() - startTick) < 2000 );

    // boot pin to high
    HAL_GPIO_WritePin(bootPort, bootPin, GPIO_PIN_SET);
}

/**
 * send AT command to SIM800
 */
ATResponse SIM800_sendAT(char *command, UART_HandleTypeDef *huart, uint8_t responseLines)
{
    ATResponse atresp = AT_NULL;
    HAL_StatusTypeDef res;

    // wait until transmit is ready and if waiting for response
    while (TxStatus == SIM800_TX_STATUS_BUSY || isWaitingResponse == 1);

    TxStatus = SIM800_TX_STATUS_BUSY;

    res = HAL_UART_Transmit_DMA(huart, (uint8_t*)command, sizeof(command)-1);
    //res = HAL_UART_Transmit(&huart, (uint8_t*)command, 1, HAL_MAX_DELAY);

    if (res == HAL_OK) {
        if (responseLines > 0) {
            atresp = SIM800_getResponse(huart, responseLines, (uint32_t)60000);
        }
    } else {
        Error_Handler(__FILE__, __LINE__);
    }

	return atresp;
}

/**
 * wait for response
 * timeout ms
 * @return ATResponse
 */
ATResponse SIM800_getResponse(UART_HandleTypeDef *huart, uint8_t lines, uint32_t timeout)
{
    ATResponse atresp = AT_NULL;
    HAL_StatusTypeDef res;

    isWaitingResponse = true;

    // wait for transmit
    while(TxStatus == SIM800_TX_STATUS_BUSY);

    res = HAL_UART_Receive_DMA(huart, (uint8_t*)respTmpBuff, sizeof(respTmpBuff));

    if (res == HAL_ERROR) {
        Error_Handler(__FILE__, __LINE__);
    }

    // milliseconds
    uint32_t startTick = HAL_GetTick();

    // wait for response lines
    // check if command is end on each iteration
    // timeout (if 0 - no timeout)
    uint8_t respLines = 0;
    while (respLines < lines){

        // timeout
        if (timeout > 0 && (HAL_GetTick() - startTick) > timeout) {
            atresp = AT_TIMEOUT;
            break;
        }

        // count response lines
        uint8_t i;
        respLines = 0;
        for (i = 0; i < sizeof(respTmpBuff); i++) {
           if (respTmpBuff[i] == '\r'/* || respTmpBuff[i] == '\n'*/) {
               respLines++;
           }
        }
    };

    // parse response
    if (atresp != AT_TIMEOUT) {
        atresp = SIM800_parseResponse(respTmpBuff);
    }

    //respTmpBuff = NULL;
    memset(respTmpBuff, 0, sizeof(respTmpBuff));

    isWaitingResponse = false;

    return atresp;
}

/**
 * blocking wait response
 */
ATResponse SIM800_waitMessage(UART_HandleTypeDef *huart, uint8_t lines)
{
    return SIM800_getResponse(huart, lines, 0);
}

/**
 * parse response from SIM800
 * @return ATResponse
 */
ATResponse SIM800_parseResponse(uint8_t *respBuff)
{
    ATResponse atresp = AT_UNKNOWN;

    char line[50];
    memset(line, 0, sizeof(line));

    uint8_t j = 0;
    for(uint8_t i = 0; i < sizeof(respBuff); i++){

        // search line
        if (respBuff[i] == '\r') {
            j = 0;

            // TODO parse other responses
            if (_strcmp("OK", line) == 0) {
                atresp = AT_OK;
                break;
            }

            memset(line, 0, sizeof(line));
        } else {
            line[++j] = respBuff[i];
        }

    }

    return atresp;
}

/**
 * compare const str with char array
 */
static uint8_t _strcmp(const char *str1, char *str2)
{
    if (sizeof(str1) != sizeof(str2)) {
        return (uint8_t)1;
    }

    for (uint8_t i; i < sizeof(str1); i++) {
        if (str1[i] != str2[i]) {
            return (uint8_t)1;
        }
    }

    return (uint8_t)0;
}

/**
 * transmit done callback
 */
void SIM800_Tx_callback(UART_HandleTypeDef *huart)
{
    TxStatus = SIM800_TX_STATUS_READY;
}

/**
 * receive done callback
 */
void SIM800_Rx_callback(UART_HandleTypeDef *huart)
{
    // buffer overflow
    isWaitingResponse = 0;
}
