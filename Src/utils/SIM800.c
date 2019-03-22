/*
 * SIM800.c
 *
 *  Created on: Mar 11, 2019
 *      Author: metalgvc
 */

#include "SIM800.h"

// TODO place to SIM800.h
#define SIM800_TX_STATUS_READY 0
#define SIM800_TX_STATUS_BUSY 1
#define SIM800_DMA_BUFF_SIZE 256  // DMA circular buffer size
#define SIM800_Rx_BUFF_SIZE 256

uint8_t TxStatus;
uint8_t isWaitingResponse;

UART_HandleTypeDef* SIM800_huart;
uint8_t respTmpBuff[SIM800_DMA_BUFF_SIZE];                  // circular UART DMA receive buffer
uint8_t receivedMsgBuff[SIM800_Rx_BUFF_SIZE];              // received message buffer to parse
UART_DMA_Rx_t respBuff;


/**
 * boot/init SIM800 module
 */
void SIM800init(UART_HandleTypeDef *huart)
{
    SIM800_huart = huart;

    // init DMA receive buffer
    initRxBuffer(&respBuff);

    // init received message buff
    _clearReceiveBuff();

    //SIM800Boot();

    TxStatus = SIM800_TX_STATUS_READY;
    isWaitingResponse = 0;

    HAL_StatusTypeDef res = HAL_UART_Receive_DMA(respBuff.huart, (uint8_t*)respBuff.respTmpBuff, respBuff.respTmpBuffSize);
    if (res == HAL_ERROR) {
        Error_Handler();
    }

    // TODO move to GSM listen commands Task
    while(1){
        UART_DMA_Check_Rx_Buff(&respBuff);
    };

    ATResponse_t resp = SIM800_sendAT("AT", 1000);

    if (resp.status == AT_OK) {
        // send PIN if exists

    } else {
        Error_Handler();
    }
}

/**
 * SIM800 boot pin -> GND on 2 seconds
 */
void SIM800Boot()
{
    // boot pin to GND
    HAL_GPIO_WritePin(SIM800_BOOT_GPIO_Port, SIM800_BOOT_Pin, GPIO_PIN_RESET);

    //uint32_t startTick = HAL_GetTick();

    // wait 2 seconds
    osDelay(2000);
    //while ( (HAL_GetTick() - startTick) < 2000 );

    // boot pin to high
    HAL_GPIO_WritePin(SIM800_BOOT_GPIO_Port, SIM800_BOOT_Pin, GPIO_PIN_SET);

    osDelay(1000);
}

/**
 * send AT command to SIM800
 */
ATResponse_t SIM800_sendAT(char* command, uint32_t responseTimeout)
{
    ATResponse_t atresp;
    HAL_StatusTypeDef res;

    atresp.status = AT_NULL;
    atresp.response = "";

    // wait until transmit is ready and if waiting for response
    while (TxStatus == SIM800_TX_STATUS_BUSY || isWaitingResponse == 1);

    TxStatus = SIM800_TX_STATUS_BUSY;

    //strcat(command, "\r\n");

    res = HAL_UART_Transmit_DMA(SIM800_huart, (uint8_t*)command, sizeof(command)-1);

    if (res == HAL_OK) {
        if (responseTimeout > 0) {
            atresp = SIM800_waitResponse(responseTimeout);
        }
    } else {
        Error_Handler();
    }

	return atresp;
}

/**
 * wait for response
 * timeout ms
 * @return ATResponse
 */
ATResponse_t SIM800_waitResponse(uint32_t timeout)
{
    ATResponse_t atresp;
    HAL_StatusTypeDef res;

    isWaitingResponse = true;

    // wait for transmit
    while(TxStatus == SIM800_TX_STATUS_BUSY);

    // milliseconds
    uint32_t startTick = HAL_GetTick();

    // timeout (if 0 - no timeout)
    uint8_t respLines = 0;
    do {
        UART_DMA_Check_Rx_Buff(&respBuff);
    } while (timeout > 0 && (HAL_GetTick() - startTick) < timeout);

    // parse response
    atresp = SIM800_parseResponse(&respBuff.receivedMsgBuff);

    _clearReceiveBuff();

    isWaitingResponse = false;

    return atresp;
}

/**
 * parse response from SIM800
 * @return ATResponse
 */
ATResponse_t SIM800_parseResponse(uint8_t *respBuff)
{
    ATResponse_t atresp;
    atresp.status = AT_UNKNOWN;

    char line[SIM800_Rx_BUFF_SIZE];
    memset(line, 0, SIM800_Rx_BUFF_SIZE);

    atresp.response = respBuff;

    uint16_t j = 0;
    uint16_t lineCounter = 0;
    for(int i = 0; i < sizeof(respBuff); i++){

        // search line
        if (respBuff[i] == '\r') {
            lineCounter++;
            j = 0;

            // TODO parse other responses

            // parse line
            if (_strcmp("OK", line) == 0) {
                atresp.status = AT_OK;
                break;
            }

            // TODO parse first command entry external command AT_EXT_COMMAND
            if (lineCounter == 1 && line[0] == '$' && line[1] == '|') {
                atresp.status = AT_EXT_COMMAND;
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

    for (int i; i < sizeof(str1); i++) {
        if (str1[i] != str2[i]) {
            return (uint8_t)1;
        }
    }

    return (uint8_t)0;
}

static void initRxBuffer(UART_DMA_Rx_t *initBuff)
{
    initBuff->huart = SIM800_huart;
    initBuff->prevCNDTR = SIM800_DMA_BUFF_SIZE;
    initBuff->receivedMsgBuffSize = SIM800_Rx_BUFF_SIZE;
    initBuff->receivedMsgBuff = receivedMsgBuff;
    initBuff->respTmpBuffSize = SIM800_DMA_BUFF_SIZE;
    initBuff->respTmpBuff = respTmpBuff;
}

static void _clearReceiveBuff()
{
    //memset(receivedMsgBuff, (uint8_t)'\0', SIM800_Rx_BUFF_SIZE);
    memset(respBuff.receivedMsgBuff, (uint8_t)'\0', respBuff.receivedMsgBuffSize);
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
    //isWaitingResponse = false;
}
