/*
 * GP4SF1109F2.c
 *
 *  Created on: Mar 17, 2019
 *      Author: metalgvc
 */

#include "GP4SF1109F2.h"

#define GPS_UART_DMA_Rx_BUFF_SIZE 256
#define GPS_MSG_BUFF_SIZE 256

UART_DMA_Rx_t respBuff;
GPS_Msg_t msgBuff;

uint8_t _msgBuff[GPS_MSG_BUFF_SIZE];
uint8_t _respTmpBuff[GPS_UART_DMA_Rx_BUFF_SIZE];
uint8_t _parsedMsgBuff[GPS_MSG_BUFF_SIZE];
UART_HandleTypeDef *GPShuart;

static void _initReceiveBuff();
static void _initMsgBuff();
static void _clearReceiveBuff();

// TODO remove
extern UART_HandleTypeDef huart1;


void GPSinit(UART_HandleTypeDef *huart)
{
    GPShuart = huart;

    // init DMA Rx buffers
    _initReceiveBuff();

    // init parsed msg buff
    _initMsgBuff();

    // start UART DMA to receive messages
    HAL_StatusTypeDef res = HAL_UART_Receive_DMA(respBuff.huart, (uint8_t*)respBuff.respTmpBuff, respBuff.respTmpBuffSize);

    if (res == HAL_ERROR) {
        Error_Handler();
    }


}

/**
 * wait data from GPS module
 * @return link to respTmpBuff
 */
GPS_Msg_t* GPS_waitMessage(uint32_t timeout)
{
    uint16_t i = 0, j = 0;
    uint32_t startTick = HAL_GetTick();
    uint8_t msgFound = 0;
    uint32_t currTick;

    // reset msg
    for (i = 0; i < msgBuff.msgBuffSize; i++) {
        msgBuff.msg[i] = '\0';
    }
    msgBuff.isMsgReceived = 0;

    do {
        UART_DMA_Check_Rx_Buff(&respBuff);

        for (i = 0; i < respBuff.receivedMsgBuffSize; i++) {

            // found start msg
            if (msgBuff.msg[0] == '\0' && respBuff.receivedMsgBuff[i] == '$'){
                msgBuff.msg[j++] = respBuff.receivedMsgBuff[i];

            // found end msg
            } else if (msgBuff.msg[0] != '\0' && respBuff.receivedMsgBuff[i] == '\r') {
                msgFound = 1;
                break;

            // copy message
            } else if (msgBuff.msg[0] != '\0' && respBuff.receivedMsgBuff[i] != '\0') {
                msgBuff.msg[j++] = respBuff.receivedMsgBuff[i];
            }
        }

        _clearReceiveBuff();

        // message found
        if (msgFound == 1) { break; }

        currTick = HAL_GetTick();
    } while( (currTick - startTick) < timeout);

    if (msgFound != 1) {
        // reset msg
        for (i = 0; i < msgBuff.msgBuffSize; i++) { msgBuff.msg[i] = '\0'; }
    } else {
        msgBuff.isMsgReceived = 1;
    }

    return &msgBuff;
}

static void _initReceiveBuff()
{
    respBuff.huart = GPShuart;
    respBuff.prevCNDTR = GPS_UART_DMA_Rx_BUFF_SIZE;
    respBuff.receivedMsgBuff = _msgBuff;
    respBuff.receivedMsgBuffSize = GPS_MSG_BUFF_SIZE;
    respBuff.respTmpBuff = _respTmpBuff;
    respBuff.respTmpBuffSize = GPS_UART_DMA_Rx_BUFF_SIZE;

    _clearReceiveBuff();
}

static void _initMsgBuff()
{
    msgBuff.msgBuffSize = GPS_MSG_BUFF_SIZE;
    msgBuff.msg = _parsedMsgBuff;
    msgBuff.isMsgReceived = 0;
}

static void _clearReceiveBuff()
{
    memset(respBuff.receivedMsgBuff, (uint8_t)'\0', respBuff.receivedMsgBuffSize);
}

/**
 * uart Rx done callback
 */
void GPS_Rx_callback(UART_HandleTypeDef *huart)
{

}
