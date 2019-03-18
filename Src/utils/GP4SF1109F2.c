/*
 * GP4SF1109F2.c
 *
 *  Created on: Mar 17, 2019
 *      Author: metalgvc
 */

#include "GP4SF1109F2.h"

uint8_t respTmpBuff[100];       // uart message buff
uint8_t lastCorrectMsg[100];    // last correct message for send on SMS command trigger

void GPSinit()
{
    memset(respTmpBuff, 0, sizeof(respTmpBuff));
    memset(lastCorrectMsg, 0, sizeof(lastCorrectMsg));
}

/**
 * wait data from GPS module & save into respTmpBuff
 * @return link to respTmpBuff
 */
char* GPS_waitAndHandleMessage(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef res = HAL_UART_Receive_IT(huart, (uint8_t*)respTmpBuff, sizeof(respTmpBuff));

    // TODO check end of respTmpBuff and save to SD



    memset(respTmpBuff, 0, sizeof(respTmpBuff));

    return respTmpBuff;
}

/**
 * save data (coordinate message from GPS module) to SD
 */
void GPS_saveMessage(char *tmpBuff)
{
    // TODO save tmpBuff to SD card
}

/**
 * get last data(coordinate message) from SD
 */
char* GPS_getLastCorrectMessage()
{
    memset(lastCorrectMsg, 0, sizeof(lastCorrectMsg));

    // TODO read last data from SD to lastCorrectMsg & return link to lastCorrectMsg

    return lastCorrectMsg;
}

/**
 * uart Rx done callback
 */
void GPS_Rx_callback(UART_HandleTypeDef *huart)
{

}
