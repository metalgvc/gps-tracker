/*
 * GP4SF1109F2.h
 *
 *  Created on: Mar 17, 2019
 *      Author: metalgvc
 */

#ifndef UTILS_GP4SF1109F2_H_
#define UTILS_GP4SF1109F2_H_

#include "main.h"
#include "stdint.h"
#include "cmsis_os.h"
#include "stdio.h"

#include "../utils/SDcard.h"

typedef struct {
    uint16_t msgBuffSize;   // msg buff size
    uint8_t *msg;           // ptr to msg
    uint8_t isMsgReceived;  // 1 or 0
} GPS_Msg_t;

void GPSinit(UART_HandleTypeDef *huart);
GPS_Msg_t* GPS_waitMessage(uint32_t timeout);
void GPS_Rx_callback(UART_HandleTypeDef *huart);

#endif /* UTILS_GP4SF1109F2_H_ */
