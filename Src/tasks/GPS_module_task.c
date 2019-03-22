/*
 * GPS_module_task.c
 *
 *  Created on: Mar 17, 2019
 *      Author: metalgvc
 */

#include "GPS_module_task.h"

extern UART_HandleTypeDef huart3;
extern char DATE;

void StartGPSModuleTask(void const *argument)
{
    GPSinit(&huart3);

    GPS_Msg_t *GPSMsg;

    for(;;){
        GPSMsg = GPS_waitMessage(1500);

        //if (msg[0] != '\0') {
            // TODO check if msg has coord then save
            //SDCard_save(&DATE, GPSMsg->msg);
        //}
    }
}
