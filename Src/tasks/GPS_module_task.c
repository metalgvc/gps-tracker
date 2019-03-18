/*
 * GPS_module_task.c
 *
 *  Created on: Mar 17, 2019
 *      Author: metalgvc
 */

#include "GPS_module_task.h"

extern UART_HandleTypeDef huart3;

void StartGPSModuleTask(void const *argument)
{
    GPSinit();

    for(;;){

    }
}
