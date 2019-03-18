/*
 * GSM_module_task.c
 *
 *  Created on: Mar 12, 2019
 *      Author: metalgvc
 */
#include "cmsis_os.h"

#include "../utils/uart_debug.h"
#include "../utils/SIM800.h"

extern UART_HandleTypeDef huart1;

void StartGSMModuleTask(void const * argument)
{

    // init SIM800 module
    SIM800init(&huart1, SIM800_BOOT_PORT, SIM800_BOOT_PIN);

    for(;;) {

        ATResponse atResp = SIM800_waitMessage(&huart1, 1);

        // handle commands
        switch(atResp){
            case AT_OK: handleATOK(); break;
        }
    }
}

void handleATOK()
{
    uartDebugSendStr("handleATOK\n");
}

