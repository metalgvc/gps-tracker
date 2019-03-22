/*
 * SDcard.c
 *
 *  Created on: Mar 12, 2019
 *      Author: metalgvc
 */

#include "fatfs.h"

FATFS fs;
FIL fil;
FRESULT frCode;

void SDcard_init(void){
    MX_FATFS_Init();

    HAL_Delay(1000);

    // mount SD
    frCode = f_mount(&fs, "", 1);
    if (frCode != FR_OK) {
      Error_Handler();
    }
}

FRESULT SDCard_save(char* filename, char* data)
{
    // open file
    frCode = f_open(&fil, *filename, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
    if (frCode == FR_NO_FILE) {
        frCode = f_open(&fil, *filename , FA_CREATE_ALWAYS | FA_WRITE);
    } else if (frCode != FR_OK) {
      Error_Handler();
    }

    // write
    frCode = f_puts(*data, &fil);
    if (frCode != FR_OK) {
      Error_Handler();
    }

    // close file
    frCode = f_close(&fil);
    if (frCode != FR_OK) {
      Error_Handler();
    }

    return frCode;
}



void SDcard_test()
{
	MX_FATFS_Init();

	// mount SD
	frCode = f_mount(&fs, "", 1);
	if (frCode != FR_OK) {
	  Error_Handler();
	}

	// open file
	frCode = f_open(&fil, "test.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if (frCode != FR_OK) {
	  Error_Handler();
	}

	// write
	frCode = f_puts("test str\n", &fil);
	if (frCode != FR_OK) {
	  Error_Handler();
	}

	// close file
	frCode = f_close(&fil);
	if (frCode != FR_OK) {
	  Error_Handler();
	}

	// unmount SD
	frCode = f_mount(NULL, "", 1);
	if (frCode != FR_OK) {
	  Error_Handler();
	}
}
