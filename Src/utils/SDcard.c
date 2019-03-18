/*
 * SDcard.c
 *
 *  Created on: Mar 12, 2019
 *      Author: metalgvc
 */

#include "fatfs.h"



void SDcard_init(void){

}

void SDcard_test()
{
	MX_FATFS_Init();

	FATFS fs;
	FIL fil;
	int frCode;

	// mount SD
	frCode = f_mount(&fs, "", 1);
	if (frCode != FR_OK) {
	  Error_Handler(__FILE__, __LINE__);
	}

	// open file
	frCode = f_open(&fil, "test.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	if (frCode != FR_OK) {
	  Error_Handler(__FILE__, __LINE__);
	}

	// write
	frCode = f_puts("test str\n", &fil);
	if (frCode != FR_OK) {
	  Error_Handler(__FILE__, __LINE__);
	}

	// close file
	frCode = f_close(&fil);
	if (frCode != FR_OK) {
	  Error_Handler(__FILE__, __LINE__);
	}

	// unmount SD
	frCode = f_mount(NULL, "", 1);
	if (frCode != FR_OK) {
	  Error_Handler(__FILE__, __LINE__);
	}
}
