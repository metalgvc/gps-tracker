

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim1;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

void USART1_IRQHandler(void) {
  HAL_UART_IRQHandler(&huart1);
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void) {

  HAL_UART_IRQHandler(&huart3);

}


/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void) {

}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) {

  while (1) {

  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void) {

  while (1) {

  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void) {

  while (1) {

  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void) {

  while (1) {

  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{

}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void) {

  HAL_TIM_IRQHandler(&htim1);

}
