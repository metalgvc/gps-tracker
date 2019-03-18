
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1xx_IT_H
#define __STM32F1xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 


/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void TIM1_UP_IRQHandler(void);

void USART1_IRQHandler(void);
//void USART2_IRQHandler(void);
void USART3_IRQHandler(void);


#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
