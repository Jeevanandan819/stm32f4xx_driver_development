/******************************************************************************
 * File: [Driver_Name].h
 * Description: Header file for [Peripheral Name] driver.
 * Author: [Your Name]
 * Date: [Date]
 * Version: [Version]
 *
 * Copyright (c) [Year] [Your Organization]. All rights reserved.
 *
 * License: [Specify License Type, e.g., MIT, Apache 2.0]
 *
 * Notes:
 * - Defines macros and function prototypes.
 * - Provides register addresses and configuration settings.
 * - Ensures compatibility with STM32 architecture.
 ******************************************************************************/

#ifndef STM32F4XX_STARTUP_H
#define STM32F4XX_STARTUP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32f4xx.h"

/* Macro Definitions */
typedef void (*vector_handler)(void);

/* Function Prototypes */
void Default_Handler(void);

/* Vector Prototypes */
void Reset_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void WWDG_IRQHandler(void);              			/* Window Watchdog interrupt                                          */
void PVD_IRQHandler(void);               			/* EXTI Line 16 interrupt / PVD through EXTI                          */
void TAMP_STAMP_IRQHandler(void);        			/* Tamper and TimeStamp interrupts through                            */
void RTC_WKUP_IRQHandler(void);          			/* RTC Wakeup interrupt through the EXTI line                         */
void FLASH_IRQHandler(void);             			/* FLASH global interrupt                                             */
void RCC_IRQHandler(void);               			/* RCC global interrupt                                               */
void EXTI0_IRQHandler(void);             			/* EXTI Line0 interrupt                                               */
void EXTI1_IRQHandler(void);             			/* EXTI Line1 interrupt                                               */
void EXTI2_IRQHandler(void);             			/* EXTI Line2 interrupt                                               */
void EXTI3_IRQHandler(void);             			/* EXTI Line3 interrupt                                               */
void EXTI4_IRQHandler(void);             			/* EXTI Line4 interrupt                                               */
void DMA1_Stream0_IRQHandler(void);      			/* DMA1 Stream0 global interrupt                                      */
void DMA1_Stream1_IRQHandler(void);      			/* DMA1 Stream1 global interrupt                                      */
void DMA1_Stream2_IRQHandler(void);      			/* DMA1 Stream2 global interrupt                                      */
void DMA1_Stream3_IRQHandler(void);      			/* DMA1 Stream3 global interrupt                                      */
void DMA1_Stream4_IRQHandler(void);      			/* DMA1 Stream4 global interrupt                                      */
void DMA1_Stream5_IRQHandler(void);      			/* DMA1 Stream5 global interrupt                                      */
void DMA1_Stream6_IRQHandler(void);      			/* DMA1 Stream6 global interrupt                                      */
void ADC_IRQHandler(void);               			/* ADC1 global interrupt                                              */
void EXTI9_5_IRQHandler(void);           			/* EXTI Line[9:5] interrupts                                          */
void TIM1_BRK_TIM9_IRQHandler(void);     			/* TIM1 Break interrupt and TIM9 global interrupt                     */
void TIM1_UP_TIM10_IRQHandler(void);     			/* TIM1 Update interrupt and TIM10 global interrupt                   */
void TIM1_TRG_COM_TIM11_IRQHandler(void);			/* TIM1 Trigger and Commutation interrupts and TIM11 global interrupt */
void TIM1_CC_IRQHandler(void);           			/* TIM1 Capture Compare interrupt                                     */
void TIM2_IRQHandler(void);              			/* TIM2 global interrupt                                              */
void TIM3_IRQHandler(void);              			/* TIM3 global interrupt                                              */
void TIM4_IRQHandler(void);              			/* TIM4 global interrupt                                              */
void I2C1_EV_IRQHandler(void);           			/* I2C1 event interrupt                                               */
void I2C1_ER_IRQHandler(void);           			/* I2C1 error interrupt                                               */
void I2C2_EV_IRQHandler(void);           			/* I2C2 event interrupt                                               */
void I2C2_ER_IRQHandler(void);           			/* I2C2 error interrupt                                               */
void SPI1_IRQHandler(void);              			/* SPI1 global interrupt                                              */
void SPI2_IRQHandler(void);              			/* SPI2 global interrupt                                              */
void USART1_IRQHandler(void);            			/* USART1 global interrupt                                            */
void USART2_IRQHandler(void);            			/* USART2 global interrupt                                            */
void EXTI15_10_IRQHandler(void);         			/* EXTI Line[15:10] interrupts                                        */
void RTC_Alarm_IRQHandler(void);         			/* RTC Alarms (A and B) through EXTI line interrupt                   */
void OTG_FS_WKUP_IRQHandler(void);       			/* USB On-The-Go FS Wakeup through EXTI line interrupt                */
void DMA1_Stream7_IRQHandler(void);      			/* DMA1 Stream7 global interrupt                                      */
void SDIO_IRQHandler(void);              			/* SDIO global interrupt                                              */
void TIM5_IRQHandler(void);              			/* TIM5 global interrupt                                              */
void SPI3_IRQHandler(void);              			/* SPI3 global interrupt                                              */
void DMA2_Stream0_IRQHandler(void);      			/* DMA2 Stream0 global interrupt                                      */
void DMA2_Stream1_IRQHandler(void);      			/* DMA2 Stream1 global interrupt                                      */
void DMA2_Stream2_IRQHandler(void);      			/* DMA2 Stream2 global interrupt                                      */
void DMA2_Stream3_IRQHandler(void);      			/* DMA2 Stream3 global interrupt                                      */
void DMA2_Stream4_IRQHandler(void);      			/* DMA2 Stream4 global interrupt                                      */
void OTG_FS_IRQHandler(void);            			/* USB On The Go FS global interrupt                                  */
void DMA2_Stream5_IRQHandler(void);      			/* DMA2 Stream5 global interrupt                                      */
void DMA2_Stream6_IRQHandler(void);      			/* DMA2 Stream6 global interrupt                                      */
void DMA2_Stream7_IRQHandler(void);      			/* DMA2 Stream7 global interrupt                                      */
void USART6_IRQHandler(void);            			/* USART6 global interrupt                                            */
void I2C3_EV_IRQHandler(void);           			/* I2C3 event interrupt                                               */
void I2C3_ER_IRQHandler(void);           			/* I2C3 error interrupt                                               */
void SPI4_IRQHandler(void);              			/* SPI 4 global interrupt                                             */
void SPI5_IRQHandler(void);              			/* SPI 5 global interrupt                                             */

#ifdef __cplusplus
}
#endif

#endif /* [DRIVER_NAME]_H */