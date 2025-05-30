/******************************************************************************
 * File: stm32f4xx_startup.c
 * Description: Implementation of startup code for STM32F411xx.
 * Author: Jeevanandan Sandan
 * Date: 29-05-2025
 * Version: 0.1
 *
 * Copyright (c) 2025 Jeevanandan Sandan . All rights reserved.
 *
 * License:
 *
 * Notes:
 * - Initializes System from ResetHandler after booting.
******************************************************************************/

#include <stddef.h>
#include "stm32f4xx_startup.h"
#include "stm32f4xx.h"

extern void _estack(void);
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _sidata;
extern int main(void);

__attribute__((used, section("isr_vector"))) 
const vector_handler vector_table[] = {
    _estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    NULL,
    NULL,
    NULL,
    NULL,
    SVC_Handler,
    DebugMon_Handler,
    NULL,
    PendSV_Handler,
    SysTick_Handler,
    WWDG_IRQHandler,              			
    PVD_IRQHandler,               			
    TAMP_STAMP_IRQHandler,        			
    RTC_WKUP_IRQHandler,          			
    FLASH_IRQHandler,             			
    RCC_IRQHandler,               			
    EXTI0_IRQHandler,             			
    EXTI1_IRQHandler,             			
    EXTI2_IRQHandler,             			
    EXTI3_IRQHandler,             			
    EXTI4_IRQHandler,             			
    DMA1_Stream0_IRQHandler,      			
    DMA1_Stream1_IRQHandler,      			
    DMA1_Stream2_IRQHandler,      			
    DMA1_Stream3_IRQHandler,      			
    DMA1_Stream4_IRQHandler,      			
    DMA1_Stream5_IRQHandler,      			
    DMA1_Stream6_IRQHandler,      			
    ADC_IRQHandler,               			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    EXTI9_5_IRQHandler,           			
    TIM1_BRK_TIM9_IRQHandler,     			
    TIM1_UP_TIM10_IRQHandler,     			
    TIM1_TRG_COM_TIM11_IRQHandler,			
    TIM1_CC_IRQHandler,           			
    TIM2_IRQHandler,              			
    TIM3_IRQHandler,              			
    TIM4_IRQHandler,              			
    I2C1_EV_IRQHandler,           			
    I2C1_ER_IRQHandler,           			
    I2C2_EV_IRQHandler,           			
    I2C2_ER_IRQHandler,           			
    SPI1_IRQHandler,              			
    SPI2_IRQHandler,              			
    USART1_IRQHandler,            			
    USART2_IRQHandler,            			
    NULL,                            			
    EXTI15_10_IRQHandler,         			
    RTC_Alarm_IRQHandler,         			
    OTG_FS_WKUP_IRQHandler,       			 
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,
    DMA1_Stream7_IRQHandler,      			
    NULL,                            			
    SDIO_IRQHandler,              			
    TIM5_IRQHandler,              			
    SPI3_IRQHandler,              			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    DMA2_Stream0_IRQHandler,      			
    DMA2_Stream1_IRQHandler,      			
    DMA2_Stream2_IRQHandler,      			
    DMA2_Stream3_IRQHandler,      			
    DMA2_Stream4_IRQHandler,      			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    OTG_FS_IRQHandler,            			
    DMA2_Stream5_IRQHandler,      			
    DMA2_Stream6_IRQHandler,      			
    DMA2_Stream7_IRQHandler,      			
    USART6_IRQHandler,            			
    I2C3_EV_IRQHandler,           			
    I2C3_ER_IRQHandler,           			 
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    NULL,                            			
    SPI4_IRQHandler,
    SPI5_IRQHandler
};

void Default_Handler(void)
{
    while(1);
}

void Reset_Handler(void)
{
    uint32_t *src, *dest;
    src = &_sidata;
    dest = &_sdata;
    while (dest < &_edata) {
        *(dest++) = *(src++);
    }

    // Zero out the .bss section
    dest = &_sbss;
    while (dest < &_ebss) {
        *(dest++) = 0;
    }
    main();
}