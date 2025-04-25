/**
 * @file stm32f4xx_usart.h
 * @brief This header file contains the declarations of USART peripheral API prototypes.
 *
 * @author Jeevanandan Sandan
 * @date April 25, 2025
 *
 * This file contains the declarations of USART peripheral functionality and 
 * is maintained by Team Alpha. For queries or issues, contact the code owner.
 */

typedef enum {
    USART_MODE_ASYNCHRONOUS,
    USART_MODE_SYNCHRONOUS_MASTER,
    USART_MODE_SYNCHRONOUS_SLAVE,
    USART_MODE_SINGLE_WIRE,
    USART_MODE_IRDA,
    USART_MODE_LIN,
    USART_MODE_SMART_CARD,
    USART_MODE_LAST
} st_usart_mode_t;

typedef enum {
    USART_PARITY_EVEN,
    USART_PARITY_ODD,
    USART_PARITY_LAST
} st_usart_parity_t;

typedef enum {
    USART_STOP_BIT_1,
    USART_STOP_BIT_0_5,
    USART_STOP_BIT_2,
    USART_STOP_BIT_1_5
} st_usart_stop_bit_t;

typedef enum {
    USART_CPOL0_CPHA0,
    USART_CPOL0_CPHA1,
    USART_CPOL1_CPHA0,
    USART_CPOL1_CPHA1
} st_usart_clock_mode_t;



 