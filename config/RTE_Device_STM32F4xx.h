#include "stm32f4xx_gpio.h"

#define USE_RTE_PIN_MAPPING 1

#if defined(USE_RTE_PIN_MAPPING) && (USE_RTE_PIN_MAPPING == 1)
/* USART1 TX pin */
#define RTE_USART1_TX_PORT_ID   0
#if (RTE_USART1_TX_PORT_ID == 0)
#define RTE_USART1_TX_PORT       GPIO_A
#define RTE_USART1_TX_PIN        GPIO9
#define RTE_USART1_TX_MUX        GPIO_ALT_FN7
#elif (RTE_USART1_TX_PORT_ID == 1)
#define RTE_USART1_TX_PORT       GPIO_B
#define RTE_USART1_TX_PIN        GPIO6
#define RTE_USART1_TX_MUX        GPIO_ALT_FN7
#endif

/* USART1 RX pin */
#define RTE_USART1_RX_PORT_ID   0
#if (RTE_USART1_RX_PORT_ID == 0)
#define RTE_USART1_RX_PORT       GPIO_A
#define RTE_USART1_RX_PIN        GPIO10
#define RTE_USART1_RX_MUX        GPIO_ALT_FN7
#elif (RTE_USART1_RX_PORT_ID == 1)
#define RTE_USART1_RX_PORT       GPIO_B
#define RTE_USART1_RX_PIN        GPIO7
#define RTE_USART1_RX_MUX        GPIO_ALT_FN7
#endif

/* USART1 CTS pin */
#define RTE_USART1_CTS_PORT_ID   0
#if (RTE_USART1_CTS_PORT_ID == 0)
#define RTE_USART1_CTS_PORT      GPIO_A
#define RTE_USART1_CTS_PIN       GPIO11
#define RTE_USART1_CTS_MUX       GPIO_ALT_FN7
#endif

/* USART1 RTS pin */
#define RTE_USART1_RTS_PORT_ID   0
#if (RTE_USART1_RTS_PORT_ID == 0)
#define RTE_USART1_RTS_PORT      GPIO_A
#define RTE_USART1_RTS_PIN       GPIO12
#define RTE_USART1_RTS_MUX       GPIO_ALT_FN7
#endif

/* USART1 CLK pin used in synchronous mode */
#define RTE_USART1_CLK_PORT_ID  0
#if (RTE_USART1_CLK_PORT_ID == 0)
#define RTE_USART1_CLK_PORT      GPIO_A
#define RTE_USART1_CLK_PIN       GPIO8
#define RTE_USART1_CLK_MUX       GPIO_ALT_FN7
#endif

/* USART2 TX pin */
#define RTE_USART2_TX_PORT_ID   0
#if (RTE_USART2_TX_PORT_ID == 0)
#define RTE_USART2_TX_PORT       GPIO_A
#define RTE_USART2_TX_PIN        GPIO2
#define RTE_USART2_TX_MUX        GPIO_ALT_FN7
#elif (RTE_USART2_TX_PORT_ID == 1)
#define RTE_USART2_TX_PORT       GPIO_D
#define RTE_USART2_TX_PIN        GPIO5
#define RTE_USART2_TX_MUX        GPIO_ALT_FN7
#endif

/* USART2 TX pin */
#define RTE_USART2_RX_PORT_ID   0
#if (RTE_USART2_RX_PORT_ID == 0)
#define RTE_USART2_RX_PORT       GPIO_A
#define RTE_USART2_RX_PIN        GPIO3
#define RTE_USART2_RX_MUX        GPIO_ALT_FN7
#elif (RTE_USART2_RX_PORT_ID == 1)
#define RTE_USART2_RX_PORT       GPIO_D
#define RTE_USART2_RX_PIN        GPIO6
#define RTE_USART2_RX_MUX        GPIO_ALT_FN7
#endif

/* USART2 CTS pin */
#define RTE_USART2_CTS_PORT_ID   0
#if (RTE_USART2_CTS_PORT_ID == 0)
#define RTE_USART2_CTS_PORT      GPIO_A
#define RTE_USART2_CTS_PIN       GPIO0
#define RTE_USART2_CTS_MUX       GPIO_ALT_FN7
#endif

/* USART2 RTS pin */
#define RTE_USART2_RTS_PORT_ID   0
#if (RTE_USART2_RTS_PORT_ID == 0)
#define RTE_USART2_RTS_PORT      GPIO_A
#define RTE_USART2_RTS_PIN       GPIO1
#define RTE_USART2_RTS_MUX       GPIO_ALT_FN7
#endif

/* USART2 CLK pin used in synchronous mode */
#define RTE_USART2_CLK_PORT_ID  0
#if (RTE_USART2_CLK_PORT_ID == 0)
#define RTE_USART2_CLK_PORT      GPIO_A
#define RTE_USART2_CLK_PIN       GPIO4
#define RTE_USART2_CLK_MUX       GPIO_ALT_FN7
#endif

// USART6 Configuration
/* USART6 TX pin */
#define RTE_USART6_TX_PORT_ID   0
#if (RTE_USART6_TX_PORT_ID == 0)
#define RTE_USART6_TX_PORT       GPIO_C
#define RTE_USART6_TX_PIN        GPIO6
#define RTE_USART6_TX_MUX        GPIO_ALT_FN8
#elif (RTE_USART6_TX_PORT_ID == 1)
#define RTE_USART6_TX_PORT       GPIO_A
#define RTE_USART6_TX_PIN        GPIO11
#define RTE_USART6_TX_MUX        GPIO_ALT_FN8
#endif

// RX Pin
#define RTE_USART6_RX_PORT_ID   0
#if (RTE_USART6_RX_PORT_ID == 0)
#define RTE_USART6_RX_PORT       GPIO_C
#define RTE_USART6_RX_PIN        GPIO7
#define RTE_USART6_RX_MUX        GPIO_ALT_FN8
#elif (RTE_USART6_RX_PORT_ID == 1)
#define RTE_USART6_RX_PORT       GPIO_A
#define RTE_USART6_RX_PIN        GPIO12
#define RTE_USART6_RX_MUX        GPIO_ALT_FN8
#endif

/* USART6 CLK pin used in synchronous mode */
#define RTE_USART6_CLK_PORT_ID  0
#if (RTE_USART6_CLK_PORT_ID == 0)
#define RTE_USART6_CLK_PORT      GPIO_C
#define RTE_USART6_CLK_PIN       GPIO8
#define RTE_USART6_CLK_MUX       GPIO_ALT_FN8
#endif

#endif
