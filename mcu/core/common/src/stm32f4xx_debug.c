#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_debug.h"

#define UART_DEBUG_INSTANCE USART_2

/**
 * @brief  Initializes the debug logger (e.g., UART/SWO configuration).
 * @note   Call this once during system initialization.
 */
st_status_t st_debug_init(void)
{
    st_status_t status = ST_STATUS_OK;
    st_usart_config_t debug_uart_config = {
        .instance = UART_DEBUG_INSTANCE,
        .baudrate = 115200,
        .clock_mode = USART_CPOL0_CPHA0,
        .mode = USART_MODE_ASYNCHRONOUS,
        .stop_bits = USART_STOP_BIT_1,
        .parity = USART_PARITY_NONE,
        .is_flow_control_enable = false,
        .oversampling = USART_OVERSAMPLING_8
    };
    st_usart_io_t debug_uart_io;
    do {
        GPIOA_PERI_CLK_EN();
        status = st_usart_init(UART_DEBUG_INSTANCE);
        if (status != ST_STATUS_OK) {
            break;
        }
        status = st_usart_set_configuration(&debug_uart_config, &debug_uart_io);
        if (status != ST_STATUS_OK) {
            break;
        }
    } while (false);
    return status;
}

void st_debug_log_send_str(const char *str)
{
    if (str != NULL) {
        st_usart_send_data_blocking(UART_DEBUG_INSTANCE, (uint8_t*)str, strlen(str));
    }
}

/**
 * @brief  Prints a formatted debug message (like printf).
 * @param  fmt: Format string
 * @param  ...: Variable arguments
 */
void st_debug_log_printf(const char *fmt, ...)
{
    char debug_str[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(debug_str, sizeof(debug_str), fmt, args);
    va_end(args);

    st_debug_log_send_str(debug_str);
}