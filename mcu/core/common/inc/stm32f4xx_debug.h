/**
 * @file    stm32f4xx_debug.h
 * @author  Jeevanandan Sandan
 * @brief   Header file for Debug Logging Utilities for STM32F4xx MCU.
 *
 *          This module provides simple macros and functions to print
 *          debug information over UART or SWO during development.
 *
 *          Features:
 *          - Configurable log levels (ERROR, WARNING, INFO, DEBUG)
 *          - Lightweight and easy integration
 *          - Optional compile-time enable/disable of logging
 **/

#include <stdio.h> // For printf (optional)
#include "st_status.h"
#include "stm32f4xx_usart.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief Log levels
 */
typedef enum
{
    LOG_LEVEL_NONE = 0U, /*!< No log output */
    LOG_LEVEL_ERROR,     /*!< Critical errors only */
    LOG_LEVEL_WARNING,   /*!< Warnings and recoverable errors */
    LOG_LEVEL_INFO,      /*!< General informational messages */
    LOG_LEVEL_DEBUG      /*!< Detailed debug messages */
} st_debug_log_level_t;

/* User-configurable: Enable or disable logging globally */
#define DEBUG_LOG_ENABLED 1

/* User-configurable: Default log level */
#define DEBUG_LOG_LEVEL LOG_LEVEL_DEBUG

#if DEBUG_LOG_ENABLED

/* Low-level macro to send a string (implementation depends on your platform) */
#define DEBUG_LOG_RAW(str) st_debug_log_send_str(str)

/* High-level log macros */
#define LOG_ERROR(fmt, ...)                                            \
    do                                                                 \
    {                                                                  \
        if (DEBUG_LOG_LEVEL >= LOG_LEVEL_ERROR)                        \
        {                                                              \
            st_debug_log_printf("[ERROR] " fmt "\r\n", ##__VA_ARGS__); \
        }                                                              \
    } while (0)
#define LOG_WARNING(fmt, ...)                                            \
    do                                                                   \
    {                                                                    \
        if (DEBUG_LOG_LEVEL >= LOG_LEVEL_WARNING)                        \
        {                                                                \
            st_debug_log_printf("[WARNING] " fmt "\r\n", ##__VA_ARGS__); \
        }                                                                \
    } while (0)
#define LOG_INFO(fmt, ...)                                            \
    do                                                                \
    {                                                                 \
        if (DEBUG_LOG_LEVEL >= LOG_LEVEL_INFO)                        \
        {                                                             \
            st_debug_log_printf("[INFO] " fmt "\r\n", ##__VA_ARGS__); \
        }                                                             \
    } while (0)
#define LOG_DEBUG(fmt, ...)                                            \
    do                                                                 \
    {                                                                  \
        if (DEBUG_LOG_LEVEL >= LOG_LEVEL_DEBUG)                        \
        {                                                              \
            st_debug_log_printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__); \
        }                                                              \
    } while (0)

#else /* DEBUG_LOG_ENABLED == 0 */

#define DEBUG_LOG_RAW(str) ((void)0)
#define LOG_ERROR(fmt, ...) ((void)0)
#define LOG_WARNING(fmt, ...) ((void)0)
#define LOG_INFO(fmt, ...) ((void)0)
#define LOG_DEBUG(fmt, ...) ((void)0)

#endif /* DEBUG_LOG_ENABLED */

/* Exported functions -------------------------------------------------------*/

/**
 * @brief  Initializes the debug logger (e.g., UART/SWO configuration).
 * @note   Call this once during system initialization.
 */
st_status_t st_debug_init(void);

/**
 * @brief  Sends a null-terminated string over the debug interface.
 * @param  str: Pointer to the string to send.
 */
void st_debug_log_send_str(const char *str);

/**
 * @brief  Prints a formatted debug message (like printf).
 * @param  fmt: Format string
 * @param  ...: Variable arguments
 */
void st_debug_log_printf(const char *fmt, ...);
