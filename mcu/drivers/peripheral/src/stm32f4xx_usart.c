/**
 * @file stm32f4xx_usart.c
 * @brief Example source file contains the definitions of USART peripheral API prototyping.
 *
 * @author Jeevanandan Sandan
 * @date April 25, 2025
 *
 * This file contains the implementation of USART peripheral functionality and
 * is maintained by Team Alpha. For queries or issues, contact the code owner.
 */

#include "stm32f4xx_usart.h"

static st_usart_callback_t usart_callbacks[] = {NULL, NULL, NULL};
static void sti_usart_clock_control(st_usart_instance_t instance, uint8_t enable);
static USART_TypeDef *sti_get_usart_base_address(st_usart_instance_t instance);

/**
 * @brief Initialize the specified USART instance.
 *
 * This function performs basic initialization for the given USART instance.
 *
 * @param instance The USART instance to initialize.
 *
 * @return st_status_t status code indicating success or error.
 */
st_status_t st_usart_init(st_usart_instance_t instance)
{
    sti_usart_clock_control(instance, ENABLE);
    return ST_STATUS_OK;
}

/**
 * @brief Set the configuration for the USART.
 *
 * Configures the USART instance with user-specified settings such as baud rate,
 * mode, parity, stop bits, clock mode, and flow control.
 *
 * @param config Pointer to a st_usart_config_t structure containing the configuration.
 *
 * @return st_status_t status code indicating success or error.
 */
st_status_t st_usart_set_configuration(st_usart_config_t *config)
{
    if (config == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }
    USART_TypeDef *pUSART = sti_get_usart_base_address(config->instance);
    if (pUSART == NULL || config->clock_mode >= USART_CLOCK_MODE_LAST ||
        config->mode >= USART_MODE_LAST || config->parity >= USART_PARITY_LAST)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }

    if ((config->mode == USART_MODE_SYNCHRONOUS_MASTER || config->mode == USART_MODE_SYNCHRONOUS_SLAVE) && (config->is_flow_control_enable))
    {
        return ST_STATUS_INVALID_PARAMETER;
    }
    return ST_STATUS_OK;
}

/**
 * @brief Register a callback function for USART events.
 *
 * Associates a user-defined callback function with a USART instance. The callback
 * is invoked when an event occurs.
 *
 * @param instance The USART instance for which to register the callback.
 * @param callback The function pointer to the callback function.
 *
 * @return st_status_t status code indicating success or error.
 */
st_status_t st_usart_register_callback(st_usart_instance_t instance, st_usart_callback_t callback)
{
    return ST_STATUS_OK;
}

/**
 * @brief Send data in blocking mode.
 *
 * Transmits data over the specified USART instance in a blocking manner, i.e., the
 * function will not return until all data has been sent.
 *
 * @param instance The USART instance to use for transmission.
 * @param tx_buf Pointer to the buffer containing data to be sent.
 * @param tx_len Length of the data in bytes.
 *
 * @return st_status_t status code indicating success or error.
 */
st_status_t st_usart_send_data_blocking(st_usart_instance_t instance, uint8_t *tx_buf, uint16_t tx_len)
{
    return ST_STATUS_OK;
}

/**
 * @brief Receive data in blocking mode.
 *
 * Receives data over the specified USART instance in a blocking manner. The function
 * waits until the specified amount of data is received.
 *
 * @param instance The USART instance to use for reception.
 * @param rx_buf Pointer to the buffer where the received data will be stored.
 * @param rx_len Length of the data expected.
 *
 * @return st_status_t status code indicating success or error.
 */
st_status_t st_usart_receive_data_blocking(st_usart_instance_t instance, uint8_t *rx_buf, uint16_t rx_len)
{
    return ST_STATUS_OK;
}

/**
 * @brief Send data in non-blocking mode.
 *
 * Initiates a non-blocking data transmission over the specified USART instance.
 * The function returns immediately while transmission is handled in the background.
 *
 * @param instance The USART instance to use for transmission.
 * @param tx_buf Pointer to the buffer containing data to be sent.
 * @param tx_len Length of the data in bytes.
 *
 * @return st_status_t status code indicating success or error.
 */
st_status_t st_usart_send_data_non_blocking(st_usart_instance_t instance, uint8_t *tx_buf, uint16_t tx_len)
{
    return ST_STATUS_OK;
}

/**
 * @brief Receive data in non-blocking mode.
 *
 * Initiates a non-blocking data reception over the specified USART instance.
 * The function returns immediately while reception is handled asynchronously.
 *
 * @param instance The USART instance to use for reception.
 * @param rx_buf Pointer to the buffer where the received data will be stored.
 * @param rx_len Length of the data expected.
 *
 * @return st_status_t status code indicating success or error.
 */
st_status_t st_usart_receive_data_non_blocking(st_usart_instance_t instance, uint8_t *rx_buf, uint16_t rx_len)
{
    return ST_STATUS_OK;
}

/**
 * @brief De-initialize the specified USART instance.
 *
 * Resets the USART configuration and releases any resources allocated during
 * initialization.
 *
 * @param instance The USART instance to de-initialize.
 *
 * @return st_status_t status code indicating success or error.
 */
st_status_t st_usart_deinit(st_usart_instance_t instance)
{
    return ST_STATUS_OK;
}

static void sti_usart_clock_control(st_usart_instance_t instance, uint8_t enable)
{
    switch (instance)
    {
    case USART_1:
        if (enable)
            RCC_USART1_PERI_CLK_EN();
        else
            RCC_USART1_PERI_CLK_DIS();
        break;
    case USART_2:
        if (enable)
            RCC_USART2_PERI_CLK_EN();
        else
            RCC_USART2_PERI_CLK_DIS();
        break;
    case USART_6:
        if (enable)
            RCC_USART6_PERI_CLK_EN();
        else
            RCC_USART6_PERI_CLK_DIS();
        break;
    default:
        break;
    }
}

static USART_TypeDef *sti_get_usart_base_address(st_usart_instance_t instance)
{
    USART_TypeDef *pUSART = NULL;
    switch (instance)
    {
    case USART_1:
        pUSART = USART1;
        break;
    case USART_2:
        pUSART = USART2;
        break;
    case USART_6:
        pUSART = USART6;
        break;
    default:
        break;
    }
    return pUSART;
}