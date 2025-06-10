/**
 * @file stm32f4xx_usart.c
 * @brief The source file contains the definitions of USART peripheral API implementation.
 *
 * @author Jeevanandan Sandan
 * @date April 25, 2025
 *
 * This file contains the implementation of USART peripheral functionality and
 * is maintained by Team Alpha. For queries or issues, contact the code owner.
 */

#include "stm32f4xx_usart.h"
#include <string.h>

#define USART_PERI_CLK 16000000UL

static void sti_usart_clock_control(st_usart_instance_t instance, uint8_t enable);
static USART_TypeDef *sti_get_usart_base_address(st_usart_instance_t instance);
static void sti_usart_set_baudrate(USART_TypeDef *pUSART, uint32_t baudrate, st_usart_oversampling_t oversampling);
static bool validate_usart_handle(const st_usart_handle_t *handle);
static void sti_usart_nvic_intr_enable(st_usart_instance_t instance, uint8_t enable);

#if defined(USE_RTE_PIN_MAPPING) && (USE_RTE_PIN_MAPPING == 1)
static st_usart_io_t usart1_io = {
    .tx = {
        .pin = RTE_USART1_TX_PIN,
        .port = RTE_USART1_TX_PORT,
        .alt_fn = RTE_USART1_TX_MUX},
    .rx = {.pin = RTE_USART1_RX_PIN, .port = RTE_USART1_RX_PORT, .alt_fn = RTE_USART1_RX_MUX},
    .cts = {.pin = RTE_USART1_CTS_PIN, .port = RTE_USART1_CTS_PORT, .alt_fn = RTE_USART1_CTS_MUX},
    .rts = {.pin = RTE_USART1_RTS_PIN, .port = RTE_USART1_RTS_PORT, .alt_fn = RTE_USART1_RTS_MUX},
    .clk = {.pin = RTE_USART1_CLK_PIN, .port = RTE_USART1_CLK_PORT, .alt_fn = RTE_USART1_CLK_MUX}};

static st_usart_io_t usart2_io = {
    .tx = {
        .pin = RTE_USART2_TX_PIN,
        .port = RTE_USART2_TX_PORT,
        .alt_fn = RTE_USART2_TX_MUX},
    .rx = {.pin = RTE_USART2_RX_PIN, .port = RTE_USART2_RX_PORT, .alt_fn = RTE_USART2_RX_MUX},
    .cts = {.pin = RTE_USART2_CTS_PIN, .port = RTE_USART2_CTS_PORT, .alt_fn = RTE_USART2_CTS_MUX},
    .rts = {.pin = RTE_USART2_RTS_PIN, .port = RTE_USART2_RTS_PORT, .alt_fn = RTE_USART2_RTS_MUX},
    .clk = {.pin = RTE_USART2_CLK_PIN, .port = RTE_USART2_CLK_PORT, .alt_fn = RTE_USART2_CLK_MUX}};

static st_usart_io_t usart6_io = {
    .tx = {
        .pin = RTE_USART6_TX_PIN,
        .port = RTE_USART6_TX_PORT,
        .alt_fn = RTE_USART6_TX_MUX},
    .rx = {.pin = RTE_USART6_RX_PIN, .port = RTE_USART6_RX_PORT, .alt_fn = RTE_USART6_RX_MUX},
    .clk = {.pin = RTE_USART6_CLK_PIN, .port = RTE_USART6_CLK_PORT, .alt_fn = RTE_USART6_CLK_MUX}};
#endif

/**
 * @brief Initialize the specified USART instance.
 *
 * This function performs basic initialization for the given USART instance.
 *
 * @param instance The USART instance to initialize.
 *
 * @return st_status_t status code indicating success or error.
 */
st_status_t st_usart_init(st_usart_instance_t instance, st_usart_handle_t *handle)
{
    if (handle == NULL) {
        return ST_STATUS_INVALID_PARAMETER;
    }
    sti_usart_clock_control(instance, ENABLE);
    handle->pUSART = sti_get_usart_base_address(instance);
    if (handle->pUSART == NULL) {
        return ST_STATUS_INVALID_PARAMETER;
    }
    handle->config.instance = instance;
    handle->state = USART_STATE_RESET;
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
st_status_t st_usart_set_configuration(const st_usart_config_t *config, st_usart_handle_t *handle, const st_usart_io_t *usart_pin_config)
{
    if (config == NULL || handle == NULL)
    {
        return ST_STATUS_NULL_PARAMETER;
    }
    
    if (!validate_usart_handle(handle) || config->clock_mode >= USART_CLOCK_MODE_LAST || config->mode >= USART_MODE_LAST || config->parity >= USART_PARITY_LAST || config->oversampling >= USART_OVERSAMPLING_LAST ||
        config->word_length >= USART_WORD_LENGTH_LAST)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }

#if defined(USE_RTE_PIN_MAPPING) && (USE_RTE_PIN_MAPPING == 1)
    if (config->instance == USART_1)
    {
        usart_pin_config = &usart1_io;
    }
    else if (config->instance == USART_2)
    {
        usart_pin_config = &usart2_io;
    }
    else if (config->instance == USART_6)
    {
        usart_pin_config = &usart6_io;
    }
#elif !defined(USE_RTE_PIN_MAPPING) || (USE_RTE_PIN_MAPPING == 0)
    if (usart_pin_config == NULL)
    {
        return ST_STATUS_NULL_PARAMETER;
    }
#endif
    st_status_t status = st_usart_pin_init(usart_pin_config, config);
    if (status != ST_STATUS_OK)
    {
        return status;
    }

    if (config->mode == USART_MODE_SYNCHRONOUS_MASTER || config->mode == USART_MODE_SYNCHRONOUS_SLAVE)
    {
        if (config->is_flow_control_enable)
        {
            return ST_STATUS_INVALID_PARAMETER;
        }
        handle->pUSART->CR2_b.CLKEN = ENABLE;
        handle->pUSART->CR2_b.CPOL = (config->clock_mode >> 1) & 0x01;
        handle->pUSART->CR2_b.CPHA = config->clock_mode & 0x01;
    }

    if (config->parity != USART_PARITY_NONE)
    {
        handle->pUSART->CR1_b.PCE = ENABLE;
        handle->pUSART->CR1_b.PS = config->parity;
    }

    if (config->is_flow_control_enable)
    {
        handle->pUSART->CR3_b.RTSE = ENABLE;
        handle->pUSART->CR3_b.CTSE = ENABLE;
    }

    handle->pUSART->CR1_b.M = config->word_length;
    handle->pUSART->CR1_b.OVER8 = config->oversampling;
    handle->pUSART->CR2_b.STOP = config->stop_bits;
    handle->pUSART->CR1_b.UE = ENABLE;
    
    // Enable the USART Interrupts in non_blocking mode
    sti_usart_nvic_intr_enable(config->instance, ENABLE);

    // set the USART baudrate
    sti_usart_set_baudrate(handle->pUSART, config->baudrate, config->oversampling);

    // Updating the handle structure
    handle->state = USART_STATE_READY;
    handle->config.baudrate = config->baudrate;
    handle->config.clock_mode = config->clock_mode;
    handle->config.is_flow_control_enable = config->is_flow_control_enable;
    handle->config.mode = config->mode;
    handle->config.oversampling = config->oversampling;
    handle->config.parity = config->parity;
    handle->config.stop_bits = config->stop_bits;
    handle->config.word_length = config->word_length;
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
st_status_t st_usart_register_callback(st_usart_handle_t *handle, st_usart_callback_t callback)
{
    if (!validate_usart_handle(handle)) {
        return ST_STATUS_INVALID_PARAMETER;
    }
    handle->user_callback = callback;
    return ST_STATUS_OK;
}


/**
 * @brief Un-Register a callback function for USART events.
 *
 * Dissassociate a user-defined callback function with a USART instance.
 *
 * @param instance The USART instance for which to un-register the callback.
 *
 * @return st_status_t status code indicating success or error.
 */
st_status_t st_usart_unregister_callback(st_usart_handle_t *handle)
{
    if (!validate_usart_handle(handle)) {
        return ST_STATUS_INVALID_PARAMETER;
    }
    handle->user_callback = NULL;
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
st_status_t st_usart_send_data_blocking(st_usart_handle_t *handle, const uint8_t *tx_buf, uint16_t tx_len)
{
    if (!validate_usart_handle(handle)) {
        return ST_STATUS_INVALID_PARAMETER;
    }

    if (handle->state != USART_STATE_READY) {
        return ST_STATUS_BUSY;
    }
    handle->state = USART_STATE_BUSY_TX;
    handle->pUSART->CR1_b.TE = ENABLE;
    handle->pTxBuffer = tx_buf;
    handle->tx_size = tx_len;
    handle->tx_count = 0;
    while (handle->tx_count < handle->tx_size)
    {
        while (!handle->pUSART->SR_b.TXE);
        handle->pUSART->DR_b.DR = tx_buf[handle->tx_count++];
    }
    while(!handle->pUSART->SR_b.TC);
    handle->pUSART->CR1_b.TE = DISABLE;
    handle->state = USART_STATE_READY;
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
st_status_t st_usart_receive_data_blocking(st_usart_handle_t *handle, uint8_t *rx_buf, uint16_t rx_len)
{
    if (!validate_usart_handle(handle)) {
        return ST_STATUS_INVALID_PARAMETER;
    }
    if (handle->state != USART_STATE_READY) {
        return ST_STATUS_BUSY;
    }
    handle->state = USART_STATE_BUSY_RX;
    handle->pUSART->CR1_b.RE = ENABLE;
    handle->pRxBuffer = rx_buf;
    handle->rx_size = rx_len;
    handle->rx_count = 0;
    while (handle->rx_count < handle->rx_size) {
        while(!handle->pUSART->SR_b.RXNE);
        handle->pRxBuffer[handle->rx_count++] = (uint8_t)handle->pUSART->DR_b.DR;
    }
    handle->pUSART->CR1_b.RE = DISABLE;
    handle->state = USART_STATE_READY;
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
st_status_t st_usart_send_data_non_blocking(st_usart_handle_t *handle, const uint8_t *tx_buf, uint16_t tx_len)
{
    if (!validate_usart_handle(handle))
    {
        return ST_STATUS_INVALID_PARAMETER;
    }
    if (handle->state != USART_STATE_READY) {
        return ST_STATUS_BUSY;
    }
    handle->state = USART_STATE_BUSY_TX;
    handle->pUSART->CR1_b.TE = ENABLE;
    handle->pTxBuffer = tx_buf;
    handle->tx_size = tx_len;
    handle->tx_count = 0;
    handle->pUSART->CR1_b.TXEIE = ENABLE;
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
st_status_t st_usart_receive_data_non_blocking(st_usart_handle_t *handle, uint8_t *rx_buf, uint16_t rx_len)
{
    if (!validate_usart_handle(handle))
    {
        return ST_STATUS_INVALID_PARAMETER;
    }
    if (handle->state != USART_STATE_READY) {
        return ST_STATUS_BUSY;
    }
    handle->state = USART_STATE_BUSY_RX;
    handle->pUSART->CR1_b.RE = ENABLE;
    handle->pRxBuffer = rx_buf;
    handle->rx_size = rx_len;
    handle->rx_count = 0;
    handle->pUSART->CR1_b.RXNEIE = ENABLE;
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
st_status_t st_usart_deinit(st_usart_handle_t *handle)
{
    if (!validate_usart_handle(handle)) {
        return ST_STATUS_INVALID_PARAMETER;
    }
    sti_usart_nvic_intr_enable(handle->config.instance, DISABLE);
    st_usart_unregister_callback(handle);
    sti_usart_clock_control(handle->config.instance, DISABLE);
    handle->pUSART = NULL;
    handle->pTxBuffer = NULL;
    handle->pRxBuffer = NULL;
    handle->tx_size = 0;
    handle->rx_size = 0;
    handle->tx_count = 0;
    handle->rx_count = 0;
    handle->state = USART_STATE_RESET;
    memset(&handle->config, 0, sizeof(handle->config));
    return ST_STATUS_OK;
}

st_status_t st_usart_pin_init(const st_usart_io_t *pin_configs, const st_usart_config_t *usart_config)
{
    st_status_t status = ST_STATUS_OK;
    st_gpio_config_t usart_pin_config = {
        .mode = ALT_FN_MODE,
        .ospeed = GPIO_SPEED_HIGH,
        .otype = GPIO_PUSH_PULL,
    };
    if (pin_configs == NULL || usart_config == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }
    do
    {
        usart_pin_config.port = pin_configs->tx.port;
        usart_pin_config.pin = pin_configs->tx.pin;
        usart_pin_config.alt_fn = pin_configs->tx.alt_fn;
        usart_pin_config.pupd_config = GPIO_NOPUPD;
        status = st_gpio_clock_control(usart_pin_config.port, ENABLE);
        if (status != ST_STATUS_OK) {
            break;
        }
        status = st_gpio_set_configuration(&usart_pin_config);
        if (status != ST_STATUS_OK)
        {
            break;
        }
        // Configure USART RX pin
        usart_pin_config.port = pin_configs->rx.port;
        usart_pin_config.pin = pin_configs->rx.pin;
        usart_pin_config.alt_fn = pin_configs->rx.alt_fn;
        usart_pin_config.pupd_config = GPIO_PU;
        status = st_gpio_clock_control(usart_pin_config.port, ENABLE);
        if (status != ST_STATUS_OK) {
            break;
        }
        status = st_gpio_set_configuration(&usart_pin_config);
        if (status != ST_STATUS_OK)
        {
            break;
        }

        if (usart_config->mode == USART_MODE_SYNCHRONOUS_MASTER || usart_config->mode == USART_MODE_SYNCHRONOUS_SLAVE)
        {
            status = st_gpio_clock_control(usart_pin_config.port, ENABLE);
            if (status != ST_STATUS_OK) {
                break;
            }
            usart_pin_config.port = pin_configs->clk.port;
            usart_pin_config.pin = pin_configs->clk.pin;
            usart_pin_config.alt_fn = pin_configs->clk.alt_fn;
            usart_pin_config.pupd_config = GPIO_NOPUPD;
            status = st_gpio_set_configuration(&usart_pin_config);
            if (status != ST_STATUS_OK)
            {
                break;
            }
        }
        else
        {
            if (usart_config->is_flow_control_enable)
            {
                usart_pin_config.port = pin_configs->cts.port;
                usart_pin_config.pin = pin_configs->cts.pin;
                usart_pin_config.alt_fn = pin_configs->cts.alt_fn;
                usart_pin_config.pupd_config = GPIO_PU;
                status = st_gpio_clock_control(usart_pin_config.port, ENABLE);
                if (status != ST_STATUS_OK) {
                    break;
                }
                status = st_gpio_set_configuration(&usart_pin_config);
                if (status != ST_STATUS_OK)
                {
                    break;
                }

                usart_pin_config.port = pin_configs->rts.port;
                usart_pin_config.pin = pin_configs->rts.pin;
                usart_pin_config.alt_fn = pin_configs->rts.alt_fn;
                usart_pin_config.pupd_config = GPIO_NOPUPD;
                status = st_gpio_clock_control(usart_pin_config.port, ENABLE);
                if (status != ST_STATUS_OK) {
                    break;
                }
                status = st_gpio_set_configuration(&usart_pin_config);
                if (status != ST_STATUS_OK)
                {
                    break;
                }
            }
        }
    } while (false);
    return status;
}

void st_usart_hal_irq_handler(st_usart_handle_t *handle)
{
    if (handle->pUSART->SR_b.TXE) {
        if (handle->tx_count < handle->tx_size) {
            handle->pUSART->DR_b.DR = handle->pTxBuffer[handle->tx_count++];
        }
        else {
            if (handle->pUSART->SR_b.TC) {
                handle->pUSART->CR1_b.TE = DISABLE;
                handle->pUSART->CR1_b.TXEIE = DISABLE;
                handle->state = USART_STATE_READY;
                if (handle->user_callback != NULL) {
                    handle->user_callback((uint8_t)handle->config.instance, USART_EVENT_SEND_COMPLETE);
                }
            }
        }
    }
    if (handle->pUSART->SR_b.RXNE && handle->rx_count < handle->rx_size) {
        handle->pRxBuffer[handle->rx_count++] = (uint8_t)(handle->pUSART->DR_b.DR);
        if (handle->rx_count == handle->rx_size - 1) {
            handle->pUSART->CR1_b.RE = DISABLE;
            handle->pUSART->CR1_b.RXNEIE = DISABLE;
            handle->state = USART_STATE_READY;
            if (handle->user_callback != NULL) {
                handle->user_callback((uint8_t)handle->config.instance, USART_EVENT_RECEIVE_COMPLETE);
            }
        }
    }
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

static void sti_usart_set_baudrate(USART_TypeDef *pUSART, uint32_t baudrate, st_usart_oversampling_t oversampling)
{
    uint32_t usart_div;
    if (oversampling == USART_OVERSAMPLING_16)
    {
        usart_div = (USART_PERI_CLK + (baudrate / 2U)) / baudrate; // rounding
        pUSART->BRR = usart_div;
    }
    else if (oversampling == USART_OVERSAMPLING_8)
    {
        usart_div = ((2U * USART_PERI_CLK) + (baudrate / 2U)) / baudrate; // rounding
        pUSART->BRR = (usart_div & 0xFFF0U) | ((usart_div & 0x000FU) >> 1U);
    }
}

static bool validate_usart_handle(const st_usart_handle_t *handle)
{
    if (handle == NULL || handle->pUSART == NULL) {
        return false;
    }
    const USART_TypeDef *pUSART = sti_get_usart_base_address(handle->config.instance);
    if (handle->pUSART != pUSART) {
        return false;
    }
    return true;
}

static void sti_usart_nvic_intr_enable(st_usart_instance_t instance, uint8_t enable)
{
    IRQn_Type irq;
    switch (instance) {
        case USART_1:
            irq = USART1_IRQn;
            break;
        case USART_2:
            irq = USART2_IRQn;
            break;
        case USART_6:
            irq = USART6_IRQn;
            break;
        default:
            irq = USART1_IRQn;
    }
    if (enable) {
        NVIC_EnableIRQ(irq);
        NVIC_SetPriority(irq, 0);
    } else {
        NVIC_DisableIRQ(irq);
    }
}