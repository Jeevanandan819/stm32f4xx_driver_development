/**
 * @file st_usart.h
 * @brief USART module API declarations.
 *
 * This file contains the base types and function prototypes for configuring
 * and using a USART peripheral.
 */

#include <stdbool.h>
#include "stm32f4xx.h"
#include "st_status.h"
#include "RTE_Device_STM32F4xx.h"

/**
 * @brief Callback function type for USART events.
 *
 * This typedef defines a function pointer for a callback function that will be
 * invoked by the USART driver when a defined event occurs.
 *
 * @param event A 32-bit value specifying the USART event (e.g., transmission complete,
 *              error conditions, etc.).
 */
typedef void (*st_usart_callback_t)(uint32_t event);

/**
 * @brief USART instance enumeration.
 *
 * This enumeration lists the supported USART instances within the MCU.
 */
typedef enum
{
    USART_1, /**< USART instance 1 */
    USART_2, /**< USART instance 2 */
    USART_6  /**< USART instance 6 */
} st_usart_instance_t;

/**
 * @brief USART operating mode enumeration.
 *
 * This enumeration defines the various operational modes for the USART peripheral.
 */
typedef enum
{
    USART_MODE_ASYNCHRONOUS,       /**< Asynchronous mode */
    USART_MODE_SYNCHRONOUS_MASTER, /**< Synchronous mode as Master */
    USART_MODE_SYNCHRONOUS_SLAVE,  /**< Synchronous mode as Slave */
    USART_MODE_SINGLE_WIRE,        /**< Single-wire mode */
    USART_MODE_IRDA,               /**< IRDA mode */
    USART_MODE_LIN,                /**< LIN mode */
    USART_MODE_SMART_CARD,         /**< Smart Card mode */
    USART_MODE_LAST                /**< Marker for last supported mode */
} st_usart_mode_t;

/**
 * @brief USART parity configuration enumeration.
 *
 * This enumeration defines the parity settings available for USART communication.
 */
typedef enum
{
    USART_PARITY_EVEN, /**< Even parity */
    USART_PARITY_ODD,  /**< Odd parity */
    USART_PARITY_NONE, /**< No parity bit */
    USART_PARITY_LAST  /**< Marker for last parity option */
} st_usart_parity_t;

/**
 * @brief USART stop bits configuration enumeration.
 *
 * This enumeration defines the number of stop bits that can be configured.
 */
typedef enum
{
    USART_STOP_BIT_1,   /**< 1 stop bit */
    USART_STOP_BIT_0_5, /**< 0.5 stop bit */
    USART_STOP_BIT_2,   /**< 2 stop bits */
    USART_STOP_BIT_1_5, /**< 1.5 stop bits */
    USART_STOP_BIT_LAST
} st_usart_stop_bit_t;

/**
 * @brief USART clock mode configuration enumeration.
 *
 * This enumeration defines the clock polarity and phase settings for synchronous modes.
 */
typedef enum
{
    USART_CPOL0_CPHA0, /**< Clock polarity 0 and clock phase 0 */
    USART_CPOL0_CPHA1, /**< Clock polarity 0 and clock phase 1 */
    USART_CPOL1_CPHA0, /**< Clock polarity 1 and clock phase 0 */
    USART_CPOL1_CPHA1, /**< Clock polarity 1 and clock phase 1 */
    USART_CLOCK_MODE_LAST
} st_usart_clock_mode_t;

typedef enum
{
    USART_OVERSAMPLING_16,
    USART_OVERSAMPLING_8,
    USART_OVERSAMPLING_LAST
} st_usart_oversampling_t;

typedef enum
{
    USART_WORD_LENGTH_8,
    USART_WORD_LENGTH_9,
    USART_WORD_LENGTH_LAST
} st_usart_word_length_t;

/**
 * @brief USART configuration structure.
 *
 * This structure holds all configuration parameters used to initialize
 * and configure a USART peripheral.
 */
typedef struct
{
    st_usart_instance_t instance;         /**< USART instance to use */
    st_usart_mode_t mode;                 /**< Operating mode */
    st_usart_parity_t parity;             /**< Parity configuration */
    st_usart_stop_bit_t stop_bits;        /**< Stop bits configuration */
    st_usart_clock_mode_t clock_mode;     /**< Clock mode for synchronous operation */
    uint32_t baudrate;                    /**< Baudrate for communication */
    st_usart_oversampling_t oversampling; /**< Oversampling for noise tolerance */
    st_usart_word_length_t word_length;   /**< Word length for USART frame format */
    bool is_flow_control_enable;          /**< Enable or disable hardware flow control */
} st_usart_config_t;

typedef struct
{
    st_peripheral_io_t tx;
    st_peripheral_io_t rx;
    st_peripheral_io_t cts;
    st_peripheral_io_t rts;
    st_peripheral_io_t clk;
} st_usart_io_t;

typedef enum
{
    USART_STATE_RESET = 0x00U,      // Peripheral not initialized
    USART_STATE_READY = 0x01U,      // Peripheral initialized and ready for use
    USART_STATE_BUSY = 0x02U,       // Process ongoing
    USART_STATE_BUSY_TX = 0x12U,    // Data Transmission ongoing
    USART_STATE_BUSY_RX = 0x22U,    // Data Reception ongoing
    USART_STATE_BUSY_TX_RX = 0x32U, // Data Transmission and Reception ongoing
    USART_STATE_TIMEOUT = 0x03U,    // Timeout state
    USART_STATE_ERROR = 0x04U       // Error occurred
} st_usart_state_t;

/**
 * @brief Initialize the specified USART instance.
 *
 * This function performs basic initialization for the given USART instance.
 *
 * @param instance The USART instance to initialize.
 *
 * @return st_status_t status code indicating success or error.
 */
st_status_t st_usart_init(st_usart_instance_t instance);

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
st_status_t st_usart_set_configuration(st_usart_config_t *config, st_usart_io_t *usart_pin_config);

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
st_status_t st_usart_register_callback(st_usart_instance_t instance, st_usart_callback_t callback);

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
st_status_t st_usart_send_data_blocking(st_usart_instance_t instance, uint8_t *tx_buf, uint16_t tx_len);

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
st_status_t st_usart_receive_data_blocking(st_usart_instance_t instance, uint8_t *rx_buf, uint16_t rx_len);

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
st_status_t st_usart_send_data_non_blocking(st_usart_instance_t instance, uint8_t *tx_buf, uint16_t tx_len);

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
st_status_t st_usart_receive_data_non_blocking(st_usart_instance_t instance, uint8_t *rx_buf, uint16_t rx_len);

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
st_status_t st_usart_deinit(st_usart_instance_t instance);

st_status_t st_usart_pin_init(st_usart_io_t *pin_configs, st_usart_config_t *usart_config);

#define RCC_USART2_PERI_CLK_EN() (RCC->APB1ENR |= 1 << RCC_APB1ENR_USART2EN_Pos)
#define RCC_USART2_PERI_CLK_DIS() (RCC->APB1ENR &= ~(1 << RCC_APB1ENR_USART2EN_Pos))
#define RCC_USART1_PERI_CLK_EN() (RCC->APB2ENR |= (1 << RCC_APB2ENR_USART1EN_Pos))
#define RCC_USART1_PERI_CLK_DIS() (RCC->APB2ENR &= ~(1 << RCC_APB2ENR_USART1EN_Pos))
#define RCC_USART6_PERI_CLK_EN() (RCC->APB2ENR |= (1 << RCC_APB2ENR_USART6EN_Pos))
#define RCC_USART6_PERI_CLK_DIS() (RCC->APB2ENR &= ~(1 << RCC_APB2ENR_USART6EN_Pos))
