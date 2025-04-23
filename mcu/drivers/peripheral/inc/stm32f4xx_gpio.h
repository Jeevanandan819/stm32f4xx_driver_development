/**
 * @file stm32f4xx_gpio.h
 * @brief Example header file contains the declarations of GPIO peripheral API prototyping.
 *
 * @author Jeevanandan Sandan
 * @date April 23, 2025
 *
 * This file contains the implementation of GPIO peripheral functionality and 
 * is maintained by Team Alpha. For queries or issues, contact the code owner.
 */

#include "stm32f4xx.h"
#include "st_status.h"

/**
 * @brief Enumeration for GPIO pin numbers.
 * 
 * This enum defines the pin numbers for a GPIO port. GPIO_LAST can be used 
 * to indicate the end or count of pins.
 */
typedef enum
{
    GPIO0,         /**< GPIO pin 0 */
    GPIO1,         /**< GPIO pin 1 */
    GPIO2,         /**< GPIO pin 2 */
    GPIO3,         /**< GPIO pin 3 */
    GPIO4,         /**< GPIO pin 4 */
    GPIO5,         /**< GPIO pin 5 */
    GPIO6,         /**< GPIO pin 6 */
    GPIO7,         /**< GPIO pin 7 */
    GPIO8,         /**< GPIO pin 8 */
    GPIO9,         /**< GPIO pin 9 */
    GPIO10,        /**< GPIO pin 10 */
    GPIO11,        /**< GPIO pin 11 */
    GPIO12,        /**< GPIO pin 12 */
    GPIO13,        /**< GPIO pin 13 */
    GPIO14,        /**< GPIO pin 14 */
    GPIO15,        /**< GPIO pin 15 */
    GPIO_PIN_LAST      /**< Indicator for the end of GPIO pins */
} st_gpio_pin_t;

/**
 * @brief Enumeration for GPIO modes.
 * 
 * This enum defines the possible modes for GPIO pins such as input, output, 
 * alternate function, and analog mode.
 */
typedef enum
{
    INPUT,         /**< GPIO pin in input mode */
    OUTPUT,        /**< GPIO pin in output mode */
    ALT_FN_MODE,   /**< GPIO pin in alternate function mode */
    ANALOG_MODE,   /**< GPIO pin in analog mode */
    GPIO_MODE_LAST /**< Indicator for the end of GPIO modes */
} st_gpio_mode_t;

/**
 * @brief Enumeration for GPIO output types.
 * 
 * This enum defines the types of GPIO outputs: push-pull and open-drain.
 */
typedef enum
{
    GPIO_PUSH_PULL, /**< GPIO pin in push-pull mode */
    GPIO_OPEN_DRAIN,/**< GPIO pin in open-drain mode */
    GPIO_OTYPE_LAST /**< Indicator for the end of GPIO output types */
} st_gpio_otype_t;

/**
 * @brief Enumeration for GPIO speed.
 * 
 * This enum defines the possible output speeds for GPIO pins: low, medium, fast, 
 * and high speed.
 */
typedef enum
{
    GPIO_SPEED_LOW,      /**< GPIO pin in low speed mode */
    GPIO_SPEED_MEDIUM,   /**< GPIO pin in medium speed mode */
    GPIO_SPEED_FAST,     /**< GPIO pin in fast speed mode */
    GPIO_SPEED_HIGH,     /**< GPIO pin in high speed mode */
    GPIO_SPEED_LAST /**< Indicator for the end of GPIO speeds */
} st_gpio_speed_t;

/**
 * @brief Enumeration for GPIO pull-up/pull-down configuration.
 * 
 * This enum defines the possible configurations for pull-up/pull-down resistors 
 * for GPIO pins.
 */
typedef enum
{
    GPIO_NOPUPD,    /**< No pull-up or pull-down resistor */
    GPIO_PU,        /**< Pull-up resistor enabled */
    GPIO_PD,        /**< Pull-down resistor enabled */
    GPIO_PUPD_LAST  /**< Indicator for the end of GPIO pull-up/pull-down configurations */
} st_gpio_pupd_config_t;

/**
 * @brief Enumeration for GPIO alternate functions.
 * 
 * This enum defines the possible alternate functions for GPIO pins.
 */
typedef enum
{
    GPIO_ALT_FN0,   /**< Alternate function 0 */
    GPIO_ALT_FN1,   /**< Alternate function 1 */
    GPIO_ALT_FN2,   /**< Alternate function 2 */
    GPIO_ALT_FN3,   /**< Alternate function 3 */
    GPIO_ALT_FN4,   /**< Alternate function 4 */
    GPIO_ALT_FN5,   /**< Alternate function 5 */
    GPIO_ALT_FN6,   /**< Alternate function 6 */
    GPIO_ALT_FN7,   /**< Alternate function 7 */
    GPIO_ALT_FN8,   /**< Alternate function 8 */
    GPIO_ALT_FN9,   /**< Alternate function 9 */
    GPIO_ALT_FN10,  /**< Alternate function 10 */
    GPIO_ALT_FN11,  /**< Alternate function 11 */
    GPIO_ALT_FN12,  /**< Alternate function 12 */
    GPIO_ALT_FN13,  /**< Alternate function 13 */
    GPIO_ALT_FN14,  /**< Alternate function 14 */
    GPIO_ALT_FN15,  /**< Alternate function 15 */
    GPIO_ALT_FN_LAST/**< Indicator for the end of GPIO alternate functions */
} st_gpio_alt_fn_t;

/**
 * @brief Enum representing GPIO port identifiers.
 *
 * This enumeration lists the available GPIO ports on the microcontroller.
 * Each value corresponds to a specific GPIO port.
 */
typedef enum {
    GPIO_A,          /**< GPIO Port A */
    GPIO_B,          /**< GPIO Port B */
    GPIO_C,          /**< GPIO Port C */
    GPIO_D,          /**< GPIO Port D */
    GPIO_E,          /**< GPIO Port E */
    GPIO_F,          /**< GPIO Port F */
    GPIO_G,          /**< GPIO Port G */
    GPIO_H,          /**< GPIO Port H */
    GPIO_I,          /**< GPIO Port I */
    GPIO_J,          /**< GPIO Port J */
    GPIO_K,          /**< GPIO Port K */
    GPIO_PORT_LAST   /**< Last entry marker for GPIO ports */
} st_gpio_port_t;

/**
 * @brief Enum representing GPIO interrupt trigger types.
 *
 * This enumeration specifies the possible trigger conditions for GPIO interrupts.
 */
typedef enum {
    GPIO_INTR_RISE_EDGE,       /**< Interrupt triggered on rising edge */
    GPIO_INTR_FALL_EDGE,       /**< Interrupt triggered on falling edge */
    GPIO_INTR_RISE_FALL_EDGE   /**< Interrupt triggered on both rising and falling edges */
} st_gpio_intr_flag_t;

typedef struct {
    st_gpio_port_t port;
    st_gpio_pin_t pin;
}st_gpio_t;

/**
 * @brief  Structure to hold the GPIO configuration parameters.
 * 
 * This structure is used to define the configuration for a GPIO pin. It includes
 * the port, pin number, mode, output type, pull-up/pull-down settings, and alternate
 * function if required.
 */
typedef struct {
    st_gpio_port_t port;              /**< GPIO port (e.g., GPIO_A, GPIO_B). */
    st_gpio_pin_t pin;                    /**< GPIO pin number. */
    st_gpio_mode_t mode;              /**< GPIO pin mode (input, output, alternate, etc.). */
    st_gpio_otype_t otype;            /**< GPIO output type (push-pull or open-drain). */
    st_gpio_speed_t ospeed;           /**< GPIO output speed (low, medium, fast etc.). */
    st_gpio_pupd_config_t pupd_config;/**< GPIO pull-up/pull-down configuration. */
    st_gpio_alt_fn_t alt_fn;          /**< GPIO alternate function (if applicable). */
} st_gpio_config_t;

/**
 * @brief  Configure the mode of a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_pin_t).
 * @param  mode: Mode to configure (of type st_gpio_mode_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_config_mode(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, st_gpio_mode_t mode);

/**
 * @brief  Set the output type of a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_pin_t).
 * @param  otype: Output type to set (of type st_gpio_otype_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_set_otype(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, st_gpio_otype_t otype);

/**
 * @brief  Configure the speed of a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_pin_t).
 * @param  ospeed: Speed to configure (of type st_gpio_speed_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_config_speed(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, st_gpio_speed_t ospeed);

/**
 * @brief  Configure the pull-up or pull-down resistor for a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_pin_t).
 * @param  pupd_config: Pull-up/pull-down configuration (of type st_gpio_pupd_config_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_config_pupd(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, st_gpio_pupd_config_t pupd_config);

/**
 * @brief  Configure the alternate function (pin multiplexing) for a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_pin_t).
 * @param  alt_fn: Alternate function to set (of type st_gpio_alt_fn_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_set_pin_mux(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, st_gpio_alt_fn_t alt_fn);

/**
 * @brief Sets the value of a specific GPIO pin.
 *
 * This function allows you to set the value (high or low) of a specific pin
 * in a GPIO port.
 *
 * @param[in] pGPIO Pointer to the GPIO port structure.
 * @param[in] pin The pin number within the GPIO port.
 * @param[in] value The value to set (0 for low, 1 for high).
 *
 * @return Status of the operation (success or error).
 */
st_status_t st_gpio_set_pin(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, uint8_t value);

/**
 * @brief Toggles the state of a specific GPIO pin.
 *
 * This function toggles the current state (high/low) of a specific pin
 * in a GPIO port.
 *
 * @param[in] pGPIO Pointer to the GPIO port structure.
 * @param[in] pin The pin number within the GPIO port.
 *
 * @return Status of the operation (success or error).
 */
st_status_t st_gpio_toggle_pin(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin);

/**
 * @brief Retrieves the current state of a specific GPIO pin.
 *
 * This function reads and returns the current state (high or low) of 
 * a specific GPIO pin in the given GPIO port.
 *
 * @param[in] pGPIO Pointer to the GPIO port structure.
 * @param[in] pin The pin number within the GPIO port to be read.
 *
 * @return The current state of the pin (0 for low, 1 for high).
 */
uint8_t st_gpio_get_pin(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin);

/**
 * @brief Sets or resets a specific GPIO pin in the given GPIO port.
 *
 * This function allows you to either set (make high) or reset (make low) 
 * the state of a specific pin in a GPIO port.
 *
 * @param[in] pGPIO Pointer to the GPIO port structure.
 * @param[in] pin The pin number within the GPIO port to be modified.
 * @param[in] set The action to perform (0 to reset the pin, 1 to set the pin).
 *
 * @return Status of the operation (success or error).
 */
st_status_t st_gpio_port_set_reset(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, uint8_t set);

/**
 * @brief  Set the configuration for a GPIO pin.
 * 
 * This API configures various settings for a specific GPIO pin, including mode,
 * output type, pull-up/pull-down configuration, and alternate function, using
 * a configuration structure.
 * 
 * @param  gpio_config: Pointer to a structure containing the GPIO configuration.
 * @retval st_status_t: Status of the operation. Possible values include:
 *          - ST_SUCCESS: Configuration completed successfully.
 *          - ST_ERROR: An error occurred during configuration (e.g., invalid parameters).
 */
st_status_t st_gpio_set_configuration(st_gpio_config_t *gpio_config);

// st_status_t st_gpio_config_interrupt()

#define GPIOA_PERI_CLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PERI_CLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PERI_CLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PERI_CLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PERI_CLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PERI_CLK_EN() (RCC->AHB1ENR |= (1 << 7))

#define GPIOA_PERI_CLK_DIS() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PERI_CLK_DIS() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PERI_CLK_DIS() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PERI_CLK_DIS() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PERI_CLK_DIS() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PERI_CLK_DIS() (RCC->AHB1ENR &= ~(1 << 7))