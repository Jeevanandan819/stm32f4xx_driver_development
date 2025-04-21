#include "stm32f4xx_gpio.h"

static GPIO_TypeDef* get_gpio_base_address(st_gpio_port_t port);

/**
 * @brief  Configure the mode of a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_t).
 * @param  mode: Mode to configure (of type st_gpio_mode_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_config_mode(GPIO_TypeDef *pGPIO, st_gpio_t pin, st_gpio_mode_t mode)
{
    if (pin < GPIO0 || pin >= GPIO_PIN_LAST) {
        return ST_STATUS_INVALID_PARAMETER;
    }

    if (mode < INPUT || mode >= GPIO_MODE_LAST) {
        return ST_STATUS_INVALID_PARAMETER;
    }
    pGPIO->MODER &= ~(3 << (2 * pin));
    pGPIO->MODER |= (mode << (2 * pin));
    return ST_STATUS_OK;
}

/**
 * @brief  Set the output type of a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_t).
 * @param  otype: Output type to set (of type st_gpio_otype_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_set_otype(GPIO_TypeDef *pGPIO, st_gpio_t pin, st_gpio_otype_t otype)
{
    if (pin < GPIO0 || pin >= GPIO_PIN_LAST) {
        return ST_STATUS_INVALID_PARAMETER;
    }

    if (otype < GPIO_PUSH_PULL || otype >= GPIO_OTYPE_LAST) {
        return ST_STATUS_INVALID_PARAMETER;
    }

    pGPIO->OTYPER &= ~(1 << pin);
    pGPIO->OTYPER |= (1 << pin);
    return ST_STATUS_OK;
}

/**
 * @brief  Configure the speed of a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_t).
 * @param  ospeed: Speed to configure (of type st_gpio_speed_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_config_speed(GPIO_TypeDef *pGPIO, st_gpio_t pin, st_gpio_speed_t ospeed)
{
    if (pin < GPIO0 || pin >= GPIO_PIN_LAST) {
        return ST_STATUS_INVALID_PARAMETER;
    }

    if (ospeed < GPIO_SPEED_LOW || ospeed >= GPIO_SPEED_LAST) {
        return ST_STATUS_INVALID_PARAMETER;
    }
    pGPIO->OSPEEDR &= ~(3 << (2 * pin));
    pGPIO->OSPEEDR |= (ospeed << (2 * pin));
    return ST_STATUS_OK;
}

/**
 * @brief  Configure the pull-up or pull-down resistor for a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_t).
 * @param  pupd_config: Pull-up/pull-down configuration (of type st_gpio_pupd_config_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_config_pupd(GPIO_TypeDef *pGPIO, st_gpio_t pin, st_gpio_pupd_config_t pupd_config)
{
    if (pin < GPIO0 || pin >= GPIO_PIN_LAST) {
        return ST_STATUS_INVALID_PARAMETER;
    }

    if (pupd_config < GPIO_PULLUP || pupd_config >= GPIO_PUPD_LAST) {
        return ST_STATUS_INVALID_PARAMETER;
    }
    pGPIO->PUPDR &= ~(3 << (2 * pin));
    pGPIO->PUPDR |= (pupd_config << (2 * pin));
    return ST_STATUS_OK;
}

/**
 * @brief  Configure the alternate function (pin multiplexing) for a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_t).
 * @param  alt_fn: Alternate function to set (of type st_gpio_alt_fn_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_set_pin_mux(GPIO_TypeDef *pGPIO, st_gpio_t pin, st_gpio_alt_fn_t alt_fn)
{

}

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
st_status_t st_gpio_set_configuration(st_gpio_config_t *gpio_config)
{

}
