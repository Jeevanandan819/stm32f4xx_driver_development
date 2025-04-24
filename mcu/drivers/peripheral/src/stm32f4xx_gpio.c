/**
 * @file stm32f4xx_gpio.c
 * @brief Example header file contains the definitions of GPIO peripheral of NUCLEO-F411RE
 *
 * @author Jeevanandan Sandan
 * @date April 23, 2025
 *
 * This file contains the implementation of GPIO peripheral functionality and
 * is maintained by Team Alpha. For queries or issues, contact the code owner.
 */

#include "stm32f4xx_gpio.h"
#include <stddef.h>

static GPIO_TypeDef *get_gpio_base_address(st_gpio_port_t port);
static void st_gpio_clear_interrupt(uint8_t channel);
static st_gpio_intr_callback gpio_intr_callbacks[16];

/**
 * @brief  Configure the mode of a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_pin_t).
 * @param  mode: Mode to configure (of type st_gpio_mode_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_config_mode(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, st_gpio_mode_t mode)
{
    if (pin >= GPIO_PIN_LAST || mode >= GPIO_MODE_LAST || pGPIO == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }

    pGPIO->MODER &= ~(3 << (2 * pin));
    pGPIO->MODER |= (mode << (2 * pin));
    return ST_STATUS_OK;
}

/**
 * @brief  Set the output type of a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_pin_t).
 * @param  otype: Output type to set (of type st_gpio_otype_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_set_otype(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, st_gpio_otype_t otype)
{
    if (pin >= GPIO_PIN_LAST || otype >= GPIO_OTYPE_LAST || pGPIO == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }

    pGPIO->OTYPER &= ~(1 << pin);
    pGPIO->OTYPER |= (1 << pin);
    return ST_STATUS_OK;
}

/**
 * @brief  Configure the speed of a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_pin_t).
 * @param  ospeed: Speed to configure (of type st_gpio_speed_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_config_speed(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, st_gpio_speed_t ospeed)
{
    if (pin >= GPIO_PIN_LAST || ospeed >= GPIO_SPEED_LAST || pGPIO == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }

    pGPIO->OSPEEDR &= ~(3 << (2 * pin));
    pGPIO->OSPEEDR |= (ospeed << (2 * pin));
    return ST_STATUS_OK;
}

/**
 * @brief  Configure the pull-up or pull-down resistor for a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_pin_t).
 * @param  pupd_config: Pull-up/pull-down configuration (of type st_gpio_pupd_config_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_config_pupd(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, st_gpio_pupd_config_t pupd_config)
{
    if (pin >= GPIO_PIN_LAST || pupd_config >= GPIO_PUPD_LAST || pGPIO == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }

    pGPIO->PUPDR &= ~(3 << (2 * pin));
    pGPIO->PUPDR |= (pupd_config << (2 * pin));
    return ST_STATUS_OK;
}

/**
 * @brief  Configure the alternate function (pin multiplexing) for a GPIO pin.
 * @param  pGPIO: Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param  pin: GPIO pin to configure (of type st_gpio_pin_t).
 * @param  alt_fn: Alternate function to set (of type st_gpio_alt_fn_t).
 * @retval st_status_t: Status of the operation (e.g., success or failure).
 */
st_status_t st_gpio_set_pin_mux(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, st_gpio_alt_fn_t alt_fn)
{
    if (pin >= GPIO_PIN_LAST || alt_fn >= GPIO_ALT_FN_LAST || pGPIO == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }
    pGPIO->AFR[pin / 8] &= ~(0x0F << (pin * 4));
    pGPIO->AFR[pin / 8] |= (alt_fn << (pin * 4));
    return ST_STATUS_OK;
}

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
st_status_t st_gpio_set_pin(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, uint8_t value)
{
    if (pin >= GPIO_PIN_LAST || pGPIO == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }

    if (value)
    {
        pGPIO->ODR |= (1 << pin);
    }
    else
    {
        pGPIO->ODR &= ~(1 << pin);
    }
    return ST_STATUS_OK;
}

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
st_status_t st_gpio_toggle_pin(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin)
{
    if (pin >= GPIO_PIN_LAST || pGPIO == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }
    pGPIO->ODR ^= (1 << pin);
    return ST_STATUS_OK;
}

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
uint8_t st_gpio_get_pin(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin)
{
    if (pin >= GPIO_PIN_LAST || pGPIO == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }
    return (pGPIO->IDR >> pin) & 1;
}

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
st_status_t st_gpio_port_set_reset(GPIO_TypeDef *pGPIO, st_gpio_pin_t pin, uint8_t set)
{
    if (pin >= GPIO_PIN_LAST || pGPIO == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }
    if (set)
    {
        pGPIO->BSRR |= (1 << pin);
    }
    else
    {
        pGPIO->BSRR |= (1 << (pin + 16));
    }
    return ST_STATUS_OK;
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
    // if (gpio_config->pin >= GPIO_PIN_LAST || gpio_config->mode >= GPIO_MODE_LAST ||
    //     gpio_config->otype >= GPIO_OTYPE_LAST || gpio_config->port >= GPIO_PORT_LAST ||
    //     gpio_config->pupd_config >= GPIO_PUPD_LAST || gpio_config->alt_fn >= GPIO_ALT_FN_LAST ||
    //     gpio_config->ospeed >= GPIO_SPEED_LAST)
    // {
    //     return ST_STATUS_INVALID_PARAMETER;
    // }
    GPIO_TypeDef *pGPIO = get_gpio_base_address(gpio_config->port);
    if (pGPIO == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }
    st_status_t status;
    status = st_gpio_config_mode(pGPIO, gpio_config->pin, gpio_config->mode);
    if (status != ST_STATUS_OK)
    {
        return status;
    }

    if (gpio_config->mode == OUTPUT || gpio_config->mode == ALT_FN_MODE)
    {
        status = st_gpio_set_otype(pGPIO, gpio_config->pin, gpio_config->otype);
        if (status != ST_STATUS_OK)
        {
            return status;
        }
        status = st_gpio_config_speed(pGPIO, gpio_config->pin, gpio_config->ospeed);
        if (status != ST_STATUS_OK)
        {
            return status;
        }
    }

    status = st_gpio_config_pupd(pGPIO, gpio_config->pin, gpio_config->pupd_config);
    if (status != ST_STATUS_OK)
    {
        return status;
    }

    if (gpio_config->mode == ALT_FN_MODE)
    {
        status = st_gpio_set_pin_mux(pGPIO, gpio_config->pin, gpio_config->alt_fn);
        if (status != ST_STATUS_OK)
        {
            return status;
        }
    }
    return ST_STATUS_OK;
}

/**
 * @brief Configures an interrupt for a specific GPIO pin.
 *
 * This function sets up the interrupt trigger conditions for the specified
 * GPIO pin, allowing it to respond to external events such as rising edges,
 * falling edges, or both.
 *
 * @param[in] gpio_port_pin Pointer to the GPIO port and pin structure.
 * @param[in] intr_flag Interrupt trigger type, specified using
 *                       st_gpio_intr_flag_t (rising edge, falling edge, or both).
 *
 * @return Status of the configuration operation (success or error).
 */
st_status_t st_gpio_config_interrupt(st_gpio_t *gpio_port_pin, st_gpio_intr_flag_t intr_flag, st_gpio_intr_callback callback_function)
{
    GPIO_TypeDef *pGPIO = get_gpio_base_address(gpio_port_pin->port);
    if (gpio_port_pin->pin >= GPIO_PIN_LAST || pGPIO == NULL || callback_function == NULL)
    {
        return ST_STATUS_INVALID_PARAMETER;
    }
    EXTI->IMR |= (1 << gpio_port_pin->pin);
    if (intr_flag == GPIO_INTR_FALL_EDGE)
    {
        EXTI->FTSR |= (1 << gpio_port_pin->pin);
    }
    else if (intr_flag == GPIO_INTR_RISE_EDGE)
    {
        EXTI->RTSR |= (1 << gpio_port_pin->pin);
    }
    else
    {
        EXTI->FTSR |= (1 << gpio_port_pin->pin);
        EXTI->RTSR |= (1 << gpio_port_pin->pin);
    }
    EXTI->PR |= (1 << gpio_port_pin->pin);
    SYSCFG->EXTICR[gpio_port_pin->pin / 4] &= ~(0xF << (gpio_port_pin->pin % 4) * 4);
    SYSCFG->EXTICR[gpio_port_pin->pin / 4] |= (gpio_port_pin->port << (gpio_port_pin->pin % 4) * 4);
    gpio_intr_callbacks[gpio_port_pin->pin] = callback_function;
    return ST_STATUS_OK;
}

static GPIO_TypeDef *get_gpio_base_address(st_gpio_port_t port)
{
    GPIO_TypeDef *pGPIO = NULL;
    switch (port)
    {
    case GPIO_A:
        pGPIO = GPIOA;
        break;
    case GPIO_B:
        pGPIO = GPIOB;
        break;
    case GPIO_C:
        pGPIO = GPIOC;
        break;
    case GPIO_D:
        pGPIO = GPIOD;
        break;
    case GPIO_E:
        pGPIO = GPIOE;
        break;
    case GPIO_H:
        pGPIO = GPIOH;
        break;
    default:
        pGPIO = NULL;
        break;
    }
    return pGPIO;
}

static void st_gpio_clear_interrupt(uint8_t channel)
{
    EXTI->PR |= (1 << channel);
}

void EXTI0_IRQHandler(void)
{
    st_gpio_clear_interrupt(GPIO0);
    gpio_intr_callbacks[GPIO0](GPIO0);
}

void EXTI1_IRQHandler(void)
{
    st_gpio_clear_interrupt(GPIO1);
    gpio_intr_callbacks[GPIO1](GPIO1);
}

void EXTI2_IRQHandler(void)
{
    st_gpio_clear_interrupt(GPIO2);
    gpio_intr_callbacks[GPIO2](GPIO2);
}

void EXTI3_IRQHandler(void)
{
    st_gpio_clear_interrupt(GPIO3);
    gpio_intr_callbacks[GPIO3](GPIO3);
}

void EXTI4_IRQHandler(void)
{
    st_gpio_clear_interrupt(GPIO4);
    gpio_intr_callbacks[GPIO4](GPIO4);
}

void EXTI9_5_IRQHandler(void)
{
    int channel = __builtin_ctz(EXTI->PR);
    st_gpio_clear_interrupt(channel);
    gpio_intr_callbacks[channel]((uint8_t)channel);
}

void EXTI15_10_IRQHandler(void)
{
    int channel = __builtin_ctz(EXTI->PR);
    st_gpio_clear_interrupt(channel);
    gpio_intr_callbacks[channel]((uint8_t)channel);
}