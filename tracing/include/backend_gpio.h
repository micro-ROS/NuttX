#ifndef __BACKEND_GPIO_H__
#define __BACKEND_GPIO_H__

/**
 * @brief Generate the trigger 
 */
void gpio_backend_trigger(void);

/**
 * @brief configure the GPIO to trigger the measurement
 */
void gpio_backend_configure_trigger(void);

#endif //__BACKEND_GPIOS_H__
