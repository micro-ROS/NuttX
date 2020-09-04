#include <nuttx/config.h>
#include <backend_gpio.h>

#ifdef CONFIG_ARCH_CHIP_STM32

#include <stm32_gpio.h>

#ifdef CONFIG_GPIO_TRIGGER_PORT_A
#define TRIGGER_PORT_ID GPIO_PORTA
#elif defined(CONFIG_GPIO_TRIGGER_PORT_B)
#define TRIGGER_PORT_ID GPIO_PORTB
#elif defined(CONFIG_GPIO_TRIGGER_PORT_C)
#define TRIGGER_PORT_ID GPIO_PORTC
#elif defined(CONFIG_GPIO_TRIGGER_PORT_D)
#define TRIGGER_PORT_ID GPIO_PORTD
#elif defined(CONFIG_GPIO_TRIGGER_PORT_E)
#define TRIGGER_PORT_ID GPIO_PORTE
#elif defined(CONFIG_GPIO_TRIGGER_PORT_F)
#define TRIGGER_PORT_ID GPIO_PORTF
#elif defined(CONFIG_GPIO_TRIGGER_PORT_G)
#define TRIGGER_PORT_ID GPIO_PORTG
#elif defined(CONFIG_GPIO_TRIGGER_PORT_H)
#define TRIGGER_PORT_ID GPIO_PORTH
#elif defined(CONFIG_GPIO_TRIGGER_PORT_I)
#define TRIGGER_PORT_ID GPIO_PORTI
#elif defined(CONFIG_GPIO_TRIGGER_PORT_J)
#define TRIGGER_PORT_ID GPIO_PORTJ
#else
	#error "Provide a GPIO PORT for the trigger"
#endif

#ifdef CONFIG_GPIO_TRIGGER_PIN_0
#elif defined(CONFIG_GPIO_TRIGGER_PIN_1)
#define TRIGGER_PIN_ID GPIO_PIN1
#elif defined(CONFIG_GPIO_TRIGGER_PIN_2)
#define TRIGGER_PIN_ID GPIO_PIN2
#elif defined(CONFIG_GPIO_TRIGGER_PIN_3)
#define TRIGGER_PIN_ID GPIO_PIN3
#elif defined(CONFIG_GPIO_TRIGGER_PIN_4)
#define TRIGGER_PIN_ID GPIO_PIN4
#elif defined(CONFIG_GPIO_TRIGGER_PIN_5)
#define TRIGGER_PIN_ID GPIO_PIN5
#elif defined(CONFIG_GPIO_TRIGGER_PIN_6)
#define TRIGGER_PIN_ID GPIO_PIN6
#elif defined(CONFIG_GPIO_TRIGGER_PIN_7)
#define TRIGGER_PIN_ID GPIO_PIN7
#elif defined(CONFIG_GPIO_TRIGGER_PIN_8)
#define TRIGGER_PIN_ID GPIO_PIN8
#elif defined(CONFIG_GPIO_TRIGGER_PIN_9)
#define TRIGGER_PIN_ID GPIO_PIN9
#elif defined(CONFIG_GPIO_TRIGGER_PIN_10)
#define TRIGGER_PIN_ID GPIO_PIN10
#elif defined(CONFIG_GPIO_TRIGGER_PIN_11)
#define TRIGGER_PIN_ID GPIO_PIN11
#elif defined(CONFIG_GPIO_TRIGGER_PIN_12)
#define TRIGGER_PIN_ID GPIO_PIN12
#elif defined(CONFIG_GPIO_TRIGGER_PIN_13)
#define TRIGGER_PIN_ID GPIO_PIN13
#elif defined(CONFIG_GPIO_TRIGGER_PIN_14)
#define TRIGGER_PIN_ID GPIO_PIN14
#elif defined(CONFIG_GPIO_TRIGGER_PIN_15)
#define TRIGGER_PIN_ID GPIO_PIN15
#else
	#error "Provide a GPIO PIN"
#endif 

#define GPIO_TRIGGER (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                            TRIGGER_PORT_ID | TRIGGER_PIN_ID)
#endif  //CONFIG_ARCH_CHIP_STM32

#define GPIO_TRIGGER_ID	0

static uint32_t gpiosconf[] = {  
		[GPIO_TRIGGER_ID] = GPIO_TRIGGER,
};

static void gpio_configure(int gpio_id)
{
#ifdef CONFIG_ARCH_CHIP_STM32
	stm32_configgpio(gpiosconf[gpio_id]);
	stm32_gpiowrite(gpiosconf[gpio_id], 1);
#endif
}

static void gpio_set(int gpio_id)
{
#ifdef CONFIG_ARCH_CHIP_STM32
	stm32_gpiowrite(gpiosconf[gpio_id], 1);
#endif
}

void gpio_backend_trigger(void)
{
	gpio_set(GPIO_TRIGGER_ID);
}

void gpio_backend_configure_trigger(void)
{
	gpio_configure(GPIO_TRIGGER_ID);
}
