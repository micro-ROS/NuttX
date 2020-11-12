/************************************************************************************
 * configs/stm32f103-minimum/src/stm32_ina219.c
 *
 *   Copyright (C) 2018 Erle Robotics (Juan Flores Muñoz). All rights reserved.
 *   Author: Erle Robotics (Juan Flores Muñoz) <juan@erlerobotics.com>
 *  Base on the implementation of BMP180 driver
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/ina219.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "olimex-stm32-e407.h"

#define INA219_ADC(x) (1 << (x - 1))

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_INA219)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define INA219_I2C_PORTNO 1   /* On I2C1 */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_ina219initialize
 *
 * Description:
 *   Initialize and register the INA219 voltage/current sensor.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/ina219"
 *   addr - The full path to the driver to register. E.g., "/dev/ina219"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

int stm32_ina219initialize(FAR const char *devpath, unsigned char addr)
{
	FAR struct i2c_master_s *i2c;
	int ret;
	uint32_t config = (2 << 11) | 
			((INA219_ADC(3) | INA219_ADC(2) | INA219_ADC(1)) << 7) |
			((INA219_ADC(3) | INA219_ADC(2) | INA219_ADC(1)) << 3);

	sninfo("Initializing INA219!\n");

	/* Initialize I2C */

	i2c = stm32_i2cbus_initialize(INA219_I2C_PORTNO);
	if (!i2c) {
		return -ENODEV;
	}

	/* Then register the v sensor */
#if 1
	int32_t shuntval;
	if(addr == 0) {
		shuntval = 50;
	}
	else {
		shuntval = 100;
	}
	ret = ina219_register(devpath, i2c, 0x40 + addr, shuntval, config);
#endif	

#if 0
	ret = ina219_register(devpath, i2c, 0x40 + addr, 100, config);
#endif

	if (ret < 0) {
		snerr("ERROR: Error registering ina219 address %d\n", (int) addr);
	}

	return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_INA219 && CONFIG_STM32_I2C1 */
