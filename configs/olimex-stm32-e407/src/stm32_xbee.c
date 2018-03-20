/************************************************************************************
 * configs/stm32f4discovery/src/stm32_mfrc522.c
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/xbee.h>

#include "stm32_gpio.h"
#include "stm32_exti.h"
#include "stm32_spi.h"

#include "stm32.h"
#include "olimex-stm32-e407.h"

#if defined(CONFIG_SPI) && defined(CONFIG_STM32_SPI1) && defined(CONFIG_IEEE802154_XBEE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_priv_s
{
  struct xbee_lower_s dev;
  xcpt_t handler;
  FAR void *arg;
  uint32_t rstcfg;
  uint32_t attncfg;
  uint8_t spidev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* IRQ/GPIO access callbacks.  These operations all hidden behind
 * callbacks to isolate the XBee driver from differences in GPIO
 * interrupt handling by varying boards and MCUs.  If possible,
 * interrupts should be configured on both rising and falling edges
 * so that contact and loss-of-contact events can be detected.
 *
 *   reset        - Reset the XBee using the reset pin
 *   attn_attach  - Attach the XBee interrupt handler to the GPIO
 *                 interrupt (ATTN)
 *   attn_enable  - Enable or disable the GPIO interrupt
 *   attn_poll    - Poll the current state of the GPIO interrupt (ATTN)
 */

static void stm32_reset(FAR const struct xbee_lower_s *lower);
static int  stm32_attach_attn(FAR const struct xbee_lower_s *lower,
                              xcpt_t handler, FAR void *arg);
static void stm32_enable_attn(FAR const struct xbee_lower_s *lower, bool state);
static bool stm32_poll_attn(FAR const struct xbee_lower_s *lower);
static int  stm32_xbee_devsetup(FAR struct stm32_priv_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the XBee
 * driver.  This structure provides information about the configuration
 * of the XBee and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */
 #define GPIO_XBEE_RST  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|\
                             GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN15)/*PG15/D9*/
 #define GPIO_XBEE_INT   (GPIO_INPUT|GPIO_FLOAT|\
                             GPIO_EXTI|GPIO_PORTG|GPIO_PIN12)/*PG12/D8*/


static struct stm32_priv_s g_xbee_priv =
{
  .dev.reset    = stm32_reset,
  .dev.attach   = stm32_attach_attn,
  .dev.enable   = stm32_enable_attn,
  .dev.poll     = stm32_poll_attn,
  .handler      = NULL,
  .arg          = NULL,
  .rstcfg       = GPIO_XBEE_RST,
  .attncfg      = GPIO_XBEE_INT,
  .spidev       = 1,
};

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define XBEE_SPI_PORTNO 1   /* On SPI1 */


/****************************************************************************
 * Private Functions
 ****************************************************************************/

 static void stm32_reset(FAR const struct xbee_lower_s *lower)
 {
   //This function execute the reset
   FAR struct stm32_priv_s *priv = (FAR struct stm32_priv_s *)lower;

   DEBUGASSERT(priv != NULL);

   /* Hold reset line low for min. 200ns */

   stm32_gpiowrite(priv->rstcfg, false);
   up_udelay(1);
   stm32_gpiowrite(priv->rstcfg, true);

   up_mdelay(100);
 }

 static int stm32_attach_attn(FAR const struct xbee_lower_s *lower,
                             xcpt_t handler, FAR void *arg)
 {
   FAR struct stm32_priv_s *priv = (FAR struct stm32_priv_s *)lower;

   DEBUGASSERT(priv != NULL);

   /* Just save the handler for use when the interrupt is enabled */

   priv->handler = handler;
   priv->arg     = arg;
   return OK;
 }


 static void stm32_enable_attn(FAR const struct xbee_lower_s *lower,
                              bool state)
 {
   FAR struct stm32_priv_s *priv = (FAR struct stm32_priv_s *)lower;

   /* The caller should not attempt to enable interrupts if the handler
    * has not yet been 'attached'
    */

   DEBUGASSERT(priv != NULL && (priv->handler != NULL || !state));

   /* Attach and enable, or detach and disable */

   if (state)
     {
       (void)stm32_gpiosetevent(priv->attncfg, false, true, true,
                                priv->handler, priv->arg);
     }
   else
     {
       (void)stm32_gpiosetevent(priv->attncfg, false, false, false,
                                NULL, NULL);
     }
 }

 static bool stm32_poll_attn(FAR const struct xbee_lower_s *lower)
 {
   //the callback function??
   FAR struct stm32_priv_s *priv = (FAR struct stm32_priv_s *)lower;

   return !stm32_gpioread(priv->attncfg);
 }

 static int stm32_xbee_devsetup(FAR struct stm32_priv_s *priv)
 {
   FAR struct spi_dev_s *spi;
   XBEEHANDLE xbee;
   int ret;

   /* Configure the Reset and Attention pins */

   stm32_configgpio(priv->rstcfg);
   stm32_configgpio(priv->attncfg);

   /* Initialize the SPI bus and get an instance of the SPI interface */
   wlerr("iniciando el bus\n");
   spi = stm32_spibus_initialize(priv->spidev);
   if (spi == NULL)
     {
       wlerr("ERROR: Failed to initialize SPI bus %d\n", priv->spidev);
       return -ENODEV;
     }

   /* Initialize and register the SPI XBee device */
   wlerr("iniciando el xbee\n");
   xbee = xbee_init(spi, &priv->dev);
   if (xbee == NULL)
     {
       wlerr("ERROR: Failed to initialize XBee driver%d\n", priv->dev);
       return -ENODEV;
     }

   /* Register the XBee netdev providing it the XBee MAC layer to interface with */
   wlerr("registrando el xbee\n");
   ret = xbee_netdev_register(xbee);
   if (ret < 0)
     {
       wlerr("ERROR: Failed to register the XBee MAC network driver wpan%d: %d\n",
             0, ret);
       return ret;
     }

   return OK;
 }


/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_mfrc522initialize
 *
 * Description:
 *
 *
 * Input Parameters:
 *
 *
 * Returned Value:
 *
 *
 ************************************************************************************/


int stm32_xbee_initialize(void)
{
  int ret;
  wlinfo("Configuring XBee!\n");

  ret = stm32_xbee_devsetup(&g_xbee_priv);//mirar de donde viene esta estructura
  if (ret < 0)
    {
      wlerr("ERROR: Failed to initialize XBee: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_SPI && CONFIG_IEEE802154_XBEE */
