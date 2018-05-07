/****************************************************************************
 * drivers/sensors/hih6130.c
 * Character driver for the HIH-6130 temperature/humidity sensor
 *
 *   Copyright (C) 2018 Erle Robotics (Juan Flores Muñoz). All rights reserved.
 *   Author: Erle Robotics (Juan Flores Muñoz) <juan@erlerobotics.com>
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
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/hih6130.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_HIH6130)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 #define HIH6130_ADDRESS 0x27
 #define HIH6130_FREQ         100000

//commands and constants
 #define HIH6130_READREG 0x00

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct hih6130_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* hih6130 I2C address */
  int freq;                     /* hih6130 Frequency <= 3.4MHz */
  bool fahrenheit;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t hih6130_getreg8(FAR struct hih6130_dev_s *priv, uint8_t regaddr);
static uint16_t hih6130_getreg16(FAR struct hih6130_dev_s *priv, uint8_t regaddr);
static void hih6130_putreg8(FAR struct hih6130_dev_s *priv, uint8_t regaddr,
                           uint8_t regval);
static uint32_t hih6130_getreg32(FAR struct hih6130_dev_s *priv, uint8_t regaddr);
static int humidity_temp(FAR struct hih6130_dev_s *priv, FAR struct hih6130_s *buffer);

/* Character driver methods */

static int hih6130_open(FAR struct file *filep);
static int hih6130_close(FAR struct file *filep);
static ssize_t hih6130_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static int     hih6130_ioctl(FAR struct file *filep,int cmd,unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_hih6130fops =
{
  hih6130_open,                 /* open */
  hih6130_close,                /* close */
  hih6130_read,                 /* read */
  0,                 			      /* write */
  0,                            /* seek */
  hih6130_ioctl,                /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                            /* poll */
#endif
  0                             /* unlink */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hih6130_getreg8
 *
 * Description:
 *   Read from an 8-bit HIH6130 register
 *
 ****************************************************************************/

static uint8_t hih6130_getreg8(FAR struct hih6130_dev_s *priv, uint8_t regaddr)
{
  struct i2c_config_s config;
  uint8_t regval = 0;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

   //Read the register value 

  ret = i2c_read(priv->i2c, &config, &regval, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  return regval;
}

/****************************************************************************
 * Name: hih6130_getreg16
 *
 * Description:
 *   Read two 8-bit from a HIH6130 register
 *
 ****************************************************************************/

static uint16_t hih6130_getreg16(FAR struct hih6130_dev_s *priv, uint8_t regaddr)
{
  struct i2c_config_s config;
  uint16_t msb, lsb;
  uint16_t regval = 0;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Register to read */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read register */
  
  ret = i2c_read(priv->i2c, &config, (uint8_t *)&regval, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* MSB and LSB are inverted */

  msb = (regval & 0xFF);
  lsb = (regval & 0xFF00) >> 8;

  regval = (msb << 8) | lsb;

  return regval;
}
/****************************************************************************
 * Name: hih6130_getreg32
 *
 * Description:
 *   Read four 8-bit from a HIH6130 register
 *
 ****************************************************************************/
static uint32_t hih6130_getreg32(FAR struct hih6130_dev_s *priv, uint8_t regaddr)
{
  struct i2c_config_s config;
  uint16_t byte1, byte2, byte3, byte4;//byte1 is the msb and byte4 is the lsb
  uint32_t regval = 0;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Register to read */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read register */
  
  ret = i2c_read(priv->i2c, &config, (uint8_t *)&regval, 4);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* The bytes are in the opposite order of importance*/
  byte1=(regval & 0xFF);
  byte2=(regval & 0xFF00) >>8;
  byte3=(regval & 0xFFFF00) >> 16;
  byte4=(regval & 0xFFFFFF00) >> 24;
 
  regval=(byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4;

  return regval;
}


/****************************************************************************
 * Name: hih6130_putreg8
 *
 * Description:
 *   Write to an 8-bit HIH6130 register
 *
 ****************************************************************************/

static void hih6130_putreg8(FAR struct hih6130_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t data[2];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  data[0] = regaddr;
  data[1] = regval;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (uint8_t *) &data, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }

  return;
}

static int hih6130_open(FAR struct file *filep)
{
  sninfo("Device correctly open!");
  return OK;
}

static int hih6130_close(FAR struct file *filep)
{
  return OK;
}

static int humidity_temp(FAR struct hih6130_dev_s *priv, FAR struct hih6130_s *buffer){
  uint32_t data = 0;
  uint8_t sensor_status=0;
  uint16_t humidity_data,humidity = 0;
  int16_t temperature_data, temperature = 0;
  data = hih6130_getreg32(priv, HIH6130_READREG);
  //This is the data that we recieve from the device
  //Are four bytes
  sninfo("HEX Data: 0x%04x\n", data);
  //First we check the state of the sensor (First two bits)
  sensor_status=(data & 0xFFFFFF00) >> 30;
  sninfo ("Sensor status %01x \n",sensor_status);
  if(sensor_status == 0 || sensor_status == 1){
    //sensor status = 0, sensor is working correctly
    //sensor status = 1, sensor is working correctly but it's recommendable to slow the speed of the bus
    //because the data that is showing is from the buffer
    //Getting the humidity data
    humidity_data = (data & 0xFFFFFF00) >> 16;
    humidity_data = (humidity_data) << 2;
    humidity_data = (humidity_data) >> 2;
    sninfo ("humidity_hex_data %04x \n",humidity_data);
    humidity=humidity_data*100/(16384 -1);
    sninfo("humidity value %i \n",humidity);
    //Getting the temperature data
    temperature_data = (data & 0xFFFF) >> 2;
    sninfo ("temp_hex_data %04x \n",temperature_data);
    temperature = temperature_data*165/(16384-1) - 40;
    sninfo("temperature value %i \n",temperature);
    
    if(priv->fahrenheit){
      temperature=(temperature*1.8) + 32;
    }
    buffer->temp=temperature;
    buffer->hum=humidity;
     
    return OK;
    
  }
  else if(sensor_status == 2){
    //The sensor is in command mode (programming mode), this mode shouldn't be see during normal mode
    //Somenthing is wrong
    return -11;
  }
  else{
    //Diagnostic mode
    return -22;
  }
   



 
}

static ssize_t hih6130_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
	FAR struct inode        *inode = filep->f_inode;
  FAR struct hih6130_dev_s *priv  = inode->i_private;

  FAR struct hih6130_s *ptr = (FAR struct hih6130 *) buffer;

  int state; 

  
  struct hih6130_s measure;

  //This function active the sensor and ask for the measured value
  state=humidity_temp(priv,&measure);
  if(state == OK){
    //If everything is ok, we will save the data to the buffer and we will return the number of bytes
    *ptr++ = measure;//saving the measure to the buffer
    return sizeof(struct hih6130_s);
  }
  else{
    //In this case, somenthing have been wrong, so we return the code of the error that is negative
    return state;
  }

  
 
  return OK;
}

static int hih6130_ioctl(FAR struct file *filep,int cmd,unsigned long arg){
  FAR struct inode      *inode = filep->f_inode;
  FAR struct hih6130_dev_s *priv   = inode->i_private;

  switch (cmd)
    {
      //You can choose between temperature measure or humidity measure
      case SNIOC_FAHRENHEIT:
      {
        sninfo("Set fahrenheit\n");
        priv->fahrenheit=true;
      }
      break;
      case SNIOC_CENTIGRADE:
      {
        sninfo("Set celsius\n");
        priv->fahrenheit=false;
      }
      break;
    }
}



int hih6130_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct hih6130_dev_s *priv;
  int ret;

  /* Initialize the HIH6130 device structure */

  priv = (FAR struct hih6130_dev_s *)kmm_malloc(sizeof(struct hih6130_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = HIH6130_ADDRESS;
  priv->freq = HIH6130_FREQ;
  priv->fahrenheit=false;

  

  ret = register_driver(devpath, &g_hih6130fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_HIH6130*/
