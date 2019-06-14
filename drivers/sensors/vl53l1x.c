
/****************************************************************************
 * drivers/sensors/bmp180.c
 * Character driver for the Freescale BMP1801 Barometer Sensor
 *
 *   Copyright (C) 2015 Alan Carvalho de Assis
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <nuttx/sensors/vl53l1x.h>
#include <nuttx/random.h>



#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_VL53L1X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define VLM53L1X_FREQ         100000
//  ResultBuffer results;
/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct vl53l1x_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* BMP180 I2C address */
  int freq;                     /* BMP180 Frequency <= 3.4MHz */

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t vl53l1x_getreg8(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr);
static uint16_t vl53l1x_getreg16(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr);
static void vl53l1x_putreg8(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                           uint8_t regval);
static void vl53l1x_putreg16(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                          uint16_t regval);
static void vl53l1x_putreg32(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                          uint32_t regval);
static  bool vl53l1x_init(FAR struct vl53l1x_dev_s *priv,bool io_2v8);
static bool vl53l1x_setDistanceMode(FAR struct vl53l1x_dev_s *priv,uint8_t mode);
static bool vl53l1x_setMeasurementTimingBudget(FAR struct vl53l1x_dev_s *priv,uint32_t budget_us);
static uint32_t vl53l1x_getMeasurementTimingBudget(FAR struct vl53l1x_dev_s *priv);
static void vl53l1x_startContinuous(FAR struct vl53l1x_dev_s *priv, uint32_t period_ms);
static void vl53l1x_stopContinuous(FAR struct vl53l1x_dev_s *priv );
static bool vl53l1x_timeoutOccurred();
 static void vl53l1x_setupManualCalibration(FAR struct vl53l1x_dev_s *priv);
 static uint32_t vl53l1x_calcMacroPeriod(uint8_t vcsel_period);
 static uint32_t vl53l1x_timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us);
 static uint16_t vl53l1x_encodeTimeout(uint32_t timeout_mclks);
 static uint32_t vl53l1x_decodeTimeout(uint16_t reg_val);

/* Character driver methods */

static int vl53l1x_open(FAR struct file *filep);
static int vl53l1x_close(FAR struct file *filep);
static ssize_t vl53l1x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t vl53l1x_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static ssize_t vl53l1x_ioctl(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_vl53l1xfops =
{
  vl53l1x_open,                 /* open */
  vl53l1x_close,                /* close */
  vl53l1x_read,                 /* read */
  vl53l1x_write,                /* write */
  NULL,                         /* seek */
  vl53l1x_ioctl,                 /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  NULL ,                       /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
 NULL ,                       /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static  bool vl53l1x_init(FAR struct vl53l1x_dev_s *priv,bool io_2v8)
{
  // check model ID and module type registers (values specified in datasheet)
  sninfo("func init!\n");
  if (vl53l1x_getreg16(priv, IDENTIFICATION__MODEL_ID)!= 0xEACC) { sninfo("flaset!\n"); return false; }
  sninfo("func init 2!\n");
  // VL53L1_software_reset() begin

  vl53l1x_putreg8(priv,SOFT_RESET, 0x00);
  usleep(100);
  vl53l1x_putreg8(priv,SOFT_RESET, 0x01);

  // give it some time to boot; otherwise the sensor NACKs during the vl53l1x_getreg8(priv,)
  // call below and the Arduino 101 doesn't seem to handle that well
  usleep(1000);

  // VL53L1_poll_for_boot_completion() begin

  //startTimeout(); //TODO

  // check last_status in case we still get a NACK to try to deal with it correctly
  while (vl53l1x_getreg8(priv,FIRMWARE__SYSTEM_STATUS) != 0 );

  /*  if (checkTimeoutExpired())
    {*/
    /*  did_timeout = true;
      return false;*/
    //}
  //}
  // VL53L1_poll_for_boot_completion() end

  // VL53L1_software_reset() end

  // VL53L1_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    vl53l1x_putreg8(priv,PAD_I2C_HV__EXTSUP_CONFIG,
    vl53l1x_getreg8(priv,PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);
  }

  // store oscillator info for later use
  fast_osc_frequency = vl53l1x_getreg16(priv,OSC_MEASURED__FAST_OSC__FREQUENCY);
  osc_calibrate_val = vl53l1x_getreg16(priv,RESULT__OSC_CALIBRATE_VAL);

  // VL53L1_DataInit() end

  // VL53L1_StaticInit() begin

  // Note that the API does not actually apply the configuration settings below
  // when VL53L1_StaticInit() is called: it keeps a copy of the sensor's
  // register contents in memory and doesn't actually write them until a
  // measurement is started. Writing the configuration here means we don't have
  // to keep it all in memory and avoids a lot of redundant writes later.

  // the API sets the preset mode to LOWPOWER_AUTONOMOUS here:
  // VL53L1_set_preset_mode() begin

  // VL53L1_preset_mode_standard_ranging() begin

  // values labeled "tuning parm default" are from vl53l1_tuning_parm_defaults.h
  // (API uses these in VL53L1_init_tuning_parm_storage_struct())

  // static config
  // API resets PAD_I2C_HV__EXTSUP_CONFIG here, but maybe we don't want to do
  // that? (seems like it would disable 2V8 mode)
  vl53l1x_putreg16(priv,DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate); // should already be this value after reset
  vl53l1x_putreg8(priv,GPIO__TIO_HV_STATUS, 0x02);
  vl53l1x_putreg8(priv,SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8); // tuning parm default
  vl53l1x_putreg8(priv,SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
  vl53l1x_putreg8(priv,ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
  vl53l1x_putreg8(priv,ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
  vl53l1x_putreg8(priv,ALGO__RANGE_MIN_CLIP, 0); // tuning parm default
  vl53l1x_putreg8(priv,ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); // tuning parm default

  // general config
  vl53l1x_putreg16(priv,SYSTEM__THRESH_RATE_HIGH, 0x0000);
  vl53l1x_putreg16(priv,SYSTEM__THRESH_RATE_LOW, 0x0000);
  vl53l1x_putreg8(priv,DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

  // timing config
  // most of these settings will be determined later by distance and timing
  // budget configuration
  vl53l1x_putreg16(priv,RANGE_CONFIG__SIGMA_THRESH, 360); // tuning parm default
  vl53l1x_putreg16(priv,RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default

  // dynamic config

  vl53l1x_putreg8(priv,SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
  vl53l1x_putreg8(priv,SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
  vl53l1x_putreg8(priv,SD_CONFIG__QUANTIFIER, 2); // tuning parm default

  // VL53L1_preset_mode_standard_ranging() end

  // from VL53L1_preset_mode_timed_ranging_*
  // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
  // and things don't seem to work if we don't set GPH back to 0 (which the API
  // does here).
  vl53l1x_putreg8(priv,SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
  vl53l1x_putreg8(priv,SYSTEM__SEED_CONFIG, 1); // tuning parm default

  // from VL53L1_config_low_power_auto_mode
  vl53l1x_putreg8(priv,SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
  vl53l1x_putreg16(priv,DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
  vl53l1x_putreg8(priv,DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS

  // VL53L1_set_preset_mode() end

  // default to long range, 50 ms timing budget
  // note that this is different than what the API defaults to
  vl53l1x_setDistanceMode(priv,Long);
  vl53l1x_setMeasurementTimingBudget(priv,50000);

  // VL53L1_StaticInit() end

  // the API triggers this change in VL53L1_init_and_start_range() once a
  // measurement is started; assumes MM1 and MM2 are disabled
  vl53l1x_putreg16(priv,ALGO__PART_TO_PART_RANGE_OFFSET_MM,
  vl53l1x_getreg16(priv,MM_CONFIG__OUTER_OFFSET_MM) * 4);
  sninfo("casi!\n");
  if(vl53l1x_getreg16(priv,I2C_SLAVE__DEVICE_ADDRESS) == AddressDefault){
    sninfo("Bien!\n");
    return 1;
  }
  else{
    sninfo("Mal!\n");
    return 0;
  }



}

/****************************************************************************
 * Name: vl53l1x_getreg8
 *
 * Description:
 *   Read from an 8-bit VL53L1X register
 *
 ****************************************************************************/

static uint8_t vl53l1x_getreg8(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr)
{
  struct i2c_config_s config;
  uint8_t regval = 0;
  uint8_t reg_addr_aux[2];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Split the I2C direction */
  reg_addr_aux[0] = (regaddr >> 8) & 0xFF;
  reg_addr_aux[1] = regaddr ;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read the register value */

  ret = i2c_read(priv->i2c, &config, &regval, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  return regval;
}

/****************************************************************************
 * Name: bmp180_getreg16
 *
 * Description:
 *   Read two 8-bit from a BMP180 register
 *
 ****************************************************************************/

static uint16_t vl53l1x_getreg16(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr)
{
  struct i2c_config_s config;
  uint16_t msb, lsb;
  uint16_t regval = 0;
  uint8_t reg_addr_aux[2];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Split the I2C direction */
  reg_addr_aux[0] = (regaddr >> 8) & 0xFF;
  reg_addr_aux[1] = regaddr;

  /* Register to read */

  ret = i2c_write(priv->i2c, &config, &reg_addr_aux, 2);
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
 * Name: vl53l1x_putreg8
 *
 * Description:
 *   Write to an 8-bit VL53L1X register
 *
 ****************************************************************************/

static void vl53l1x_putreg8(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t data[3];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  data[0] = (regaddr >> 8) & 0xFF;
  data[1] = regaddr;
  data[2] = regval & 0xFF;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (uint8_t *) &data, 3);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }

  return;
}


/****************************************************************************
 * Name: vl53l1x_putreg16
 *
 * Description:
 *   Write to an 16-bit VL53L1X register
 *
 ****************************************************************************/

static void vl53l1x_putreg16(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                           uint16_t regval)
{
  struct i2c_config_s config;
  uint8_t data[4];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  data[0] = (regaddr >> 8) & 0xFF;
  data[1] = regaddr;
  data[2] = (regval >> 8) & 0xFF;
  data[3] = regval & 0xFF;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (uint8_t *) &data, 4);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }

  return;
}

/****************************************************************************
 * Name: vl53l1x_putreg32
 *
 * Description:
 *   Write to an 32-bit VL53L1X register
 *
 ****************************************************************************/

static void vl53l1x_putreg32(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                           uint32_t regval)
{
  struct i2c_config_s config;
  uint8_t data[7];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->freq;
  config.address   = priv->addr;
  config.addrlen   = 7;

  data[0] = (regaddr >> 8) & 0xFF;
  data[1] = regaddr;
  data[2] = (regval >> 24) & 0xFF;
  data[4] = (regval >> 16) & 0xFF;
  data[5] = (regval >> 8) & 0xFF;
  data[6] = regval & 0xFF ;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (uint8_t *) &data, 7);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
    }

  return;
}

/****************************************************************************
 * Name: vl53l1x_setDistanceMode
 *
 * Description:
 *   Set distance mode to Short, Medium, or Long
 *
 ****************************************************************************/

static bool vl53l1x_setDistanceMode(FAR struct vl53l1x_dev_s *priv, uint8_t mode)
{
  // save existing timing budget
  uint32_t budget_us = vl53l1x_getMeasurementTimingBudget(priv);

  switch (mode)
  {
    case Short:
      // from VL53L1_preset_mode_standard_ranging_short_range()

      // timing config
      vl53l1x_putreg8(priv,RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      vl53l1x_putreg8(priv,RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      vl53l1x_putreg8(priv,RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

      // dynamic config
      vl53l1x_putreg8(priv,SD_CONFIG__WOI_SD0, 0x07);
      vl53l1x_putreg8(priv,SD_CONFIG__WOI_SD1, 0x05);
      vl53l1x_putreg8(priv,SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
      vl53l1x_putreg8(priv,SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default

      break;

    case Medium:
      // from VL53L1_preset_mode_standard_ranging()

      // timing config
      vl53l1x_putreg8(priv,RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
      vl53l1x_putreg8(priv,RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
      vl53l1x_putreg8(priv,RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

      // dynamic config
      vl53l1x_putreg8(priv,SD_CONFIG__WOI_SD0, 0x0B);
      vl53l1x_putreg8(priv,SD_CONFIG__WOI_SD1, 0x09);
      vl53l1x_putreg8(priv,SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
      vl53l1x_putreg8(priv,SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default

      break;

    case Long: // long
      // from VL53L1_preset_mode_standard_ranging_long_range()

      // timing config
      vl53l1x_putreg8(priv,RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
      vl53l1x_putreg8(priv,RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
      vl53l1x_putreg8(priv,RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

      // dynamic config
      vl53l1x_putreg8(priv,SD_CONFIG__WOI_SD0, 0x0F);
      vl53l1x_putreg8(priv,SD_CONFIG__WOI_SD1, 0x0D);
      vl53l1x_putreg8(priv,SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
      vl53l1x_putreg8(priv,SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default

      break;

    default:
      // unrecognized mode - do nothing
      return false;
  }

  // reapply timing budget
  vl53l1x_setMeasurementTimingBudget(priv,budget_us);

  // save mode so it can be returned by getDistanceMode()
  //distance_mode = mode; TODO

  return true;
}

/****************************************************************************
 * Name: vl53l1x_setMeasurementTimingBudget
 *
 * Description:
 *   Set the measurement timing budget in microseconds, which is the time
 *   allowed for one measurement. A longer timing budget allows for more
 *   measurements.
 *
 ****************************************************************************/

static bool vl53l1x_setMeasurementTimingBudget(FAR struct vl53l1x_dev_s *priv,uint32_t budget_us)
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS

  if (budget_us <= TimingGuard) { return false; }

  uint32_t range_config_timeout_us = budget_us -= TimingGuard;
  if (range_config_timeout_us > 1100000) { return false; } // FDA_MAX_TIMING_BUDGET_US * 2

  range_config_timeout_us /= 2;

  // VL53L1_calc_timeout_register_values() begin

  uint32_t macro_period_us;

  // "Update Macro Period for Range A VCSEL Period"
  macro_period_us = vl53l1x_calcMacroPeriod(vl53l1x_getreg8(priv,RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Update Phase timeout - uses Timing A"
  // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg().
  uint32_t phasecal_timeout_mclks = vl53l1x_timeoutMicrosecondsToMclks(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
  vl53l1x_putreg8(priv,PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

  // "Update MM Timing A timeout"
  // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
  // actually ends up with a slightly different value because it gets assigned,
  // retrieved, recalculated with a different macro period, and reassigned,
  // but it probably doesn't matter because it seems like the MM ("mode
  // mitigation"?) sequence steps are disabled in low power auto mode anyway.
  vl53l1x_putreg16(priv,MM_CONFIG__TIMEOUT_MACROP_A, vl53l1x_encodeTimeout(
    vl53l1x_timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing A timeout"
  vl53l1x_putreg16(priv,RANGE_CONFIG__TIMEOUT_MACROP_A, vl53l1x_encodeTimeout(
    vl53l1x_timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // "Update Macro Period for Range B VCSEL Period"
  macro_period_us = vl53l1x_calcMacroPeriod(vl53l1x_getreg8(priv,RANGE_CONFIG__VCSEL_PERIOD_B));

  // "Update MM Timing B timeout"
  // (See earlier comment about MM Timing A timeout.)
  vl53l1x_putreg16(priv,MM_CONFIG__TIMEOUT_MACROP_B, vl53l1x_encodeTimeout(
    vl53l1x_timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing B timeout"
  vl53l1x_putreg16(priv,RANGE_CONFIG__TIMEOUT_MACROP_B, vl53l1x_encodeTimeout(
    vl53l1x_timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // VL53L1_calc_timeout_register_values() end

  return true;
}


/****************************************************************************
 * Name: vl53l1x_getMeasurementTimingBudget
 *
 * Description:
 *   Get the measurement timing budget in microseconds.
 *
 ****************************************************************************/

static uint32_t vl53l1x_getMeasurementTimingBudget(FAR struct vl53l1x_dev_s *priv)
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
  // enabled: VHV, PHASECAL, DSS1, RANGE

  // VL53L1_get_timeouts_us() begin

  // "Update Macro Period for Range A VCSEL Period"
  uint32_t macro_period_us = vl53l1x_calcMacroPeriod(vl53l1x_getreg8(priv,RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Get Range Timing A timeout"

//  uint32_t range_config_timeout_us = vl53l1x_timeoutMclksToMicroseconds(vl53l1x_decodeTimeout(vl53l1x_getreg16(priv,RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us); TODO

  // VL53L1_get_timeouts_us() end

  //return  2 * range_config_timeout_us + TimingGuard; TODO
}


/****************************************************************************
 * Name: vl53l1x_startContinuous
 *
 * Description:
 *   Start continuous ranging measurements, with the given inter-measurement
 *   period in milliseconds determining how often the sensor takes a measurement.
 *
 ****************************************************************************/

static void vl53l1x_startContinuous(FAR struct vl53l1x_dev_s *priv, uint32_t period_ms)
{
  // from VL53L1_set_inter_measurement_period_ms()
  vl53l1x_putreg32(priv,SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);

  vl53l1x_putreg8(priv,SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
  vl53l1x_putreg8(priv,SYSTEM__MODE_START, 0x40); // mode_range__timed
}


/****************************************************************************
 * Name: vl53l1x_stopContinuous
 *
 * Description:
 *   Stop continuous measurements.
 *
 ****************************************************************************/

static void vl53l1x_stopContinuous(FAR struct vl53l1x_dev_s *priv)
{
  vl53l1x_putreg8(priv,SYSTEM__MODE_START, 0x80); // mode_range__abort

  // VL53L1_low_power_auto_data_stop_range() begin

  calibrated = false;

  // "restore vhv configs"
  if (saved_vhv_init != 0)
  {
    vl53l1x_putreg8(priv,VHV_CONFIG__INIT, saved_vhv_init);
  }
  if (saved_vhv_timeout != 0)
  {
     vl53l1x_putreg8(priv,VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout);
  }

  // "remove phasecal override"
  vl53l1x_putreg8(priv,PHASECAL_CONFIG__OVERRIDE, 0x00);

  // VL53L1_low_power_auto_data_stop_range() end
}


/****************************************************************************
 * Name: vl53l1x_timeoutOccurred
 *
 * Description:
 *   Did a timeout occur in one of the read functions since the last call to.
 *
 ****************************************************************************/

static bool vl53l1x_timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}


/****************************************************************************
 * Name: vl53l1x_setupManualCalibration
 *
 * Description:
 *   "Setup ranges after the first one in low power auto mode by turning off
 *    FW calibration steps and programming static values"
 *
 ****************************************************************************/

 static void vl53l1x_setupManualCalibration(FAR struct vl53l1x_dev_s *priv)
 {
   // "save original vhv configs"
   saved_vhv_init = vl53l1x_getreg8(priv,VHV_CONFIG__INIT);
   saved_vhv_timeout = vl53l1x_getreg8(priv,VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);

   // "disable VHV init"
   vl53l1x_putreg8(priv,VHV_CONFIG__INIT, saved_vhv_init & 0x7F);

   // "set loop bound to tuning param"
   vl53l1x_putreg8(priv,VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
     (saved_vhv_timeout & 0x03) + (3 << 2)); // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

   // "override phasecal"
   vl53l1x_putreg8(priv,PHASECAL_CONFIG__OVERRIDE, 0x01);
   vl53l1x_putreg8(priv,CAL_CONFIG__VCSEL_START, vl53l1x_getreg8(priv,PHASECAL_RESULT__VCSEL_START));
 }


/****************************************************************************
* Name: vl53l1x_readResults
*
* Description:
*   Read measurement results into buffer.
*
****************************************************************************/


 static void vl53l1x_readResults()
 {
   /*Wire.beginTransmission(address);
   Wire.write((RESULT__RANGE_STATUS >> 8) & 0xFF); // reg high byte
   Wire.write( RESULT__RANGE_STATUS       & 0xFF); // reg low byte
   last_status = Wire.endTransmission();

   Wire.requestFrom(address, (uint8_t)17);

   results.range_status = Wire.read();

   Wire.read(); // report_status: not used

   results.stream_count = Wire.read();

   results.dss_actual_effective_spads_sd0  = (uint16_t)Wire.read() << 8; // high byte
   results.dss_actual_effective_spads_sd0 |=           Wire.read();      // low byte

   Wire.read(); // peak_signal_count_rate_mcps_sd0: not used
   Wire.read();

   results.ambient_count_rate_mcps_sd0  = (uint16_t)Wire.read() << 8; // high byte
   results.ambient_count_rate_mcps_sd0 |=           Wire.read();      // low byte

   Wire.read(); // sigma_sd0: not used
   Wire.read();

   Wire.read(); // phase_sd0: not used
   Wire.read();

   results.final_crosstalk_corrected_range_mm_sd0  = (uint16_t)Wire.read() << 8; // high byte
   results.final_crosstalk_corrected_range_mm_sd0 |=           Wire.read();      // low byte

   results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0  = (uint16_t)Wire.read() << 8; // high byte
   results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |=           Wire.read();      // low byte*/
 }

/****************************************************************************
* Name: vl53l1x_updateDSS
*
* Description:
*   Perform Dynamic SPAD Selection calculation/update.
*
****************************************************************************/

static void vl53l1x_updateDSS(FAR struct vl53l1x_dev_s *priv)
{
  uint16_t spadCount = results.dss_actual_effective_spads_sd0;

  if (spadCount != 0)
  {
    // "Calc total rate per spad"

    uint32_t totalRatePerSpad =
      (uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
      results.ambient_count_rate_mcps_sd0;

    // "clip to 16 bits"
    if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

    // "shift up to take advantage of 32 bits"
    totalRatePerSpad <<= 16;

    totalRatePerSpad /= spadCount;

    if (totalRatePerSpad != 0)
    {
      // "get the target rate and shift up by 16"
      uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

      // "clip to 16 bit"
      if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

      // "override DSS config"
      vl53l1x_putreg16(priv,DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
      // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

      return;
    }
  }

  // If we reached this point, it means something above would have resulted in a
  // divide by zero.
  // "We want to gracefully set a spad target, not just exit with an error"

   // "set target to mid point"
   vl53l1x_putreg16(priv,DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

/****************************************************************************
* Name: vl53l1x_getRangingData
*
* Description:
*   Get range, status, rates from results buffer.
*
****************************************************************************/

static void vl53l1x_getRangingData()
{
  // VL53L1_copy_sys_and_core_results_to_range_results() begin
/*
  uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;

  // "apply correction gain"
  // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
  // Basically, this appears to scale the result by 2011/2048, or about 98%
  // (with the 1024 added for proper rounding).
  ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

  // VL53L1_copy_sys_and_core_results_to_range_results() end

  // set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
  // mostly based on ConvertStatusLite()
  switch(results.range_status)
  {
    case 17: // MULTCLIPFAIL
    case 2: // VCSELWATCHDOGTESTFAILURE
    case 1: // VCSELCONTINUITYTESTFAILURE
    case 3: // NOVHVVALUEFOUND
      // from SetSimpleData()
      ranging_data.range_status = HardwareFail;
      break;

    case 13: // USERROICLIP
     // from SetSimpleData()
      ranging_data.range_status = MinRangeFail;
      break;

    case 18: // GPHSTREAMCOUNT0READY
      ranging_data.range_status = SynchronizationInt;
      break;

    case 5: // RANGEPHASECHECK
      ranging_data.range_status =  OutOfBoundsFail;
      break;

    case 4: // MSRCNOTARGET
      ranging_data.range_status = SignalFail;
      break;

    case 6: // SIGMATHRESHOLDCHECK
      ranging_data.range_status = SignalFail;
      break;

    case 7: // PHASECONSISTENCY
      ranging_data.range_status = WrapTargetFail;
      break;

    case 12: // RANGEIGNORETHRESHOLD
      ranging_data.range_status = XtalkSignalFail;
      break;

    case 8: // MINCLIP
      ranging_data.range_status = RangeValidMinRangeClipped;
      break;

    case 9: // RANGECOMPLETE
      // from VL53L1_copy_sys_and_core_results_to_range_results()
      if (results.stream_count == 0)
      {
        ranging_data.range_status = RangeValidNoWrapCheckFail;
      }
      else
      {
        ranging_data.range_status = RangeValid;
      }
      break;

    default:
      ranging_data.range_status = None;
  }

  // from SetSimpleData()
  ranging_data.peak_signal_count_rate_MCPS =
    countRateFixedToFloat(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
  ranging_data.ambient_count_rate_MCPS =
    countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);*/
}


/****************************************************************************
* Name: vl53l1x_decodeTimeout
*
* Description:
*   Decode sequence step timeout in MCLKs from register value.
*
****************************************************************************/

static uint32_t vl53l1x_decodeTimeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

/****************************************************************************
* Name: vl53l1x_encodeTimeout
*
* Description:
*   Encode sequence step timeout register value from timeout in MCLKs.
*
****************************************************************************/

static uint16_t vl53l1x_encodeTimeout(uint32_t timeout_mclks)
{
  // encoded format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

/****************************************************************************
* Name: vl53l1x_timeoutMclksToMicroseconds
*
* Description:
*   Convert sequence step timeout from macro periods to microseconds with given
*   macro period in microseconds (12.12 format).
*
****************************************************************************/

static uint32_t vl53l1x_timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

/****************************************************************************
* Name: vl53l1x_vl53l1x_timeoutMicrosecondsToMclks
*
* Description:
*   Convert sequence step timeout from microseconds to macro periods with given
*   macro period in microseconds (12.12 format).
*
****************************************************************************/

static uint32_t vl53l1x_timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

/****************************************************************************
* Name: vl53l1x_vl53l1x_calcMacroPeriod
*
* Description:
*   Calculate macro period in microseconds (12.12 format) with given VCSEL period
*   assumes fast_osc_frequency has been read and stored.
*
****************************************************************************/

static uint32_t vl53l1x_calcMacroPeriod(uint8_t vcsel_period)
{
  // from VL53L1_calc_pll_period_us()
  // fast osc frequency in 4.12 format; PLL period in 0.24 format
  uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

  // from VL53L1_decode_vcsel_period()
  uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

  // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
  uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
  macro_period_us >>= 6;
  macro_period_us *= vcsel_period_pclks;
  macro_period_us >>= 6;

  return macro_period_us;
}


/****************************************************************************
 * Name: vl53l1x_open
 *
 * Description:
 *   This function is called whenever the VL53L1X device is opened.
 *
 ****************************************************************************/

static int vl53l1x_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: vl53l1x_close
 *
 * Description:
 *   This routine is called when the VL53L1X device is closed.
 *
 ****************************************************************************/

static int vl53l1x_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: vl53l1x_read
 ****************************************************************************/

static ssize_t vl53l1x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct vl53l1x_dev_s *priv  = inode->i_private;
  FAR uint32_t            *press = (FAR uint32_t *) buffer;

  if (!buffer)
    {
      snerr("ERROR: Buffer is null\n");
      return -1;
    }

  if (buflen != 4)
    {
      snerr("ERROR: You can't read something other than 32 bits (4 bytes)\n");
      return -1;
    }




  //startTimeout();
/*  while (!dataReady())
  {
    if (checkTimeoutExpired())
    {
      did_timeout = true;
    //  ranging_data.range_status = None;
      ranging_data.range_mm = 0;
      ranging_data.peak_signal_count_rate_MCPS = 0;
      ranging_data.ambient_count_rate_MCPS = 0;
      return ranging_data.range_mm;
    }
  }


readResults();

if (!calibrated)
{
  setupManualCalibration();
  calibrated = true;
}

updateDSS();

getRangingData();

vl53l1x_putreg8(priv,SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range

return ranging_data.range_mm;*/
}

/****************************************************************************
 * Name: vl53l1x_write
 ****************************************************************************/

static ssize_t vl53l1x_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}



/****************************************************************************
 * Name: vl53l1x_ioctl
 ****************************************************************************/

static ssize_t vl53l1x_ioctl(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  //return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vl53l1x_register
 *
 * Description:
 *   Register the VL53L1X character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/tof"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             VL53L1X TOF
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vl53l1x_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct vl53l1x_dev_s *priv;
  int ret;

  /* Initialize the vl53l1x device structure */

  priv = (FAR struct vl53l1x_dev_s *)kmm_malloc(sizeof(struct vl53l1x_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = AddressDefault;
  priv->freq = VLM53L1X_FREQ;

  /* Check Init the device */
sninfo("Init!\n");
  if(vl53l1x_init(priv,false))
  {
    snerr("ERROR: Failed to register driver: %d\n", ret);
    kmm_free(priv);
    return ret;
  }
  sninfo("Init fin!\n");
  /* Register the character driver */

  ret = register_driver(devpath, &g_vl53l1xfops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("foo driver loaded successfully!\n");
  return ret;


}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BMP180 */
