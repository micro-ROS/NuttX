
/****************************************************************************
 * drivers/sensors/vl53l1x.c
 * Character driver for the ST VL53L1X Distance and brigh sensor.
 *
 *   Copyright (C) 2019 Acutronics Robotics
 *   Author: Acutronics Robotics (Juan Flores Muñoz) <juan@erlerobotics.com>
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
#include <nuttx/sensors/ioctl.h>
#include <nuttx/random.h>


#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_VL53L1X)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define VLM53L1X_FREQ         100000
#define VL53L1X_ADDR  0x29

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct vl53l1x_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* VL53L0X I2C address */
  int freq;                     /* VL53L0X Frequency */

};

const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
0x00, /* 0x32 : not user-modifiable */
0x02, /* 0x33 : not user-modifiable */
0x08, /* 0x34 : not user-modifiable */
0x00, /* 0x35 : not user-modifiable */
0x08, /* 0x36 : not user-modifiable */
0x10, /* 0x37 : not user-modifiable */
0x01, /* 0x38 : not user-modifiable */
0x01, /* 0x39 : not user-modifiable */
0x00, /* 0x3a : not user-modifiable */
0x00, /* 0x3b : not user-modifiable */
0x00, /* 0x3c : not user-modifiable */
0x00, /* 0x3d : not user-modifiable */
0xff, /* 0x3e : not user-modifiable */
0x00, /* 0x3f : not user-modifiable */
0x0F, /* 0x40 : not user-modifiable */
0x00, /* 0x41 : not user-modifiable */
0x00, /* 0x42 : not user-modifiable */
0x00, /* 0x43 : not user-modifiable */
0x00, /* 0x44 : not user-modifiable */
0x00, /* 0x45 : not user-modifiable */
0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
0x0b, /* 0x47 : not user-modifiable */
0x00, /* 0x48 : not user-modifiable */
0x00, /* 0x49 : not user-modifiable */
0x02, /* 0x4a : not user-modifiable */
0x0a, /* 0x4b : not user-modifiable */
0x21, /* 0x4c : not user-modifiable */
0x00, /* 0x4d : not user-modifiable */
0x00, /* 0x4e : not user-modifiable */
0x05, /* 0x4f : not user-modifiable */
0x00, /* 0x50 : not user-modifiable */
0x00, /* 0x51 : not user-modifiable */
0x00, /* 0x52 : not user-modifiable */
0x00, /* 0x53 : not user-modifiable */
0xc8, /* 0x54 : not user-modifiable */
0x00, /* 0x55 : not user-modifiable */
0x00, /* 0x56 : not user-modifiable */
0x38, /* 0x57 : not user-modifiable */
0xff, /* 0x58 : not user-modifiable */
0x01, /* 0x59 : not user-modifiable */
0x00, /* 0x5a : not user-modifiable */
0x08, /* 0x5b : not user-modifiable */
0x00, /* 0x5c : not user-modifiable */
0x00, /* 0x5d : not user-modifiable */
0x01, /* 0x5e : not user-modifiable */
0xdb, /* 0x5f : not user-modifiable */
0x0f, /* 0x60 : not user-modifiable */
0x01, /* 0x61 : not user-modifiable */
0xf1, /* 0x62 : not user-modifiable */
0x0d, /* 0x63 : not user-modifiable */
0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
0x68, /* 0x65 : Sigma threshold LSB */
0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
0x80, /* 0x67 : Min count Rate LSB */
0x08, /* 0x68 : not user-modifiable */
0xb8, /* 0x69 : not user-modifiable */
0x00, /* 0x6a : not user-modifiable */
0x00, /* 0x6b : not user-modifiable */
0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
0x00, /* 0x6d : Intermeasurement period */
0x0f, /* 0x6e : Intermeasurement period */
0x89, /* 0x6f : Intermeasurement period LSB */
0x00, /* 0x70 : not user-modifiable */
0x00, /* 0x71 : not user-modifiable */
0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x73 : distance threshold high LSB */
0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x75 : distance threshold low LSB */
0x00, /* 0x76 : not user-modifiable */
0x01, /* 0x77 : not user-modifiable */
0x0f, /* 0x78 : not user-modifiable */
0x0d, /* 0x79 : not user-modifiable */
0x0e, /* 0x7a : not user-modifiable */
0x0e, /* 0x7b : not user-modifiable */
0x00, /* 0x7c : not user-modifiable */
0x00, /* 0x7d : not user-modifiable */
0x02, /* 0x7e : not user-modifiable */
0xc7, /* 0x7f : ROI center, use SetROI() */
0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
0x9B, /* 0x81 : not user-modifiable */
0x00, /* 0x82 : not user-modifiable */
0x00, /* 0x83 : not user-modifiable */
0x00, /* 0x84 : not user-modifiable */
0x01, /* 0x85 : not user-modifiable */
0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t vl53l1x_getreg8(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr);
static uint16_t vl53l1x_getreg16(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr);
static uint32_t vl53l1x_getreg32(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr);
static void vl53l1x_putreg8(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                           uint8_t regval);
static void vl53l1x_putreg16(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                          uint16_t regval);
static void vl53l1x_putreg32(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr,
                          uint32_t regval);


static  void VL53L1X_SetI2CAddress(uint8_t new_address);
static  void VL53L1X_SensorInit(struct vl53l1x_dev_s *priv);
static  void VL53L1X_ClearInterrupt(struct vl53l1x_dev_s *priv);
static  void VL53L1X_SetInterruptPolarity(struct vl53l1x_dev_s *priv , uint8_t IntPol);
static  void VL53L1X_GetInterruptPolarity(uint8_t *pIntPol);
static  void VL53L1X_StartRanging(struct vl53l1x_dev_s *priv);
static  void VL53L1X_StopRanging(struct vl53l1x_dev_s *priv);
static  void VL53L1X_CheckForDataReady(struct vl53l1x_dev_s *priv, uint8_t *isDataReady);
static  void VL53L1X_SetTimingBudgetInMs(struct vl53l1x_dev_s *priv, uint16_t TimingBudgetInMs);
static  void VL53L1X_GetTimingBudgetInMs(struct vl53l1x_dev_s *priv, uint16_t *pTimingBudgetInMs);
static  void VL53L1X_SetDistanceMode(struct vl53l1x_dev_s *priv, uint16_t DistanceMode);
static  void VL53L1X_GetDistanceMode(struct vl53l1x_dev_s *priv, uint16_t *pDistanceMode);
static  void VL53L1X_SetInterMeasurementInMs(struct vl53l1x_dev_s *priv, uint16_t InterMeasurementInMs);
static  void VL53L1X_GetInterMeasurementInMs(struct vl53l1x_dev_s *priv, uint16_t * pIM);
static  void VL53L1X_BootState(struct vl53l1x_dev_s *priv, uint8_t *state);
static  void VL53L1X_GetSensorId(struct vl53l1x_dev_s *priv, uint16_t *id);
static  void VL53L1X_GetDistance(struct vl53l1x_dev_s *priv, uint16_t *distance);
static  void VL53L1X_GetSignalPerSpad(struct vl53l1x_dev_s *priv, uint16_t *signalPerSp);
static  void VL53L1X_GetAmbientPerSpad(struct vl53l1x_dev_s *priv, uint16_t *amb);
static  void VL53L1X_GetSignalRate(struct vl53l1x_dev_s *priv, uint16_t *signalRate);
static  void VL53L1X_GetSpadNb(struct vl53l1x_dev_s *priv, uint16_t *spNb);
static  void VL53L1X_GetAmbientRate(struct vl53l1x_dev_s *priv, uint16_t *ambRate);
static  void VL53L1X_GetRangeStatus(struct vl53l1x_dev_s *priv, uint8_t *rangeStatus);
static  void VL53L1X_SetOffset(struct vl53l1x_dev_s *priv, int16_t OffsetValue);
static  void VL53L1X_GetOffset(struct vl53l1x_dev_s *priv, int16_t *Offset);
static  void VL53L1X_SetXtalk(struct vl53l1x_dev_s *priv, uint16_t XtalkValue);
static  void VL53L1X_GetXtalk(struct vl53l1x_dev_s *priv, uint16_t *Xtalk);
static  void VL53L1X_SetDistanceThreshold(struct vl53l1x_dev_s *priv, uint16_t ThreshLow,
				  uint16_t ThreshHigh, uint8_t Window,
				  uint8_t IntOnNoTarget);
static  void VL53L1X_GetDistanceThresholdWindow(struct vl53l1x_dev_s *priv, uint16_t *window);
static  void VL53L1X_GetDistanceThresholdLow(struct vl53l1x_dev_s *priv, uint16_t *low);
static  void VL53L1X_GetDistanceThresholdHigh(struct vl53l1x_dev_s *priv, uint16_t *high);
static  void VL53L1X_SetROI(struct vl53l1x_dev_s *priv, uint16_t X, uint16_t Y);
static  void VL53L1X_GetROI_XY(struct vl53l1x_dev_s *priv, uint16_t *ROI_X, uint16_t *ROI_Y);
static  void VL53L1X_SetSignalThreshold(struct vl53l1x_dev_s *priv, uint16_t signal);
static  void VL53L1X_GetSignalThreshold(struct vl53l1x_dev_s *priv, uint16_t *signal);
static  void VL53L1X_SetSigmaThreshold(struct vl53l1x_dev_s *priv, uint16_t sigma);
static  void VL53L1X_GetSigmaThreshold(struct vl53l1x_dev_s *priv, uint16_t *signal);
static  void VL53L1X_StartTemperatureUpdate();
static  void VL53L1X_CalibrateOffset(struct vl53l1x_dev_s *priv, uint16_t TargetDistInMm, int16_t *offset);
static int8_t VL53L1X_CalibrateXtalk(struct vl53l1x_dev_s *priv, uint16_t TargetDistInMm, uint16_t *xtalk);


/* Character driver methods */

static int vl53l1x_open(FAR struct file *filep);
static int vl53l1x_close(FAR struct file *filep);
static ssize_t vl53l1x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t vl53l1x_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static ssize_t vl53l1x_ioctl(FAR struct file *filep,int cmd,uint16_t arg);

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

 /****************************************************************************
  * Name: VL53L1X_SensorInit
  *
  * Description:
  *   This function loads the 135 bytes default values to initialize the sensor.
  *
  ****************************************************************************/

 static  void VL53L1X_SensorInit(struct vl53l1x_dev_s *priv)
 {

 	uint8_t Addr = 0x00, tmp=0;

 	for (Addr = 0x2D; Addr <= 0x87; Addr++){
 		vl53l1x_putreg8(priv, Addr,VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D]);
 	}
 	VL53L1X_StartRanging(priv);
 	while(tmp==0){
 			VL53L1X_CheckForDataReady(priv,&tmp);
 	}
 	tmp  = 0;
 	VL53L1X_ClearInterrupt(priv);
 	VL53L1X_StopRanging(priv);
 	vl53l1x_putreg8(priv, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,0x09); // two bounds VHV
 	vl53l1x_putreg8(priv, 0x0B,0); // start VHV from the previous temperature

}

/****************************************************************************
 * Name: VL53L1X_ClearInterrupt
 *
 * Description:
 *   This function clears the interrupt, to be called after a ranging data
 *   reading to arm the interrupt for the next data ready event.
 *
 ****************************************************************************/

static void VL53L1X_ClearInterrupt(struct vl53l1x_dev_s *priv)
{
	vl53l1x_putreg8(priv,  SYSTEM__INTERRUPT_CLEAR, 0x01);
}

/****************************************************************************
 * Name: VL53L1X_SetInterruptPolarity
 *
 * Description:
 *   This function programs the interrupt polarity.
 *
 ****************************************************************************/

static void VL53L1X_SetInterruptPolarity(struct vl53l1x_dev_s *priv,uint8_t NewPolarity)
{
	uint8_t Temp;

	Temp = vl53l1x_getreg8(priv, GPIO_HV_MUX__CTRL);
	Temp = Temp & 0xEF;
	vl53l1x_putreg8(priv,  GPIO_HV_MUX__CTRL, Temp | (!(NewPolarity & 1)) << 4);

}

/****************************************************************************
 * Name: VL53L1X_StartRanging
 *
 * Description:
 *   This function starts the ranging distance operation. The ranging operation
 *   is continuous. The clear interrupt has to be done after each get data to
 *   allow the interrupt to raise when the next data is ready.
 *
 ****************************************************************************/

static void VL53L1X_StartRanging(struct vl53l1x_dev_s *priv)
{

	vl53l1x_putreg8(priv,  SYSTEM__MODE_START, 0x40);	// Enable VL53L1X

}

/****************************************************************************
 * Name: VL53L1X_StopRanging
 *
 * Description:
 *   This function stops the ranging.
 *
 ****************************************************************************/

static void VL53L1X_StopRanging(struct vl53l1x_dev_s *priv)
{
	vl53l1x_putreg8(priv,  SYSTEM__MODE_START, 0x00);	// Disable VL53L1X
}

/****************************************************************************
 * Name: VL53L1X_CheckForDataReady
 *
 * Description:
 *   This function checks if the new ranging data is available by polling the
 *   dedicated register.
 *
 ****************************************************************************/

static void VL53L1X_CheckForDataReady(struct vl53l1x_dev_s *priv, uint8_t *isDataReady)
{
	uint8_t Temp;
	uint8_t IntPol;

	Temp = vl53l1x_getreg8(priv, GPIO__TIO_HV_STATUS);
	// Read in the register to check if a new value is available

	if ((Temp & 1) == IntPol)
  {
    *isDataReady = 1;
  }
	else
  {
    *isDataReady = 0;
  }

}

/****************************************************************************
 * Name: VL53L1X_SetTimingBudgetInMs
 *
 * Description:
 *   This function programs the timing budget in ms.
 *
 ****************************************************************************/

static void VL53L1X_SetTimingBudgetInMs(struct vl53l1x_dev_s *priv, uint16_t TimingBudgetInMs)
{
	uint16_t DM;
	VL53L1X_GetDistanceMode(priv,&DM);
	if (DM == 0)
		return 1;
	else if (DM == 1) {	// Short DistanceMode
		switch (TimingBudgetInMs) {
		case 15: // only available in short distance mode
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x01D);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x0027);
			break;
		case 20:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x0051);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x006E);
			break;
		case 33:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x00D6);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x006E);
			break;
		case 50:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x1AE);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x01E8);
			break;
		case 100:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x02E1);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x0388);
			break;
		case 200:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x03E1);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x0496);
			break;
		case 500:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x0591);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x05C1);
			break;
		default:

			break;
		}
	} else {
		switch (TimingBudgetInMs) {
		case 20:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x001E);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x0022);
			break;
		case 33:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x0060);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x006E);
			break;
		case 50:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x00AD);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x00C6);
			break;
		case 100:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x01CC);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x01EA);
			break;
		case 200:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x02D9);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x02F8);
			break;
		case 500:
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
					0x048F);
			vl53l1x_putreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
					0x04A4);
			break;
		default:

			break;
		}
	}

}

/****************************************************************************
 * Name: VL53L1X_GetTimingBudgetInMs
 *
 * Description:
 *   This function returns the timing budget in ms.
 *
 ****************************************************************************/

static void VL53L1X_GetTimingBudgetInMs(struct vl53l1x_dev_s *priv, uint16_t *pTimingBudget)
{
	uint16_t Temp;

	Temp = vl53l1x_getreg16(priv, RANGE_CONFIG__TIMEOUT_MACROP_A_HI);
	switch (Temp) {
		case 0x001D :
			*pTimingBudget = 15;
			break;
		case 0x0051 :
		case 0x001E :
			*pTimingBudget = 20;
			break;
		case 0x00D6 :
		case 0x0060 :
			*pTimingBudget = 33;
			break;
		case 0x1AE :
		case 0x00AD :
			*pTimingBudget = 50;
			break;
		case 0x02E1 :
		case 0x01CC :
			*pTimingBudget = 100;
			break;
		case 0x03E1 :
		case 0x02D9 :
			*pTimingBudget = 200;
			break;
		case 0x0591 :
		case 0x048F :
			*pTimingBudget = 500;
			break;
		default:
			*pTimingBudget = 0;
			break;
	}

}

/****************************************************************************
 * Name: VL53L1X_SetDistanceMode
 *
 * Description:
 *   This function programs the distance mode (1=short, 2=long(default)).
 *   Short mode max distance is limited to 1.3 m but better ambient immunity.
 *   Long mode can range up to 4 m in the dark with 200 ms timing budget.
 *
 ****************************************************************************/

static void VL53L1X_SetDistanceMode(struct vl53l1x_dev_s *priv, uint16_t DM)
{
	uint16_t TB;

	VL53L1X_GetTimingBudgetInMs(priv,&TB);
	switch (DM) {
	case 1:
		vl53l1x_putreg8(priv,  PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
		vl53l1x_putreg8(priv,  RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
		vl53l1x_putreg8(priv,  RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
		vl53l1x_putreg8(priv,  RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
		vl53l1x_putreg16(priv, SD_CONFIG__WOI_SD0, 0x0705);
		vl53l1x_putreg16(priv, SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
		break;
	case 2:
		vl53l1x_putreg8(priv,  PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
		vl53l1x_putreg8(priv,  RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
		vl53l1x_putreg8(priv,  RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
		vl53l1x_putreg8(priv,  RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
		vl53l1x_putreg16(priv, SD_CONFIG__WOI_SD0, 0x0F0D);
		vl53l1x_putreg16(priv, SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
		break;
	default:
		break;
	}
	VL53L1X_SetTimingBudgetInMs(priv,TB);

}

/****************************************************************************
 * Name: VL53L1X_GetDistanceMode
 *
 * Description:
 *   This function returns the current distance mode (1=short, 2=long).
 *
 ****************************************************************************/

static void VL53L1X_GetDistanceMode(struct vl53l1x_dev_s *priv, uint16_t *DM)
{
	uint8_t TempDM;

	TempDM = vl53l1x_getreg8(priv,PHASECAL_CONFIG__TIMEOUT_MACROP);
	if (TempDM == 0x14){
    *DM=1;
  }
	if(TempDM == 0x0A)
  {
    *DM=2;
  }
}

/****************************************************************************
 * Name: VL53L1X_SetInterMeasurementInMs
 *
 * Description:
 *   This function programs the Intermeasurement period in ms.
 *
 ****************************************************************************/

static void VL53L1X_SetInterMeasurementInMs(struct vl53l1x_dev_s *priv, uint16_t InterMeasMs)
{
	uint16_t ClockPLL;


	ClockPLL = vl53l1x_getreg16(priv, VL53L1_RESULT__OSC_CALIBRATE_VAL);
	ClockPLL = ClockPLL&0x3FF;
	vl53l1x_putreg32(priv, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
			(uint32_t)(ClockPLL * InterMeasMs * 1.075));


}

/****************************************************************************
 * Name: VL53L1X_GetInterMeasurementInMs
 *
 * Description:
 *   This function returns the Intermeasurement period in ms.
 *
 ****************************************************************************/

static void VL53L1X_GetInterMeasurementInMs(struct vl53l1x_dev_s *priv, uint16_t *pIM)
{
	uint16_t ClockPLL;

	uint32_t tmp;

	tmp = vl53l1x_getreg32(priv,VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD);
	*pIM = (uint16_t)tmp;
	vl53l1_getreg16(priv,  VL53L1_RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
	ClockPLL = ClockPLL&0x3FF;
	*pIM= (uint16_t)(*pIM/(ClockPLL*1.065));

}

/****************************************************************************
 * Name: VL53L1X_BootState
 *
 * Description:
 *  This function returns the boot state of the device (1:booted, 0:not booted)
 *
 ****************************************************************************/

static void VL53L1X_BootState(struct vl53l1x_dev_s *priv, uint8_t *state)
{
	uint8_t tmp = 0;

	tmp = vl53l1x_getreg8(priv, VL53L1_FIRMWARE__SYSTEM_STATUS);
	*state = tmp;

}

/****************************************************************************
 * Name: VL53L1X_GetSensorId
 *
 * Description:
 *  This function returns the sensor id, sensor Id must be 0xEEAC.
 *
 ****************************************************************************/

static void VL53L1X_GetSensorId(struct vl53l1x_dev_s *priv, uint16_t *sensorId)
{
	uint16_t tmp = 0;

	tmp = vl53l1x_getreg16(priv,  VL53L1_IDENTIFICATION__MODEL_ID);
	*sensorId = tmp;

}

/****************************************************************************
 * Name: VL53L1X_GetDistance
 *
 * Description:
 *  This function returns the distance measured by the sensor in mm.
 *
 ****************************************************************************/

static void VL53L1X_GetDistance(struct vl53l1x_dev_s *priv, uint16_t *distance)
{
	uint16_t tmp;

	tmp = (vl53l1x_getreg16(priv,
			VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0));
	*distance = tmp;

}
/****************************************************************************
 * Name: VL53L1X_GetSignalPerSpad
 *
 * Description:
 *  This function returns the returned signal per SPAD in kcps/SPAD.
 *  With kcps stands for Kilo Count Per Second.
 *
 ****************************************************************************/

static void VL53L1X_GetSignalPerSpad(struct vl53l1x_dev_s *priv, uint16_t *signalRate)
{

	uint16_t SpNb=1, signal;

	signal = vl53l1x_getreg16(priv,
		VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0);
	SpNb = vl53l1x_getreg16(priv,
		VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0);
	*signalRate = (uint16_t) (2000.0*signal/SpNb);

}

/****************************************************************************
 * Name: VL53L1X_GetAmbientPerSpad
 *
 * Description:
 *  This function returns the ambient per SPAD in kcps/SPAD.
 *
 ****************************************************************************/

static void VL53L1X_GetAmbientPerSpad(struct vl53l1x_dev_s *priv,  uint16_t *ambPerSp)
{
	uint16_t AmbientRate, SpNb=1;

	AmbientRate = vl53l1x_getreg16(priv,  RESULT__AMBIENT_COUNT_RATE_MCPS_SD);
	SpNb = vl53l1x_getreg16(priv,  VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0);
	*ambPerSp=(uint16_t) (2000.0 * AmbientRate / SpNb);

}

/****************************************************************************
 * Name: VL53L1X_GetSignalRate
 *
 * Description:
 *  This function returns the returned signal in kcps.
 *
 ****************************************************************************/

static void VL53L1X_GetSignalRate(struct vl53l1x_dev_s *priv,  uint16_t *signal)
{

	uint16_t tmp;

	tmp = vl53l1x_getreg16(priv,
		VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0);
	*signal = tmp*8;

}

/****************************************************************************
 * Name: VL53L1X_GetSpadNb
 *
 * Description:
 *  This function returns the current number of enabled SPADs.
 *
 ****************************************************************************/

static void VL53L1X_GetSpadNb(struct vl53l1x_dev_s *priv, uint16_t *spNb)
{

	uint16_t tmp;

	tmp = vl53l1x_getreg16(priv,
			      VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0);
	*spNb = tmp >> 8;

}

/****************************************************************************
 * Name: VL53L1X_GetAmbientRate
 *
 * Description:
 *  This function returns the ambient rate in kcps.
 *
 ****************************************************************************/

static void VL53L1X_GetAmbientRate(struct vl53l1x_dev_s *priv,  uint16_t *ambRate)
{
	uint16_t tmp;

	tmp = vl53l1x_getreg16(priv,  RESULT__AMBIENT_COUNT_RATE_MCPS_SD);
	*ambRate = tmp*8;

}

/****************************************************************************
 * Name: VL53L1X_GetRangeStatus
 *
 * Description:
 *  This function returns the ranging status error.
 *
 ****************************************************************************/

static void VL53L1X_GetRangeStatus(struct vl53l1x_dev_s *priv,  uint8_t *rangeStatus)
{
	uint8_t RgSt;

	RgSt = vl53l1x_getreg8(priv,  VL53L1_RESULT__RANGE_STATUS);
	RgSt = RgSt&0x1F;
	switch (RgSt) {
	case 9:
		RgSt = 0;
		break;
	case 6:
		RgSt = 1;
		break;
	case 4:
		RgSt = 2;
		break;
	case 8:
		RgSt = 3;
		break;
	case 5:
		RgSt = 4;
		break;
	case 3:
		RgSt = 5;
		break;
	case 19:
		RgSt = 6;
		break;
	case 7:
		RgSt = 7;
		break;
	case 12:
		RgSt = 9;
		break;
	case 18:
		RgSt = 10;
		break;
	case 22:
		RgSt = 11;
		break;
	case 23:
		RgSt = 12;
		break;
	case 13:
		RgSt = 13;
		break;
	default:
		RgSt = 255;
		break;
	}
	*rangeStatus = RgSt;

}

/****************************************************************************
 * Name: VL53L1X_SetOffset
 *
 * Description:
 *  This function programs the offset correction in mm.
 *
 ****************************************************************************/

static void VL53L1X_SetOffset(struct vl53l1x_dev_s *priv, int16_t OffsetValue)
{
	int16_t Temp;

	Temp = (OffsetValue*4);
	vl53l1x_putreg16(priv, ALGO__PART_TO_PART_RANGE_OFFSET_MM,
			(uint16_t)Temp);
	vl53l1x_putreg16(priv, MM_CONFIG__INNER_OFFSET_MM, 0x0);
	vl53l1x_putreg16(priv, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
}

/****************************************************************************
 * Name: VL53L1X_GetOffset
 *
 * Description:
 *  This function returns the programmed offset correction value in mm.
 *
 ****************************************************************************/

static void  VL53L1X_GetOffset(struct vl53l1x_dev_s *priv, int16_t *offset)
{
	uint16_t Temp;

	Temp = vl53l1x_getreg16(priv, ALGO__PART_TO_PART_RANGE_OFFSET_MM);
	Temp = Temp<<3;
	Temp = Temp >>5;
	*offset = (int16_t)(Temp);
}

/****************************************************************************
 * Name: VL53L1X_SetXtalk
 *
 * Description:
 *  This function programs the xtalk correction value in cps (Count Per Second).
 *  This is the number of photons reflected back from the cover glass in cps.
 *
 ****************************************************************************/

static void VL53L1X_SetXtalk(struct vl53l1x_dev_s *priv, uint16_t XtalkValue)
{
/* XTalkValue in count per second to avoid float type */

	 vl53l1x_putreg16(priv,
			ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
			0x0000);
	vl53l1x_putreg16(priv, ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
			0x0000);
	 vl53l1x_putreg16(priv, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
			(XtalkValue<<9)/1000); // * << 9 (7.9 format) and /1000 to convert cps to kpcs

}

/****************************************************************************
 * Name: VL53L1X_GetXtalk
 *
 * Description:
 *  This function returns the current programmed xtalk correction value in cps.
 *
 ****************************************************************************/

static void VL53L1X_GetXtalk(struct vl53l1x_dev_s *priv, uint16_t *xtalk )
{
	uint16_t tmp;

	tmp = vl53l1x_getreg16(priv, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS);
	*xtalk = (tmp*1000)>>9; // * 1000 to convert kcps to cps and >> 9 (7.9 format)
}

/****************************************************************************
 * Name: VL53L1X_SetDistanceThreshold
 *
 * Description:
 *  This function programs the threshold detection mode.
 *
 ****************************************************************************/

static void VL53L1X_SetDistanceThreshold(struct vl53l1x_dev_s *priv, uint16_t ThreshLow,
			      uint16_t ThreshHigh, uint8_t Window,
			      uint8_t IntOnNoTarget)
{

	uint8_t Temp = 0;

	Temp = vl53l1x_getreg8(priv,  SYSTEM__INTERRUPT_CONFIG_GPIO);
	Temp = Temp & 0x47;
	if (IntOnNoTarget == 0) {
		vl53l1x_putreg8(priv,  SYSTEM__INTERRUPT_CONFIG_GPIO,
			       (Temp | (Window & 0x07)));
	} else {
		vl53l1x_putreg8(priv,  SYSTEM__INTERRUPT_CONFIG_GPIO,
			       ((Temp | (Window & 0x07)) | 0x40));
	}
	vl53l1x_putreg16(priv, SYSTEM__THRESH_HIGH, ThreshHigh);
	vl53l1x_putreg16(priv, SYSTEM__THRESH_LOW, ThreshLow);

}

/****************************************************************************
 * Name: VL53L1X_GetDistanceThresholdWindow
 *
 * Description:
 *  This function returns the window detection mode (0=below; 1=above; 2=out; 3=in).
 *
 ****************************************************************************/

static void VL53L1X_GetDistanceThresholdWindow(struct vl53l1x_dev_s *priv, uint16_t *window)
{
	uint8_t tmp;
	tmp = vl53l1x_getreg8(priv, SYSTEM__INTERRUPT_CONFIG_GPIO);
	*window = (uint16_t)(tmp & 0x7);
}

/****************************************************************************
 * Name: VL53L1X_GetDistanceThresholdLow
 *
 * Description:
 *  This function returns the low threshold in mm.
 *
 ****************************************************************************/

static void VL53L1X_GetDistanceThresholdLow(struct vl53l1x_dev_s *priv, uint16_t *low)
{
	uint16_t tmp;

	tmp = vl53l1x_getreg16(priv, SYSTEM__THRESH_LOW);
	*low = tmp;
}

/****************************************************************************
 * Name: VL53L1X_GetDistanceThresholdHigh
 *
 * Description:
 *  This function returns the high threshold in mm.
 *
 ****************************************************************************/

static void VL53L1X_GetDistanceThresholdHigh(struct vl53l1x_dev_s *priv, uint16_t *high)
{
	uint16_t tmp;

	tmp = vl53l1x_getreg16(priv, SYSTEM__THRESH_HIGH);
	*high = tmp;
}

/****************************************************************************
 * Name: VL53L1X_SetROI
 *
 * Description:
 *  This function programs the ROI (Region of Interest).
 *
 ****************************************************************************/

static void VL53L1X_SetROI(struct vl53l1x_dev_s *priv, uint16_t X, uint16_t Y)
{
	uint8_t OpticalCenter;

	OpticalCenter =vl53l1x_getreg8(priv,  VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD);
	if (X > 16)
		X = 16;
	if (Y > 16)
		Y = 16;
	if (X > 10 || Y > 10){
		OpticalCenter = 199;
	}
	vl53l1x_putreg8(priv,  ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter);
	vl53l1x_putreg8(priv,  ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
		       (Y - 1) << 4 | (X - 1));
}

/****************************************************************************
 * Name: VL53L1X_GetROI_XY
 *
 * Description:
 *  This function returns width X and height Y.
 *
 ****************************************************************************/

static void VL53L1X_GetROI_XY(struct vl53l1x_dev_s *priv, uint16_t *ROI_X, uint16_t *ROI_Y)
{
	uint8_t tmp;

	tmp = vl53l1x_getreg8(priv, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE);
	*ROI_X = ((uint16_t)tmp & 0x0F) + 1;
	*ROI_Y = (((uint16_t)tmp & 0xF0) >> 4) + 1;
}

/****************************************************************************
 * Name: VL53L1X_SetSignalThreshold
 *
 * Description:
 *  This function programs a new signal threshold in kcps.
 *
 ****************************************************************************/

static void VL53L1X_SetSignalThreshold(struct vl53l1x_dev_s *priv, uint16_t Signal)
{
  vl53l1x_putreg16(priv,RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,Signal>>3);
}

/****************************************************************************
 * Name: VL53L1X_GetSignalThreshold
 *
 * Description:
 *  This function returns the current signal threshold in kcps.
 *
 ****************************************************************************/

static void VL53L1X_GetSignalThreshold(struct vl53l1x_dev_s *priv, uint16_t *signal)
{
	uint16_t tmp;

	tmp = vl53l1x_getreg16(priv,
				RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS);
	*signal = tmp <<3;
}

/****************************************************************************
 * Name: VL53L1X_SetSigmaThreshold
 *
 * Description:
 *  This function programs a new sigma threshold in mm (default=15 mm).
 *
 ****************************************************************************/

static void VL53L1X_SetSigmaThreshold(struct vl53l1x_dev_s *priv, uint16_t Sigma)
{
	if(Sigma>(0xFFFF>>2)){
		return 1;
	}
	// 16 bits register 14.2 format
	vl53l1x_putreg16(priv,RANGE_CONFIG__SIGMA_THRESH,Sigma<<2);
}

/****************************************************************************
 * Name: VL53L1X_GetSigmaThreshold
 *
 * Description:
 *  This function returns the current sigma threshold in mm.
 *
 ****************************************************************************/

static void VL53L1X_GetSigmaThreshold(struct vl53l1x_dev_s *priv, uint16_t *sigma)
{
	uint16_t tmp;

	tmp = vl53l1x_getreg16(priv, RANGE_CONFIG__SIGMA_THRESH);
	*sigma = tmp >> 2;
}

/****************************************************************************
 * Name: VL53L1X_StartTemperatureUpdate
 *
 * Description:
 *  This function performs the temperature calibration. It is recommended to
 *  call this function any time the temperature might have changed by more than
 *  8 deg C without sensor ranging activity for an extended period.
 *
 ****************************************************************************/

static void VL53L1X_StartTemperatureUpdate(struct vl53l1x_dev_s *priv)
{
	uint8_t tmp=0;

	vl53l1x_putreg8(priv, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,0x81); // full VHV
	vl53l1x_putreg8(priv, 0x0B,0x92);
	VL53L1X_StartRanging(priv);
	while(tmp==0){
		 VL53L1X_CheckForDataReady(priv, &tmp);
	}
	tmp  = 0;
	VL53L1X_ClearInterrupt(priv);
	VL53L1X_StopRanging(priv);
	vl53l1x_putreg8(priv,  VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); // two bounds VHV
	vl53l1x_putreg8(priv,  0x0B, 0); // start VHV from the previous temperature
}

/****************************************************************************
 * Name: VL53L1X_CalibrateOffset
 *
 * Description:
 *  The function returns the offset value found and programs the offset
 *  compensation into the device.
 *
 ****************************************************************************/

static void VL53L1X_CalibrateOffset(struct vl53l1x_dev_s *priv, uint16_t TargetDistInMm, int16_t *offset)
{
	uint8_t i = 0, tmp;
	int16_t AverageDistance = 0;
	uint16_t distance;

	vl53l1x_putreg16(priv, ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
	vl53l1x_putreg16(priv, MM_CONFIG__INNER_OFFSET_MM, 0x0);
	vl53l1x_putreg16(priv, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
	VL53L1X_StartRanging(priv);	// Enable VL53L1X sensor
	for (i = 0; i < 50; i++) {
		while (tmp == 0){
			VL53L1X_CheckForDataReady(priv,&tmp);
		}
		tmp = 0;
		VL53L1X_GetDistance(priv,&distance);
		VL53L1X_ClearInterrupt(priv);
		AverageDistance = AverageDistance + distance;
	}
	VL53L1X_StopRanging(priv);
	AverageDistance = AverageDistance / 50;
	*offset = TargetDistInMm - AverageDistance;
	vl53l1x_putreg16(priv, ALGO__PART_TO_PART_RANGE_OFFSET_MM, *offset*4);

}

/****************************************************************************
 * Name: VL53L1X_CalibrateXtalk
 *
 * Description:
 *  The function returns the xtalk value found and programs the xtalk
 *  compensation to the device.
 *
 ****************************************************************************/

static int8_t VL53L1X_CalibrateXtalk(struct vl53l1x_dev_s *priv, uint16_t TargetDistInMm, uint16_t *xtalk)
{
	uint8_t i, tmp= 0;
	float AverageSignalRate = 0;
	float AverageDistance = 0;
	float AverageSpadNb = 0;
	uint16_t distance = 0, spadNum;
	uint16_t sr;

	vl53l1x_putreg16(priv, 0x0016,0);
	VL53L1X_StartRanging(priv);
	for (i = 0; i < 50; i++) {
		while (tmp == 0){
			VL53L1X_CheckForDataReady(priv,&tmp);
		}
		tmp=0;
		VL53L1X_GetSignalRate(priv, &sr);
		VL53L1X_GetDistance(priv, &distance);
		VL53L1X_ClearInterrupt(priv);
		AverageDistance = AverageDistance + distance;
		VL53L1X_GetSpadNb(priv, &spadNum);
		AverageSpadNb = AverageSpadNb + spadNum;
		AverageSignalRate =
		    AverageSignalRate + sr;
	}
	VL53L1X_StopRanging(priv);
	AverageDistance = AverageDistance / 50;
	AverageSpadNb = AverageSpadNb / 50;
	AverageSignalRate = AverageSignalRate / 50;
	// Calculate Xtalk value
	*xtalk = (uint16_t)(512*(AverageSignalRate*(1-(AverageDistance/TargetDistInMm)))/AverageSpadNb);
	vl53l1x_putreg16(priv, 0x0016, *xtalk);

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
   sninfo("Reg %02x % \r\n",reg_addr_aux[0],reg_addr_aux[1]);
   ret = i2c_write(priv->i2c, &config,  &reg_addr_aux, 2);
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
* Name: vl53l1x_getreg32
*
* Description:
*   Read 4 8-bit from a VL53L1X register
*
****************************************************************************/

 static uint32_t vl53l1x_getreg32(FAR struct vl53l1x_dev_s *priv, uint16_t regaddr)
 {
   struct i2c_config_s config;
   uint16_t byte1, byte2, byte3, byte4;//byte1 is the msb and byte4 is the lsb
   uint32_t regval = 0;
   int ret;
   uint8_t reg_addr_aux[2];

   reg_addr_aux[0] = (regaddr >> 8) & 0xFF;
   reg_addr_aux[1] = regaddr;

   /* Set up the I2C configuration */

   config.frequency = priv->freq;
   config.address   = priv->addr;
   config.addrlen   = 7;

   /* Register to read */

   ret = i2c_write(priv->i2c, &config, &regaddr, 2);
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
  FAR uint16_t            *aux = (FAR uint16_t *) buffer;

  VL53L1X_StartRanging(priv);
  VL53L1X_GetDistance(priv, aux);
  VL53L1X_StopRanging(priv);

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

static ssize_t vl53l1x_ioctl(FAR struct file *filep,int cmd,uint16_t arg)
{
  FAR struct inode      *inode = filep->f_inode;
  FAR struct vl53l1x_dev_s *priv   = inode->i_private;

  switch (cmd)
  {
    case SNIOC_DISTANCESHORT:
    {
      sninfo("Set distance up to 1.3M\n");
        VL53L1X_SetDistanceMode(priv,1);
    }
    break;
    case SNIOC_DISTANCELONG:
    {
      sninfo("Set distance up to 4M\n");
      VL53L1X_SetDistanceMode(priv,2);
    }
    break;
    case SNIOC_CALIBRATE:
    {
      sninfo("Calibrating distance\n");
      int16_t offset;
      VL53L1X_GetOffset(priv, offset);
      VL53L1X_CalibrateOffset(priv, arg, offset);
    }
    break;
    case SNIOC_TEMPUPDATE:
    {
      sninfo("Recalculating due to temperature change\n");
      VL53L1X_StartTemperatureUpdate(priv);
    }
    break;
  }
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
  uint16_t sensorId;

  /* Initialize the vl53l1x device structure */

  priv = (FAR struct vl53l1x_dev_s *)kmm_malloc(sizeof(struct vl53l1x_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = 0x29;
  priv->freq = VLM53L1X_FREQ;

  VL53L1X_SensorInit(priv);

  VL53L1X_GetSensorId(priv,&sensorId);
  if(sensorId != 0xEACC){
    snerr("ERROR: Failed sensor ID %04x\n", sensorId);
    kmm_free(priv);
    return 0;
  }



  register_driver(devpath, &g_vl53l1xfops, 0666, priv);
  if (ret < 0)
  {
    snerr("ERROR: Failed to register driver: %d\n", ret);
    kmm_free(priv);
    return 0;
  }

sninfo("VL53L1X driver loaded successfully!\n");
return 1;

}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BMP180 */
