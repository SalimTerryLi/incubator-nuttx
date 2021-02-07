/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_bringup.c
 *
 *   Copyright (C) 2019 Bill Gatliff. All rights reserved.
 *   Copyright (C) 2012, 2014-2018 Gregory Nutt. All rights reserved.
 *   Author: Bill Gatliff <bgat@billgatliff.com>
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

#include <sys/mount.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#include "stm32.h"
#include "stm32_romfs.h"

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#include <nuttx/input/buttons.h>
#include "vrgate.h"

/* Conditional logic in omnibusf4.h will determine if certain features
 * are supported.  Tests for these features need to be made after including
 * omnibusf4.h.
 */

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2c_register(int bus)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          stm32_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2ctool(void)
{
  stm32_i2c_register(2);
#if 0
  stm32_i2c_register(1);
  stm32_i2c_register(2);
#endif
}
#else
#  define stm32_i2ctool()
#endif

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
	// bring up TC358870XBG
	stm32_configgpio(TC358870XBG_RST);
	stm32_gpiowrite(TC358870XBG_RST, false);
	usleep(1000);
	stm32_gpiowrite(TC358870XBG_RST, true);

	stm32_configgpio(BOARD_LED1_DIS);

  int ret = OK;

  ret=stm32_button_exti_setup();
  if(ret<0){
	  syslog(LOG_ERR, "ERROR: stm32_button_exti_setup() failed: %d\n", ret);
  }

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
  stm32_i2ctool();
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      uerr("ERROR: Failed to start USB monitor: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_PWM
  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n",
           STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_STM32_ROMFS
  /* Initialize and mount ROMFS. */

  ret = stm32_romfs_initialize();
  if (ret < 0)
    {
      serr("ERROR: Failed to mount romfs at %s: %d\n",
           CONFIG_STM32_ROMFS_MOUNTPOINT, ret);
    }
#endif

#if defined(CONFIG_RNDIS)
  /* Set up a MAC address for the RNDIS device. */

  uint8_t mac[6];
  mac[0] = 0xa0; /* TODO */
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

  return ret;
}
