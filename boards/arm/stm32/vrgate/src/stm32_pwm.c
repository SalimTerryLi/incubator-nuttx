/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_pwm.c
 *
 *   Copyright (C) 2019 Bill Gatliff. All rights reserved.
 *   Copyright (C) 2011, 2016 Gregory Nutt. All rights reserved.
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

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <limits.h>

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "stm32_pwm.h"
#include "vrgate.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *
 *   Initialize PWM and register VRGATE's TIM2 and TIM3 PWM devices:
 *
 *    TIM3 CH1
 *
 ****************************************************************************/

int stm32_pwm_setup(void)
{
  int npwm = 3;                       /* hardware device enumerator */
  const char *ppwm = "/dev/pwm0";            /* pointer to PWM device name */
  struct pwm_lowerhalf_s *pwm = NULL; /* lower-half driver handle */

  pwm = stm32_pwminitialize(npwm);

  /* If we can't get the lower-half handle, skip and keep going. */

  if (pwm)
  {
  	pwm_register(ppwm, pwm);
  }else{
  	syslog(LOG_ERR, "ERROR: stm32_pwminitialize(%d) failed\n", npwm);
  }

  return 0;
}
