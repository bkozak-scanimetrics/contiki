/*
 * Copyright (c) 2016, Scanimetrics - http://www.scanimetrics.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef SOC_WDT_H_
#define SOC_WDT_H_
/*---------------------------------------------------------------------------*/
#include "hw_sysctl.h"
#include "hw_prcm.h"
#include <stdint.h>
/*---------------------------------------------------------------------------*/
/* These macros are only valid when the SCLK_HF is acting as MCU clock and
   the cpu is in run mode or any other mode where the infrastrucutre clock
   divider is identical to that of run mode. */


/* Note in the TRM Rev. D, it says that PRCM_O_INFRCLKDIVR is overriden if it's
   value is '1'. This seems to be a mistake, the divider is actually overriden
   (to 2) if the value is '0'. */
#define SOC_WDT_CLK_DIV() \
  (32 << ( \
     HWREG(PRCM_BASE + PRCM_O_INFRCLKDIVR) ? \
     HWREG(PRCM_BASE + PRCM_O_INFRCLKDIVR) : 1 \
     ) \
  )
#define SOC_WDT_SECONDS(s) ((s) * (GET_MCU_CLOCK / SOC_WDT_CLK_DIV()))

#define SOC_WDT_MILLISECONDS(m) \
  (uint32_t)(((uint64_t)(m) * SOC_WDT_SECONDS(1)) / 1000)
#define SOC_WDT_CLK_TO_MILLISECONDS(n) \
  (uint32_t)(((uint64_t)(n) * 1000) / SOC_WDT_SECONDS(1))
/*---------------------------------------------------------------------------*/
#endif /* SOC_WDDT_H_ */