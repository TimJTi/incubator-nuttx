/****************************************************************************
 * arch/arm/src/sama5/sam_adc.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_ADC_H
#define __ARCH_ARM_SRC_SAMA5_SAM_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/analog/adc.h>

#include "hardware/sam_adc.h"
#include <nuttx/wqueue.h>

#ifdef CONFIG_SAMA5_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_SAMA5_ADC_REGDEBUG
#endif

/* ADC channels 0-3 or 0-4 are not available to the ADC driver if touchscreen
 * support is enabled.
 */
#ifdef CONFIG_SAMA5_TSD
#  undef CONFIG_SAMA5_ADC_CHAN0
#  undef CONFIG_SAMA5_ADC_CHAN1
#  undef CONFIG_SAMA5_ADC_CHAN2
#  undef CONFIG_SAMA5_ADC_CHAN3
#  ifdef CONFIG_SAMA5_TSD_5WIRE
#    undef CONFIG_SAMA5_ADC_CHAN4
#  endif
#endif

/* Do we have any ADC channels enabled?  If not, then the ADC driver may
 * still need to exist to support the touchscreen.
 */

#undef SAMA5_ADC_HAVE_CHANNELS
#if defined(CONFIG_SAMA5_ADC_CHAN0) || defined(CONFIG_SAMA5_ADC_CHAN1) || \
    defined(CONFIG_SAMA5_ADC_CHAN2) || defined(CONFIG_SAMA5_ADC_CHAN3) || \
    defined(CONFIG_SAMA5_ADC_CHAN4) || defined(CONFIG_SAMA5_ADC_CHAN5) || \
    defined(CONFIG_SAMA5_ADC_CHAN6) || defined(CONFIG_SAMA5_ADC_CHAN7) || \
    defined(CONFIG_SAMA5_ADC_CHAN8) || defined(CONFIG_SAMA5_ADC_CHAN9) || \
    defined(CONFIG_SAMA5_ADC_CHAN10) || defined(CONFIG_SAMA5_ADC_CHAN11)
#  define SAMA5_ADC_HAVE_CHANNELS 1
#elif !defined(CONFIG_SAMA5_TSD)
#  error "No ADC channels nor touchscreen"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adcinitialize
 *
 * Description:
 *   Initialize the ADC
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *sam_adc_initialize(void);
void sam_adc_shutdown(struct adc_dev_s *dev);

/****************************************************************************
 * Interfaces exported from the ADC to the touchscreen driver
 ****************************************************************************/

/****************************************************************************
 * Name: sam_adc_lock
 *
 * Description:
 *   Get exclusive access to the ADC interface
 *
 ****************************************************************************/

//struct sam_adc_s;
struct sam_adc_s
{
  const struct adc_callback_s *cb;
  sem_t exclsem;         /* Supports exclusive access to the ADC interface */
  bool initialized;      /* The ADC driver is already initialized */
  int nopen;             /* number times this ADC peripheral's been opened*/
  uint32_t frequency;    /* ADC clock frequency */

#ifdef SAMA5_ADC_HAVE_CHANNELS
#ifdef CONFIG_SAMA5_ADC_DMA
  volatile bool odd;     /* Odd buffer is in use */
  volatile bool ready;   /* Worker has completed the last set of samples */
  volatile bool enabled; /* DMA data transfer is enabled */
#endif
  struct adc_dev_s *dev; /* A reference to the outer, ADC device container */
  uint32_t pending;      /* Pending EOC events */
  struct work_s work;    /* Supports the interrupt handling "bottom half" */
#ifdef CONFIG_SAMA5_ADC_DMA
  DMA_HANDLE dma;        /* Handle for DMA channel */
#endif
#ifdef CONFIG_SAMA5_ADC_TIOATRIG
  TC_HANDLE tc;          /* Handle for the timer channel */
#endif

  /* DMA sample data buffer */

#ifdef CONFIG_SAMA5_ADC_DMA
  uint16_t evenbuf[SAMA5_ADC_SAMPLES];
  uint16_t oddbuf[SAMA5_ADC_SAMPLES];
#endif
#endif /* SAMA5_ADC_HAVE_CHANNELS */

  /* Debug stuff */

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
  bool wrlast;          /* Last was a write */
  uintptr_t addrlast;   /* Last address */
  uint32_t vallast;     /* Last value */
  int ntimes;           /* Number of times */
#endif
};

int sam_adc_lock(struct sam_adc_s *priv);

//void sam_cref_inc(struct sam_adc_s *priv);
//void sam_cref_dec(struct sam_adc_s *priv);

/****************************************************************************
 * Name: sam_adc_unlock
 *
 * Description:
 *   Relinquish the lock on the ADC interface
 *
 ****************************************************************************/

void sam_adc_unlock(struct sam_adc_s *priv);

/****************************************************************************
 * Name: sam_adc_getreg
 *
 * Description:
 *   Read any 32-bit register using an absolute address.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
uint32_t sam_adc_getreg(struct sam_adc_s *priv, uintptr_t address);
#else
#  define sam_adc_getreg(handle,addr) getreg32(addr)
#endif

/****************************************************************************
 * Name: sam_adc_putreg
 *
 * Description:
 *   Write to any 32-bit register using an absolute address.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_ADC_REGDEBUG
void sam_adc_putreg(struct sam_adc_s *priv, uintptr_t address,
                    uint32_t regval);
#else
#  define sam_adc_putreg(handle,addr,val) putreg32(val,addr)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SAMA5_ADC */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_ADC_H */
