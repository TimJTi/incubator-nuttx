/****************************************************************************
 * boards/arm/sama5/sama5d2-xult/src/sam_at25.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>

#include "sam_spi.h"
#include "jti-toucan2.h"

#ifdef HAVE_AT25
# include <nuttx/eeprom/spi_xx25xx.h>
# include <fcntl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_at25_automount
 *
 * Description:
 *   Initialize and configure the AT25 serial FLASH
 *
 ****************************************************************************/

int sam_at25_automount(int minor)
{
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* No.. Get the SPI port driver */

      spi = sam_spibus_initialize(AT25_PORT);
      if (!spi)
        {
          ferr("ERROR: Failed to initialize SPI port %d\n", AT25_PORT);
          return -ENODEV;
        }

      /* Now bind the SPI interface to the AT25 SPI FLASH driver */
      ret = ee25xx_initialize(spi, "/dev/at25", EEPROM_25XX128, O_RDWR);      

      //mtd = at25_initialize(spi);
      //if (!mtd)
      if (ret < 0)
        {
          ferr("ERROR: Failed to bind SPI port %d to AT25 FLASH driver\n");
          return -ENODEV;
        }
      else
        {
          syslog(LOG_INFO, "Successfully initialised the AT25 driver\n");
        }

      /* Now we are initializeed */

      initialized = true;
    }
    else
      {
        syslog(LOG_INFO, "AT25 already initialised\n");
      }

  return OK;
}

#endif /* HAVE_AT25 */
