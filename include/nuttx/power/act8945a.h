/****************************************************************************
 * include/nuttx/power/act8945a.h
 * msa301 Driver declaration
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

#ifndef __INCLUDE_NUTTX_POWER_BATTERY_ACT8945A_H
#define __INCLUDE_NUTTX_POWER_BATTERY_ACT8945A_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define ACT8945A_BUS_SPEED                        380000
#define ACT8945A_SLAVE_ADDRESS                    (0x5B)

#define ACT8945A_SYS0                             (0x00)
#define ACT8945A_SYS1                             (0x01)

#define ACT8945A_REG1_VSEL_LO                     (0x20)
#define ACT8945A_REG1_VSEL_HI                     (0x21)
#define ACT8945A_REG1_CONTROL                     (0x22)
#define ACT8945A_REG2_VSEL_LO                     (0x30)
#define ACT8945A_REG2_VSEL_HI                     (0x31)
#define ACT8945A_REG2_CONTROL                     (0x32)
#define ACT8945A_REG3_VSEL_LO                     (0x40)
#define ACT8945A_REG3_VSEL_HI                     (0x41)
#define ACT8945A_REG3_CONTROL                     (0x42)
#define ACT8945A_REG4_VSEL                        (0x50)
#define ACT8945A_REG4_CONTROL                     (0x51)
#define ACT8945A_REG5_VSEL                        (0x54)
#define ACT8945A_REG5_CONTROL                     (0x55)
#define ACT8945A_REG6_VSEL                        (0x60)
#define ACT8945A_REG6_CONTROL                     (0x61)
#define ACT8945A_REG7_VSEL                        (0x64)
#define ACT8945A_REG7_CONTROL                     (0x65)

#define ACT8945A_VSET_MASK                        (0x3f)

#define ACT8945A_LOW_IQ_ENABLE                    (1 << 5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum act8945a_interrupt
{
	CHARGE_STATE_OUT_EOC_STATE,
	INPUT_VOLTAGE_OUT_VALID_RANGE,
	BATTERY_TEMPERATURE_OUT_RANGE,
	PRECHARGE_TIME_OUT,
	CHARGE_STATE_INTO_EOC_STATE,
	INPUT_VOLTAGE_INTO_VALID_RANGE,
	BATTERY_TEMPERATURE_INTO_RANGE,
	TOTAL_CHARGE_TIME_OUT,
};
  


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/


#if defined(CONFIG_I2C) && defined(CONFIG_I2C_ACT8945A)

#ifdef __cplusplus
extern "C"
{
#endif

struct i2c_master_s;
FAR struct battery_charger_dev_s *act8945a_initialize(
                                     FAR struct i2c_master_s *i2c,
                                     uint8_t addr,
                                     uint32_t frequency
                                     );

/****************************************************************************
 * Name: act8945a_register
 *
 * Description:
 *   Register a lower half act8945a driver with the common, upper-half
 *   battery driver.
 *
 * Input Parameters:
 *   devpath - The location in the pseudo-filesystem to create the driver.
 *     Recommended standard is "/dev/bat0", "/dev/bat1", etc.
 *   dev - An instance of the battery state structure .
 *
 * Returned Value:
 *    Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int act8945a_register(FAR const char *devpath,
                      FAR struct battery_charger_dev_s *dev);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_I2C_ACT8945A */
#endif

