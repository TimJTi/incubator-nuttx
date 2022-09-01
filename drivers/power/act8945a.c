/****************************************************************************
 * drivers/power/act8945a.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/act8945a.h>

#if defined(CONFIG_I2C) && defined(CONFIG_I2C_ACT8945A)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#if (CONFIG_ACT8945A_VSEL == 0)
#  define ACT8945A_REG1_VSEL ACT8945A_REG1_VSEL_LO
#  define ACT8945A_REG2_VSEL ACT8945A_REG2_VSEL_LO
#  define ACT8945A_REG3_VSEL ACT8945A_REG2_VSEL_LO
#else
#  define ACT8945A_REG1_VSEL ACT8945A_REG1_VSEL_HI
#  define ACT8945A_REG2_VSEL ACT8945A_REG2_VSEL_HI
#  define ACT8945A_REG3_VSEL ACT8945A_REG2_VSEL_HI
#endif

#ifdef CONFIG_DEBUG_POWER
#  define act8945a_err(x, ...)        _err(x, ##__VA_ARGS__)
#  define act8945a_warn(x, ...)       _warn(x, ##__VA_ARGS__)
#  define act8945a_info(x, ...)       _info(x, ##__VA_ARGS__)
#else
#  define act8945a_err(x, ...)        uerr(x, ##__VA_ARGS__)
#  define act8945a_warn(x, ...)       uwarn(x, ##__VA_ARGS__)
#  define act8945a_info(x, ...)       uinfo(x, ##__VA_ARGS__)
#endif

/****************************************************************************
 * Private type
 ****************************************************************************/

struct act8945a_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  FAR const struct battery_charger_operations_s *ops; /* Battery operations */
  sem_t batsem;                                       /* Enforce mutually exclusive access */

  /* Data fields specific to the lower half act8945a driver follow */

  FAR struct i2c_master_s *i2c;       /* I2C interface */
  uint8_t                  addr;      /* I2C address */
  uint32_t                 frequency; /* I2C frequency */
};

enum act8945a_regulator
{
  REG1 = 0,
  REG2,
  REG3,
  REG4,
  REG5,
  REG6,
  REG7,
};



/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C support */

static int act8945a_getreg(FAR struct act8945a_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval);
static int act8945a_putreg(FAR struct act8945a_dev_s *priv, uint8_t regaddr,
                          uint8_t regval);

/* Battery driver lower half methods */

static int act8945a_enable_reg(FAR struct act8945a_dev_s *priv,
                                    enum act8945a_regulator, bool enable);
static int act8945a_set_reg_voltage(FAR struct act8945a_dev_s *priv,
                                    enum act8945a_regulator, int voltage);
                                 
static int act8945a_status(FAR struct act8945a_dev_s *priv,
                           FAR int *status);
/****************************************************************************
 * Private Data
 ****************************************************************************/
static const uint8_t reg_volt_register[] = {ACT8945A_REG1_VSEL, ACT8945A_REG2_VSEL,
                                     ACT8945A_REG3_VSEL, ACT8945A_REG4_VSEL,
                                     ACT8945A_REG5_VSEL, ACT8945A_REG6_VSEL,
                                     ACT8945A_REG7_VSEL};
static const uint8_t reg_enable_register[] = {ACT8945A_REG1_CONTROL, ACT8945A_REG2_CONTROL,
                                     ACT8945A_REG3_CONTROL, ACT8945A_REG4_CONTROL,
                                     ACT8945A_REG5_CONTROL, ACT8945A_REG6_CONTROL,
                                     ACT8945A_REG7_CONTROL};
                                     

static const struct battery_charger_operations_s g_act8945aops =
{
  act8945a_status,

};


/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: act8945a_getreg
 *
 * Description:
 *   Reads a single regiaster from the ACT8945A PMIC.
 *
 ****************************************************************************/
static int act8945a_getreg(FAR struct act8945a_dev_s *priv, uint8_t regaddr,
                          FAR uint8_t *regval)
{
  struct i2c_config_s config;
  int                 ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      act8945a_err("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8 bits from the register */

  ret = i2c_read(priv->i2c, &config, regval, sizeof(*regval));
  if (ret < 0)
    {
      act8945a_err("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  act8945a_info("addr: %02x value: %02x\n", regaddr, *regval);

  return OK;
}

/****************************************************************************
 * Name: act8945a_putreg
 *
 * Description:
 *   Writes a single byte to one of the ACT8945A  registers.
 *
 ****************************************************************************/
static int act8945a_putreg(FAR struct act8945a_dev_s *priv, uint8_t regaddr,
                          uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t             buffer[2];
  int                 ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Set up a 2-byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Set up the I2C configuration */

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */


  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      act8945a_err("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  act8945a_info("INFO:addr: %02x value: %02x\n", regaddr, regval);

  return OK;
}

static int act8945a_status(FAR struct act8945a_dev_s *priv,
                           FAR int *status)
{
  *status = BATTERY_UNKNOWN;
  return OK;
}

/****************************************************************************
 * Name: act8945a_enable_reg
 *
 * Description:
 *   Enable/disable the regulator as required
 *
 ****************************************************************************/
static int act8945a_enable_reg(FAR struct act8945a_dev_s *priv,
                                    enum act8945a_regulator regulator, bool enable)
{
  int ret;
  uint8_t regval = 0;
  
  ret = act8945a_getreg(priv, reg_enable_register[regulator], &regval);

  if (enable)
    {
      regval &= ~0x80;
      act8945a_info("INFO: Setting regulator %d enable bit to %x\n", (regulator + 1), regval);
      regval |= 0x80;
      act8945a_info("INFO: Setting regulator %d enable bit to %x\n", (regulator + 1), regval);
    }
  else
    {
      regval |= 0x80;
      act8945a_info("INFO: Setting regulator %d enable bit to %x\n", (regulator + 1), regval);
      regval &= ~0x80;
      act8945a_info("INFO: Setting regulator %d enable bit to %x\n", (regulator + 1), regval);
    }
  return ret;
  
}
                                      

/****************************************************************************
 * Name: act8945a_set_reg_voltage
 *
 * Description:
 *   Set the regulator output voltage as required
 *
 ****************************************************************************/

static int act8945a_set_reg_voltage(FAR struct act8945a_dev_s *priv,
                                    enum act8945a_regulator regulator, 
                                    int voltreqd)
{
  uint8_t volts = 0;
  int ret = OK;
  
  if (voltreqd < 600)
  {
    act8945a_warn("WARN: Cannot set regulator %d to %dmV, 600mV used\n", (regulator+1), voltreqd);
    voltreqd = 600;
  }
  if (voltreqd >3900)
  {
    act8945a_warn("WARN: Cannot set regulator %d to %dmV, 3900mV used\n", (regulator+1), voltreqd);
    voltreqd = 3900;
  }
  
  if (voltreqd < 1200)
  {
    volts = (voltreqd - 600) / 25;
  }
	else if (voltreqd < 2400)
  {
		volts = 0x18 + (voltreqd - 1200) / 50;
	}
  else if (voltreqd <= 3900)
  {
		volts = 0x30 + (voltreqd - 2400) / 100;
	}
  
  act8945a_info("INFO: Setting regulator %d voltage registor to %x\n", (regulator + 1), volts);
  ret = act8945a_putreg(priv, reg_volt_register[regulator], volts);

  return ret;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct battery_charger_dev_s *
act8945a_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                  uint32_t frequency)
{
  FAR struct act8945a_dev_s *priv;
  uint8_t                  chipid = 0;

  /* Initialize the act8945a device structure */

  priv = (FAR struct act8945a_dev_s *)kmm_zalloc(sizeof(struct act8945a_dev_s));

  if (priv)
    {
      /* Initialize the act8945a device structure */

      priv->ops       = &g_act8945aops;
      priv->i2c       = i2c;
      priv->addr      = addr;
      priv->frequency = frequency;
    }

  /*
  act8945a_getreg(priv, ACT8945A_REG2, &chipid);

  if (ACT8945A_CHIP_ID != chipid)
    {
      kmm_free(priv);
      return NULL;
    }
  */
  /* Initialise regulator enables */
  //act8945a_enable_reg(priv, REG1, CONFIG_ACT8945A_REG1_ENABLE);
  //act8945a_enable_reg(priv, REG2, CONFIG_ACT8945A_REG2_ENABLE);
  //act8945a_enable_reg(priv, REG3, CONFIG_ACT8945A_REG3_ENABLE);
  //act8945a_enable_reg(priv, REG4, CONFIG_ACT8945A_REG4_ENABLE);
  //act8945a_enable_reg(priv, REG5, CONFIG_ACT8945A_REG5_ENABLE);
  act8945a_enable_reg(priv, REG6, 1);//CONFIG_ACT8945A_REG6_ENABLE);
  //act8945a_enable_reg(priv, REG7, 0);//CONFIG_ACT8945A_REG7_ENABLE);
  
  /* Initialise output voltages */

  //act8945a_set_reg_voltage(priv, REG1, CONFIG_ACT8945A_REG1_VOLTAGE);
  //act8945a_set_reg_voltage(priv, REG2, CONFIG_ACT8945A_REG2_VOLTAGE);
  //act8945a_set_reg_voltage(priv, REG3, CONFIG_ACT8945A_REG3_VOLTAGE);
  //act8945a_set_reg_voltage(priv, REG4, CONFIG_ACT8945A_REG4_VOLTAGE);
  //act8945a_set_reg_voltage(priv, REG5, CONFIG_ACT8945A_REG5_VOLTAGE);
  act8945a_set_reg_voltage(priv, REG6, CONFIG_ACT8945A_REG6_VOLTAGE);
  //act8945a_set_reg_voltage(priv, REG7, CONFIG_ACT8945A_REG7_VOLTAGE);

  return (FAR struct battery_charger_dev_s *)priv;
}

#endif /* CONFIG_I2C && CONFIG_I2C_ACT8945A */
