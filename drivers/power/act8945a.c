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
#include <string.h>
#include <debug.h>
#include <poll.h>
#include <fcntl.h>
#include <assert.h>
#include <nuttx/fs/fs.h>
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
#  define ACT8945A_REG3_VSEL ACT8945A_REG3_VSEL_LO
#else
#  define ACT8945A_REG1_VSEL ACT8945A_REG1_VSEL_HI
#  define ACT8945A_REG2_VSEL ACT8945A_REG2_VSEL_HI
#  define ACT8945A_REG3_VSEL ACT8945A_REG3_VSEL_HI
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

#define CONV_TO_MV(n)                       \
  ({                                        \
      int mv;                               \
      if ((n) < 12)                         \
        {                                   \
          mv = (((n) * 25) + 600);          \
        }                                   \
      else if ((n) < 48)                    \
        {                                   \
          mv = ((((n) -24) * 50) + 1200);   \
        }                                   \
      else if ((n) <= 63)                   \
        {                                   \
          mv = ((((n) - 48) * 100) + 2400); \
        }                                   \
      else                                  \
      {                                     \
        mv = -1;                             \
      }                                     \
      mv;                                   \
  })                                        \

#define CONV_FROM_MV(n)                     \
  ({                                        \
      uint8_t val;                          \
      if ((n) < 1200)                       \
        {                                   \
          val = (((n) - 600 ) / 25);        \
        }                                   \
      else if ((n) < 2400)                  \
        {                                   \
          val = ((((n) - 1200) / 50) + 24); \
        }                                   \
      else if ((n) <= 3900)                 \
        {                                   \
          val = ((((n) - 2400) / 100) + 48);\
        }                                   \
      else                                  \
      {                                     \
        val = -1;                           \
      }                                     \
      val;                                  \
  })
/****************************************************************************
 * Private type
 ****************************************************************************/
struct act8945a_priv_s
{
  struct list_node  node;
  sem_t             lock;
  sem_t             wait;
  uint32_t          mask;
  FAR struct pollfd *fds;
};

struct act8945a_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */
  /* Fields required by the upper-half driver */

  FAR const struct battery_charger_operations_s *ops; /* Battery operations */

  sem_t batsem;  /* Enforce mutually exclusive access */

  struct list_node flist;

  //FAR const struct act8945a_operations_s *ops; /* Battery operations */

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
/* Character driver methods */

static int     act8945a_open(FAR struct file *filep);
static int     act8945a_close(FAR struct file *filep);
static ssize_t act8945a_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
static ssize_t act8945a_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen);
static int     act8945a_ioctl(FAR struct file *filep, int cmd,
                                 unsigned long arg);
static int     act8945a_poll(FAR struct file *filep,
                                FAR struct pollfd *fds, bool setup);

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
static int act8945a_enable_lowiq(FAR struct act8945a_dev_s *priv,
                                    enum act8945a_regulator, bool enable);
                                 
static int act8945a_status(FAR struct act8945a_dev_s *priv,
                           FAR int *status);

static const struct battery_charger_operations_s g_act8945a_ops = 
{
  act8945a_status,
};

/****************************************************************************
 * Things to do
 * ------------
 *
 * All the fileops stuff
 *
 * Allow Vset reg to be set. Dangerous?
 * Phase, mode, delay. Dangerous?
 * regs 4/5/6/7
 * - output discharge control
 * - Low-IQ mode
 
 * System voltage monitor 
 * - programmable SYSLEV level
 * - mode: interrupt source or shutdown
 * - mode - 
 * Interrupts
 * fault mask stuff
 * - pin config and driver side of things (example app?
 * - determine what interrupts are enabled (default and ioctl)
 *   - syslev
 *   - 
 * Charger functionality
 * - OVP charge threshold config
****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_batteryops =
{
  act8945a_open,    /* open */
  act8945a_close,   /* close */
  act8945a_read,    /* read */
  act8945a_write,   /* write */
  NULL,                /* seek */
  act8945a_ioctl,   /* ioctl */
  act8945a_poll     /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL               /* unlink */

};
#endif
static const uint8_t reg_volt_register[] =
                    {ACT8945A_REG1_VSEL, ACT8945A_REG2_VSEL,
                     ACT8945A_REG3_VSEL, ACT8945A_REG4_VSEL,
                     ACT8945A_REG5_VSEL, ACT8945A_REG6_VSEL,
                     ACT8945A_REG7_VSEL
};
static const uint8_t reg_enable_register[] =
                    {ACT8945A_REG1_CONTROL, ACT8945A_REG2_CONTROL,
                     ACT8945A_REG3_CONTROL, ACT8945A_REG4_CONTROL,
                     ACT8945A_REG5_CONTROL, ACT8945A_REG6_CONTROL,
                     ACT8945A_REG7_CONTROL
};
static const uint8_t reg_control_register[] =
                    {ACT8945A_REG1_CONTROL, ACT8945A_REG2_CONTROL,
                     ACT8945A_REG3_CONTROL, ACT8945A_REG4_CONTROL,
                     ACT8945A_REG5_CONTROL, ACT8945A_REG6_CONTROL,
                     ACT8945A_REG7_CONTROL
                    };
#if 0
static const struct act8945a_operations_s g_act8945aops =
{
  act8945a_status,
};
#endif


/****************************************************************************
 * Private Functions
 ****************************************************************************/

 static int act8945a_notify(FAR struct act8945a_priv_s *priv,
                                  uint32_t mask)
{
  FAR struct pollfd *fd = priv->fds;
  int semcnt;
  int ret;

  if (!fd)
    {
      return OK;
    }

  ret = nxsem_wait_uninterruptible(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  priv->mask |= mask;
  if (priv->mask)
    {
      fd->revents |= POLLIN;
      nxsem_get_value(fd->sem, &semcnt);
      if (semcnt < 1)
        {
          nxsem_post(fd->sem);
        }

      nxsem_get_value(&priv->wait, &semcnt);
      if (semcnt < 1)
        {
          nxsem_post(&priv->wait);
        }
    }

  nxsem_post(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: act8945a_open
 *
 * Description:
 *   This function is called whenever the battery device is opened.
 *
 ****************************************************************************/

static int act8945a_open(FAR struct file *filep)
{
  FAR struct act8945a_priv_s *priv;
  FAR struct act8945a_dev_s *dev = filep->f_inode->i_private;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  ret = nxsem_wait(&dev->batsem);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  nxsem_init(&priv->lock, 0, 1);
  nxsem_init(&priv->wait, 0, 0);
  nxsem_set_protocol(&priv->wait, SEM_PRIO_NONE);
  list_add_tail(&dev->flist, &priv->node);
  nxsem_post(&dev->batsem);
  filep->f_priv = priv;

  return ret;
}

/****************************************************************************
 * Name: act8945a_close
 *
 * Description:
 *   This routine is called when the battery device is closed.
 *
 ****************************************************************************/

static int act8945a_close(FAR struct file *filep)
{
  FAR struct act8945a_priv_s *priv = filep->f_priv;
  FAR struct act8945a_dev_s *dev = filep->f_inode->i_private;
  int ret;

  ret = nxsem_wait(&dev->batsem);
  if (ret < 0)
    {
      return ret;
    }

  list_delete(&priv->node);
  nxsem_post(&dev->batsem);
  nxsem_destroy(&priv->lock);
  nxsem_destroy(&priv->wait);
  kmm_free(priv);

  return ret;
}

/****************************************************************************
 * Name: act8945a_read
 ****************************************************************************/

static ssize_t act8945a_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  FAR struct act8945a_priv_s *priv = filep->f_priv;
  int ret;

  if (buflen < sizeof(priv->mask))
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  while (priv->mask == 0)
    {
      nxsem_post(&priv->lock);
      if (filep->f_oflags & O_NONBLOCK)
        {
          return -EAGAIN;
        }

      ret = nxsem_wait(&priv->wait);
      if (ret < 0)
        {
          return ret;
        }

      ret = nxsem_wait(&priv->lock);
      if (ret < 0)
        {
          return ret;
        }
    }

  memcpy(buffer, &priv->mask, sizeof(priv->mask));
  priv->mask = 0;

  nxsem_post(&priv->lock);
  return sizeof(priv->mask);
}

/****************************************************************************
 * Name: act8945a_write
 ****************************************************************************/

static ssize_t act8945a_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen)
{
  /* Return nothing written */

  return 0;
}

/****************************************************************************
 * Name: act8945a_ioctl
 ****************************************************************************/

static int act8945a_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct act8945a_dev_s *dev  = inode->i_private;
  int ret;

  /* Enforce mutually exclusive access to the battery driver */

  ret = nxsem_wait(&dev->batsem);
  if (ret < 0)
    {
      return ret; /* Probably -EINTR */
    }

  /* Process the IOCTL command */

  ret = -EINVAL;  /* Assume a bad argument */
  switch (cmd)
    {
      case BATIOC_STATE:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          if (ptr)
            {
              ret = dev->ops->state(dev, ptr);
            }
        }
        break;

      case BATIOC_HEALTH:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          if (ptr)
            {
              ret = dev->ops->health(dev, ptr);
            }
        }
        break;

      case BATIOC_ONLINE:
        {
          FAR bool *ptr = (FAR bool *)((uintptr_t)arg);
          if (ptr)
            {
              ret = dev->ops->online(dev, ptr);
            }
        }
        break;

      case BATIOC_VOLTAGE:
        {
          int volts;
          FAR int *voltsp = (FAR int *)((uintptr_t)arg);
          if (voltsp)
            {
              volts = *voltsp;
              ret = dev->ops->voltage(dev, volts);
            }
        }
        break;

      case BATIOC_CURRENT:
        {
          int amps;
          FAR int *ampsp = (FAR int *)((uintptr_t)arg);
          if (ampsp)
            {
              amps = *ampsp;
              ret = dev->ops->current(dev, amps);
            }
        }
        break;

      case BATIOC_INPUT_CURRENT:
        {
          int amps;
          FAR int *ampsp = (FAR int *)((uintptr_t)arg);
          if (ampsp)
            {
              amps = *ampsp;
              ret = dev->ops->input_current(dev, amps);
            }
        }
        break;

      case BATIOC_OPERATE:
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          if (ptr)
            {
              ret = dev->ops->operate(dev, (uintptr_t)arg);
            }
        }
        break;

      case BATIOC_CHIPID:
        {
          FAR unsigned int *ptr = (FAR unsigned int *)((uintptr_t)arg);
          if (ptr)
            {
              ret = dev->ops->chipid(dev, ptr);
            }
        }
        break;

      case BATIOC_GET_VOLTAGE:
        {
          FAR int *outvoltsp = (FAR int *)((uintptr_t)arg);
          if (outvoltsp)
            {
              ret = dev->ops->get_voltage(dev, outvoltsp);
            }
        }
        break;

      default:
        _err("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  nxsem_post(&dev->batsem);
  return ret;
}

/****************************************************************************
 * Name: act8945a_poll
 ****************************************************************************/

static ssize_t act8945a_poll(FAR struct file *filep,
                                struct pollfd *fds, bool setup)
{
  FAR struct act8945a_priv_s *priv = filep->f_priv;
  int ret;

  ret = nxsem_wait(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      if (priv->fds == NULL)
        {
          priv->fds = fds;
          fds->priv = &priv->fds;
        }
      else
        {
          ret = -EBUSY;
        }
    }
  else if (fds->priv != NULL)
    {
      priv->fds = NULL;
      fds->priv = NULL;
    }

  nxsem_post(&priv->lock);

  if (setup)
    {
      act8945a_notify(priv, 0);
    }

  return ret;
}

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
  uint8_t regval;
  
  ret = act8945a_getreg(priv, reg_enable_register[regulator], &regval);

  if ((enable) && !(regval & 0x80))
    {
      /* we want to enable the regaultor and it wasn't enabled before */

      regval &= ~0x80;
      act8945a_info("INFO: Setting regulator %d enable bit to %x\n", (regulator + 1), regval);
      regval |= 0x80;
      act8945a_info("INFO: Setting regulator %d enable bit to %x\n", (regulator + 1), regval);
    }
  else if ((!enable) && (regval & 0x80))
    {
      /* we want to disable the regulator and it was enabled before */

      regval |= 0x80;
      act8945a_info("INFO: Setting regulator %d enable bit to %x\n", (regulator + 1), regval);
      regval &= ~0x80;
      act8945a_info("INFO: Setting regulator %d enable bit to %x\n", (regulator + 1), regval);
    }
  else
    {
      act8945a_info("INFO: no change to regulator %d enable/disable required\n", (regulator+1));
      return OK;
    }

  ret = act8945a_putreg(priv, reg_enable_register[regulator], regval);
  return ret;

}


/***************************************************************************
 * Name: act8945a_enable_lowiq
 *
 * Description:
 *   Enable/disable the regulator LOWIQ mode as required
 *   - only valid for regaultors 4,5,6 and 7
 *   - when set to 1enables low power operating mode
 *     NB: this decreases line regulation by x10
 *
 ***************************************************************************/
static int act8945a_enable_lowiq(FAR struct act8945a_dev_s *priv,
                                    enum act8945a_regulator regulator, bool enable)
{
  uint8_t regval;
  int ret;
  
  if (regulator < REG4)
    {
      act8945a_err("WARN: Setting LOWIQ mode on unsupported regulator: %d\n",
                   (regulator + 1));
      return -EINVAL;
    }

  ret = act8945a_getreg(priv, reg_control_register[regulator], &regval);
  if (ret < 0)
    {
      act8945a_err("ERR: failed to read register %d:\n",
                    reg_control_register[regulator]); 
      return -EIO;
    }
  if (enable)
    {
      /* we want to enable the LOW-IQ mode` */

      regval &= ACT8945A_LOW_IQ_ENABLE;
      act8945a_info("INFO: Setting regulator %d LOWIQ mode on\n",
                    (regulator + 1));
    }
  else 
    {
      /* we want to disable the LOWIQ mode*/

      regval |= ~ACT8945A_LOW_IQ_ENABLE;
      act8945a_info("INFO: Setting regulator %d LOWIQ mode off\n",
                    (regulator + 1));
    }

  return OK;//act8945a_putreg(priv, reg_control_register[regulator], regval);

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
  uint8_t newvolts;
  int ret = OK;
  uint8_t curvolts;
  
  if ((voltreqd > 3900) || (voltreqd < 600))
    {
      act8945a_warn("WARN: Requested voltage out of range for reg %d: %d\n",
                    (regulator+1), voltreqd);
      return -EINVAL;
    }
  else
    {
      newvolts = CONV_FROM_MV(voltreqd);
      if (newvolts != -1)
        {
          ret = act8945a_getreg(priv, reg_volt_register[regulator],
                                      &curvolts);          
          if (newvolts != (curvolts &= ACT8945A_VSET_MASK))
            {
              act8945a_info("INFO: Setting regulator %d from %dmV to %dmV\n",
                            (regulator + 1), CONV_TO_MV(curvolts),
                                             CONV_TO_MV(newvolts));
              ret = act8945a_putreg(priv,
                                    reg_volt_register[regulator],
                                    newvolts);
              return ret;
            }
          else
            {
              act8945a_info("INFO: no voltage change required, regulator %d\n",
                      (regulator+1));
              return OK;
            }
        }
      else
        {
          act8945a_warn("WARN: with ACT8945A volt register calculation\n");
          return -EINVAL;
        }
    }
    
  return OK;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct battery_charger_dev_s *
act8945a_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                  uint32_t frequency)
{
  FAR struct act8945a_dev_s *priv;

  /* Initialize the act8945a device structure */

  priv = (FAR struct act8945a_dev_s *)kmm_zalloc(sizeof(struct act8945a_dev_s));

  if (priv)
    {
      /* Initialize the act8945a device structure */

      priv->ops       = &g_act8945a_ops; 
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
  /* Initialise regulators */

  act8945a_set_reg_voltage(priv, REG1, CONFIG_ACT8945A_REG1_VOLTAGE);
#ifdef CONFIG_ACT8945A_REG1_ENABLE
  act8945a_enable_reg(priv, REG1, true);
#else
  act8945a_enable_reg(priv, REG1, false);
#endif

  act8945a_set_reg_voltage(priv, REG2, CONFIG_ACT8945A_REG2_VOLTAGE);
#ifdef CONFIG_ACT8945A_REG2_ENABLE
  act8945a_enable_reg(priv, REG2, true);
#else
  act8945a_enable_reg(priv, REG2, false);
#endif

  act8945a_set_reg_voltage(priv, REG3, CONFIG_ACT8945A_REG3_VOLTAGE);
#ifdef CONFIG_ACT8945A_REG3_ENABLE
  act8945a_enable_reg(priv, REG3, true);
#else
  act8945a_enable_reg(priv, REG3, false);
#endif

  act8945a_set_reg_voltage(priv, REG4, CONFIG_ACT8945A_REG4_VOLTAGE);
#ifdef CONFIG_ACT8945A_REG4_ENABLE
  act8945a_enable_reg(priv, REG4, true);
#else
  act8945a_enable_reg(priv, REG4, false);
#endif

  act8945a_set_reg_voltage(priv, REG5, CONFIG_ACT8945A_REG5_VOLTAGE);
#ifdef CONFIG_ACT8945A_REG5_ENABLE
  act8945a_enable_reg(priv, REG5, true);
#else
  act8945a_enable_reg(priv, REG5, false);
#endif

  act8945a_set_reg_voltage(priv, REG6, CONFIG_ACT8945A_REG6_VOLTAGE);
#ifdef CONFIG_ACT8945A_REG6_ENABLE
  act8945a_enable_reg(priv, REG6, true);
#else
  act8945a_enable_reg(priv, REG6, false);
#endif

  act8945a_set_reg_voltage(priv, REG7, CONFIG_ACT8945A_REG7_VOLTAGE);
#ifdef CONFIG_ACT8945A_REG7_ENABLE
  act8945a_enable_reg(priv, REG7, true);
#else
  act8945a_enable_reg(priv, REG7, false);
#endif  
  return (FAR struct act8945a_dev_s *)priv;
}
#if 0
/****************************************************************************
 * Name: act8945a_changed
 ****************************************************************************/

int act8945a_changed(FAR struct act8945a_dev_s *dev,
                             uint32_t mask)
{
  FAR struct act8945a_priv_s *priv;
  int ret;

  ret = nxsem_wait_uninterruptible(&dev->batsem);
  if (ret < 0)
    {
      return ret;
    }

  list_for_every_entry(&dev->flist, priv,
                       struct act8945a_priv_s, node)
    {
      act8945a_notify(priv, mask);
    }

  nxsem_post(&dev->batsem);
  return OK;
}
#endif
/****************************************************************************
 * Name: act8945a_register
 *
 * Description:
 *   Register a lower half battery driver with the common, upper-half
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
                      FAR struct battery_charger_dev_s *dev)
{
  int ret;

  /* Initialize the semaphore and the list */

  nxsem_init(&dev->batsem, 0, 1);
  list_initialize(&dev->flist);

  /* Register the character driver */

  ret = register_driver(devpath, &g_batteryops, 0666, dev);
  if (ret < 0)
    {
      _err("ERROR: Failed to register driver: %d\n", ret);
    }

  return ret;
}


#endif /* CONFIG_I2C && CONFIG_I2C_ACT8945A */
