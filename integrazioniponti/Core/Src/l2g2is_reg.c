/*
 ******************************************************************************
 * @file    l2g2is_reg.c
 * @author  Sensors Software Solution Team
 * @brief   L2G2IS driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
#include "l2g2is_reg.h"

/**
  * @defgroup    L2G2IS
  * @brief       This file provides a set of functions needed to drive the
  *              l2g2is enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup    L2G2IS_Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t l2g2is_read_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t l2g2is_write_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data,
                           uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L2G2IS_Sensitivity
  * @brief      These functions convert raw-data into engineering units.
  * @{
  *
  */

float_t l2g2is_from_fs100dps_to_mdps(int16_t lsb)
{
  return (((float_t)lsb *1000.0f)/262.0f);
}

float_t l2g2is_from_fs200dps_to_mdps(int16_t lsb)
{
  return (((float_t)lsb *1000.0f)/131.0f);
}

float_t l2g2is_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb *0.0625f)+25.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup    L2G2IS_Data_generation
  * @brief       This section groups all the functions concerning data
  *              generation.
  * @{
  *
  */

/**
  * @brief  Gyroscope new data available.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Iet the values of "xyda_ois" in reg DATA_STATUS_OIS.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_gy_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l2g2is_status_reg_t status_reg;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_STATUS_REG, (uint8_t*)&status_reg, 1);
  *val = status_reg.xyda;

  return ret;
}

/**
  * @brief  Gyroscope data rate selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "pw" in reg L2G2IS.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_gy_data_rate_set(stmdev_ctx_t *ctx,
                                  l2g2is_gy_data_rate_t val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if(ret == 0){
    ctrl_reg1.pw = (uint8_t)val;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG1,
                             (uint8_t*)&ctrl_reg1, 1);
  }
  return ret;
}

/**
  * @brief  Gyroscope data rate selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of pw in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_gy_data_rate_get(stmdev_ctx_t *ctx,
                                  l2g2is_gy_data_rate_t *val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  switch (ctrl_reg1.pw){
    case L2G2IS_GY_OFF:
      *val = L2G2IS_GY_OFF;
      break;
    case L2G2IS_GY_SLEEP:
      *val = L2G2IS_GY_SLEEP;
      break;
    case L2G2IS_GY_9k33Hz:
      *val = L2G2IS_GY_9k33Hz;
      break;
    default:
      *val = L2G2IS_GY_OFF;
      break;
  }
  return ret;
}

/**
  * @brief  Directional user orientation selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    "orient" Swap X axis with Y axis.
  *                "signy"  Y-axis angular rate sign selection.
  *                "signx"  X-axis angular rate sign selection.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_orient_config_set(stmdev_ctx_t *ctx,
                               l2g2is_orient_config_t val)
{
  l2g2is_orient_config_t	orient_config;

  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_ORIENT_CONFIG, (uint8_t*)&orient_config, 1);
  if(ret == 0) {
    orient_config.orient  = val.orient;
    ret = l2g2is_write_reg(ctx, L2G2IS_ORIENT_CONFIG, (uint8_t*)&orient_config, 1);
  }
  if(ret == 0) {
    orient_config.sign_x  = val.sign_x;
    orient_config.sign_y  = val.sign_y;
    ret = l2g2is_write_reg(ctx, L2G2IS_ORIENT_CONFIG, (uint8_t*)&orient_config, 1);
  }

  return ret;
}

/**
  * @brief  Directional user orientation selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    "orient" Swap X axis with Y axis.
  *                "signy"  Y-axis angular rate sign selection.
  *                "signx"  X-axis angular rate sign selection.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_orient_config_get(stmdev_ctx_t *ctx,
                                         l2g2is_orient_config_t *val)
{
  l2g2is_orient_config_t orient_config;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_ORIENT_CONFIG, (uint8_t*)&orient_config, 1);
  val->orient = orient_config.orient;
  val->sign_y = orient_config.sign_y;
  val->sign_y = orient_config.sign_y;
  return ret;
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of odu in reg CTRL_REG1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if(ret == 0){
    ctrl_reg1.odu = (uint8_t)val;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG1,
                             (uint8_t*)&ctrl_reg1, 1);
  }
  return ret;
}

/**
  * @brief  Blockdataupdate.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of odu in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  *val = (uint8_t)ctrl_reg1.odu;

  return ret;
}

/**
  * @brief  User offset correction.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    "offx" offset on X axis.
                   "offy" offset on Y axis.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_angular_rate_offset_set(stmdev_ctx_t *ctx,
                                         l2g2is_off_t val)
{
  l2g2is_off_x_t off_x;
  l2g2is_off_y_t off_y;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_OFF_X, (uint8_t*)&off_x, 1);
  if(ret == 0) {
    off_x.offx  = val.offx;
    ret = l2g2is_write_reg(ctx, L2G2IS_OFF_X, (uint8_t*)&off_x, 1);
  }
  if(ret == 0) {
    ret = l2g2is_read_reg(ctx, L2G2IS_OFF_Y, (uint8_t*)&off_y, 1);
  }
  if(ret == 0) {
    off_y.offy  = val.offy;
    ret = l2g2is_write_reg(ctx, L2G2IS_OFF_Y, (uint8_t*)&off_y, 1);
  }

  return ret;
}

/**
  * @brief  User offset correction.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    "offx" offset on X axis.
                   "offy" offset on Y axis.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_angular_rate_offset_get(stmdev_ctx_t *ctx,
                                          l2g2is_off_t *val)
{
  l2g2is_off_x_t off_x;
  l2g2is_off_y_t off_y;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_OFF_X, (uint8_t*)&off_x, 1);
  if(ret == 0) {
    ret = l2g2is_read_reg(ctx, L2G2IS_OFF_Y, (uint8_t*)&off_y, 1);
    val->offx = off_x.offx;
    val->offy = off_y.offy;
  }


  return ret;
}

/**
  * @brief  Gyroscope full-scale selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "fs_sel" in reg L2G2IS.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_gy_full_scale_set(stmdev_ctx_t *ctx,
                                   l2g2is_gy_fs_t val)
{
  l2g2is_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG4,
                          (uint8_t*)&ctrl_reg4, 1);
  if(ret == 0){
    ctrl_reg4.fs = (uint8_t)val;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG4,
                             (uint8_t*)&ctrl_reg4, 1);
  }
  return ret;
}

/**
  * @brief  Gyroscope full-scale selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of fs_sel in reg OIS_CFG_REG.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_gy_full_scale_get(stmdev_ctx_t *ctx,
                                   l2g2is_gy_fs_t *val)
{
  l2g2is_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG4,
                          (uint8_t*)&ctrl_reg4, 1);
  switch (ctrl_reg4.fs){
    case L2G2IS_100dps:
      *val = L2G2IS_100dps;
      break;
    case L2G2IS_200dps:
      *val = L2G2IS_200dps;
      break;
    default:
      *val = L2G2IS_100dps;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L2G2IS_Dataoutput
  * @brief       This section groups all the data output functions.
  * @{
  *
  */

/**
  * @brief  Temperature data output register (r). L and H registers together
  *         express a 16-bit word in two’s complement.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = l2g2is_read_reg(ctx, L2G2IS_TEMP_OUT_L, buff, 2);
  return ret;
}

/**
  * @brief  Angular rate sensor. The value is expressed as a 16-bit word in
  *         two’s complement.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_angular_rate_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = l2g2is_read_reg(ctx, L2G2IS_OUT_X_L, buff, 4);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L2G2IS_Common
  * @brief       This section groups common usefull functions.
  * @{
  *
  */

/**
  * @brief  DeviceWhoamI.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  buff   Buffer that stores the data read.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_dev_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
  int32_t ret;
  ret = l2g2is_read_reg(ctx, L2G2IS_WHO_AM_I, buff, 1);
  return ret;
}

/**
  * @brief  Device status register.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val     Data available on all axis.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_dev_status_get(stmdev_ctx_t *ctx,
                                l2g2is_dev_status_t *val)
{
  l2g2is_status_reg_t status_reg;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_STATUS_REG,
                          (uint8_t*)&status_reg, 1);
  val->xyda = status_reg.xyda;

  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "ble" in reg L2G2IS.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_dev_data_format_set(stmdev_ctx_t *ctx,
                                     l2g2is_ble_t val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if(ret == 0){
    ctrl_reg1.ble = (uint8_t)val;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG1,
                             (uint8_t*)&ctrl_reg1, 1);
  }
  return ret;
}

/**
  * @brief  Big/Little Endian data selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of ble in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_dev_data_format_get(stmdev_ctx_t *ctx,
                                     l2g2is_ble_t *val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  switch (ctrl_reg1.ble){
    case L2G2IS_LSB_LOW_ADDRESS:
      *val = L2G2IS_LSB_LOW_ADDRESS;
      break;
    case L2G2IS_MSB_LOW_ADDRESS:
      *val = L2G2IS_MSB_LOW_ADDRESS;
      break;
    default:
      *val = L2G2IS_LSB_LOW_ADDRESS;
      break;
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of boot in reg CTRL_REG1.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_dev_boot_set(stmdev_ctx_t *ctx, uint8_t val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if(ret == 0){
    ctrl_reg1.boot = (uint8_t)val;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  }
  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of boot in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_dev_boot_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  *val = (uint8_t)ctrl_reg1.boot;

  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of sw_rst in reg CTRL_REG2.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_dev_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  l2g2is_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  if(ret == 0){
    ctrl_reg2.sw_rst = (uint8_t)val;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  }
  return ret;
}

/**
  * @brief  Software reset. Restore the default values in user
  *          registers.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of sw_rst in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_dev_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l2g2is_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  *val = (uint8_t)ctrl_reg2.sw_rst;

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L2G2IS_Filters
  * @brief      This section group all the functions concerning the
  *              filters configuration.
  * @{
  *
  */

/**
  * @brief  Gyroscope high-pass filter bandwidth selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "hpf" in reg L2G2IS.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_gy_filter_hp_bandwidth_set(stmdev_ctx_t *ctx,
                                            l2g2is_gy_hp_bw_t val)
{
  l2g2is_ctrl_reg2_t ctrl_reg2;
  l2g2is_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  if(ret == 0){
    ctrl_reg2.hpf = ((uint8_t)val & 0x80U) >> 4;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG2,
                             (uint8_t*)&ctrl_reg2, 1);
  }
  if(ret == 0){
    ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG4,
                            (uint8_t*)&ctrl_reg4, 1);
  }
  if(ret == 0){
    ctrl_reg4.hpf_bw = (uint8_t)val & 0x03U;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG4,
                             (uint8_t*)&ctrl_reg4, 1);
  }

  return ret;
}

/**
  * @brief  Gyroscope high-pass filter bandwidth selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of hpf in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_gy_filter_hp_bandwidth_get(stmdev_ctx_t *ctx,
                                            l2g2is_gy_hp_bw_t *val)
{
  l2g2is_ctrl_reg2_t ctrl_reg2;
  l2g2is_ctrl_reg4_t ctrl_reg4;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  if(ret == 0){
    ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG4,
                            (uint8_t*)&ctrl_reg4, 1);

    switch ( ( ctrl_reg2.hpf << 4 ) + ctrl_reg4.hpf_bw){
      case L2G2IS_HPF_BYPASS:
        *val = L2G2IS_HPF_BYPASS;
        break;
      case L2G2IS_HPF_BW_23mHz:
        *val = L2G2IS_HPF_BW_23mHz;
        break;
      case L2G2IS_HPF_BW_91mHz:
        *val = L2G2IS_HPF_BW_91mHz;
        break;
      case L2G2IS_HPF_BW_324mHz:
        *val = L2G2IS_HPF_BW_324mHz;
        break;
      case L2G2IS_HPF_BW_1Hz457:
        *val = L2G2IS_HPF_BW_1Hz457;
        break;
      default:
        *val = L2G2IS_HPF_BYPASS;
        break;
    }
  }
  return ret;
}

/**
  * @brief  Gyroscope high-pass filter reset.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of hp_rst in reg CTRL_REG2.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_gy_filter_hp_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
  l2g2is_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  if(ret == 0){
    ctrl_reg2.hp_rst = (uint8_t)val;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  }
  return ret;
}

/**
  * @brief  Gyroscope high-pass filter reset.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of hp_rst in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_gy_filter_hp_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
  l2g2is_ctrl_reg2_t ctrl_reg2;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  *val = (uint8_t)ctrl_reg2.hp_rst;

  return ret;
}

/**
  * @brief   Gyroscope filter low pass cutoff frequency selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "lpf_bw" in reg L2G2IS.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
/*
int32_t l2g2is_gy_filter_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                            l2g2is_gy_lp_bw_t val)
{
  l2g2is_ctrl_reg2_t ctrl_reg2;
  l2g2is_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  if(ret == 0){
    ctrl_reg2.lpf_bw = (uint8_t)val & 0x03U;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
  }
  if(ret == 0){
    ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG3,
                              (uint8_t*)&ctrl_reg3, 1);
  }
  if(ret == 0){
    ctrl_reg3.lpf_bw = ((uint8_t)val & 0x04U) >> 2;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG3,
                               (uint8_t*)&ctrl_reg3, 1);
  }
  return ret;
}*/

/**
  * @brief   Gyroscope filter low pass cutoff frequency selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of lpf_bw in reg CTRL_REG2.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
/*
int32_t l2g2is_gy_filter_lp_bandwidth_get(stmdev_ctx_t *ctx,
                                            l2g2is_gy_lp_bw_t *val)
{
  l2g2is_ctrl_reg2_t ctrl_reg2;
  l2g2is_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG3,(uint8_t*)&ctrl_reg3, 1);
  if(ret == 0){
    ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    switch ( (ctrl_reg3.lpf_bw << 2) + ctrl_reg2.lpf_bw){
      case L2G2IS_LPF_BW_1150Hz:
        *val = L2G2IS_LPF_BW_1150Hz;
        break;
      case L2G2IS_LPF_BW_290Hz:
        *val = L2G2IS_LPF_BW_290Hz;
        break;
      case L2G2IS_LPF_BW_210Hz:
        *val = L2G2IS_LPF_BW_210Hz;
        break;
      case L2G2IS_LPF_BW_160Hz:
        *val = L2G2IS_LPF_BW_160Hz;
        break;
      case L2G2IS_LPF_BW_450Hz:
        *val = L2G2IS_LPF_BW_450Hz;
        break;
      default:
        *val = L2G2IS_LPF_BW_290Hz;
        break;
    }
  }
  return ret;
}*/

/**
  * @}
  *
  */

/**
  * @defgroup    L2G2IS_Serial_interface
  * @brief      This section groups all the functions concerning main
  *              serial interface management.
  * @{
  *
  */

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "sim" in reg L2G2IS.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_spi_mode_set(stmdev_ctx_t *ctx, l2g2is_sim_t val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if(ret == 0){
    ctrl_reg1.sim = (uint8_t)val;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  }
  return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of sim in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_spi_mode_get(stmdev_ctx_t *ctx, l2g2is_sim_t *val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  switch (ctrl_reg1.sim){
    case L2G2IS_SPI_4_WIRE:
      *val = L2G2IS_SPI_4_WIRE;
      break;
    case L2G2IS_SPI_3_WIRE:
      *val = L2G2IS_SPI_3_WIRE;
      break;
    default:
      *val = L2G2IS_SPI_4_WIRE;
      break;
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L2G2IS_Interrupt_pins
  * @brief       This section groups all the functions that manage
  *              interrupt pins.
  * @{
  *
  */

/**
  * @brief  Latched/pulsed interrupt.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "dr_pulsed" in reg L2G2IS.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_pin_notification_set(stmdev_ctx_t *ctx,
                                      l2g2is_lir_t val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if(ret == 0){
    ctrl_reg1.p_drdy = (uint8_t)val;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG1,
                             (uint8_t*)&ctrl_reg1, 1);
  }
  return ret;
}

/**
  * @brief  Latched/pulsed interrupt.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of dr_pulsed in reg CTRL_REG1.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_pin_notification_get(stmdev_ctx_t *ctx,
                                      l2g2is_lir_t *val)
{
  l2g2is_ctrl_reg1_t ctrl_reg1;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  switch (ctrl_reg1.p_drdy){
    case L2G2IS_INT_LATCHED:
      *val = L2G2IS_INT_LATCHED;
      break;
    case L2G2IS_INT_PULSED:
      *val = L2G2IS_INT_PULSED;
      break;
    default:
      *val = L2G2IS_INT_LATCHED;
      break;
  }
  return ret;
}

/**
  * @brief  Interrupt pin active-high/low.Interrupt active-high/low.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "h_l_active" in reg L2G2IS.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
/*
int32_t l2g2is_pin_polarity_set(stmdev_ctx_t *ctx,
                                  l2g2is_pin_pol_t val)
{
  l2g2is_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  if(ret == 0){
    ctrl_reg3.h_l_active = (uint8_t)val;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG3,
                             (uint8_t*)&ctrl_reg3, 1);
  }
  return ret;
}*/

/**
  * @brief  Interrupt pin active-high/low.Interrupt active-high/low.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of h_l_active in reg CTRL_REG3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
/*
int32_t l2g2is_pin_polarity_get(stmdev_ctx_t *ctx,
                                  l2g2is_pin_pol_t *val)
{
  l2g2is_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  switch (ctrl_reg3.h_l_active){
    case L2G2IS_ACTIVE_HIGH:
      *val = L2G2IS_ACTIVE_HIGH;
      break;
    case L2G2IS_ACTIVE_LOW:
      *val = L2G2IS_ACTIVE_LOW;
      break;
    default:
      *val = L2G2IS_ACTIVE_HIGH;
      break;
  }
  return ret;
}*/

/**
  * @brief  Route a signal on DRDY pin.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    "temp_data_on_drdy" Temperature data-ready signal.
  *                "drdy_en" Angular rate data-ready signal.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_pin_drdy_route_set(stmdev_ctx_t *ctx,
                                    l2g2is_pin_drdy_route_t val)
{
  l2g2is_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  if(ret == 0) {
    ctrl_reg3.drdy_en  = val.drdy_en;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  }
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    L2G2IS_Self_test
  * @brief       This section groups all the functions that manage self
  *              test configuration.
  * @{
  *
  */

/**
  * @brief  Enable/disable self-test mode for gyroscope.[set]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Change the values of "st_en" in reg L2G2IS.
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
int32_t l2g2is_gy_self_test_set(stmdev_ctx_t *ctx,
                                  l2g2is_gy_self_test_t val)
{
  l2g2is_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  if(ret == 0){
    ctrl_reg3.st = (uint8_t)val;
    ret = l2g2is_write_reg(ctx, L2G2IS_CTRL_REG3,
                             (uint8_t*)&ctrl_reg3, 1);
  }
  return ret;
}

/**
  * @brief  Enable/disable self-test mode for gyroscope.[get]
  *
  * @param  ctx    Read / write interface definitions.(ptr)
  * @param  val    Get the values of st_en in reg CTRL_REG3.(ptr)
  * @retval        Interface status (MANDATORY: return 0 -> no Error).
  *
  */
/*
int32_t l2g2is_gy_self_test_get(stmdev_ctx_t *ctx,
                                  l2g2is_gy_self_test_t *val)
{
  l2g2is_ctrl_reg3_t ctrl_reg3;
  int32_t ret;

  ret = l2g2is_read_reg(ctx, L2G2IS_CTRL_REG3, (uint8_t*)&ctrl_reg3, 1);
  switch ((ctrl_reg3.st_en << 1) + ctrl_reg3.st_sign){
    case L2G2IS_ST_DISABLE:
      *val = L2G2IS_ST_DISABLE;
      break;
    case L2G2IS_ST_POSITIVE:
      *val = L2G2IS_ST_POSITIVE;
      break;
    case L2G2IS_ST_NEGATIVE:
      *val = L2G2IS_ST_NEGATIVE;
      break;
    default:
      *val = L2G2IS_ST_DISABLE;
      break;
  }
  return ret;
}*/

/**
  * @}
  *
  */

/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
