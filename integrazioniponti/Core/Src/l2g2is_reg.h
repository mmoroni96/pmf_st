/*
 ******************************************************************************
 * @file    l2g2is_reg.h
 * @author  Sensors Software Solution Team
 * @brief   This file contains all the functions prototypes for the
 *          l2g2is_reg.c driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef L2G2IS_REGS_H
#define L2G2IS_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <math.h>

/** @addtogroup L2G2IS
  * @{
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct{
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*stmdev_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
	*              You can create a sensor configuration by your own or using 
	*              Unico / Unicleo tools available on STMicroelectronics
	*              web site.
  *
  * @{
  *
  */

typedef struct {
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/** @defgroup L2G2IS_Infos
  * @{
  *
  */

/** Device Identification (Who am I) **/
#define L2G2IS_ID                     0xD9U

/**
  * @}
  *
  */

#define L2G2IS_WHO_AM_I               0x00U
#define L2G2IS_TEMP_OUT_L             0x01U
#define L2G2IS_TEMP_OUT_H             0x02U
#define L2G2IS_OUT_X_L                0x03U
#define L2G2IS_OUT_X_H                0x04U
#define L2G2IS_OUT_Y_L                0x05U
#define L2G2IS_OUT_Y_H                0x06U
#define L2G2IS_STATUS_REG             0x09U
typedef struct {
  uint8_t not_used_01         : 1;
  uint8_t yda                 : 1;
  uint8_t xda                 : 1;
  uint8_t xyda                : 1;
  uint8_t not_used_02         : 1;
  uint8_t yor                 : 1;
  uint8_t xor                 : 1;
  uint8_t xyor                : 1;
} l2g2is_status_reg_t;

#define L2G2IS_CTRL_REG1              0x0BU
typedef struct {
  uint8_t pw                  : 2;
  uint8_t not_used_01         : 1;
  uint8_t odu                 : 1;
  uint8_t sim                 : 1;
  uint8_t ble                 : 1;
  uint8_t p_drdy              : 1;
  uint8_t boot                : 1;
} l2g2is_ctrl_reg1_t;

#define L2G2IS_CTRL_REG2              0x0CU
typedef struct {
  uint8_t hpf                 : 1;
  uint8_t sw_rst              : 1;
  uint8_t hp_rst              : 1;
  uint8_t not_used_01         : 1;
  uint8_t not_used_02         : 1;
  uint8_t not_used_03         : 1;
  uint8_t not_used_04         : 1;
  uint8_t lpf_o               : 1;
} l2g2is_ctrl_reg2_t;

#define L2G2IS_CTRL_REG3              0x0DU
typedef struct {
  uint8_t lpf_d               : 1;
  uint8_t drdy_en             : 1;
  uint8_t not_used_01         : 1;
  uint8_t not_used_02         : 1;
  uint8_t pp_od               : 1;
  uint8_t not_used_03         : 1;
  uint8_t st                  : 1;
  uint8_t not_used_04         : 1;
} l2g2is_ctrl_reg3_t;

#define L2G2IS_CTRL_REG4              0x1FU
typedef struct {
  uint8_t hpf_bw              : 1;
  uint8_t not_used_01         : 2;
  uint8_t fs                  : 1;
  uint8_t not_used_02         : 4;
} l2g2is_ctrl_reg4_t;

#define L2G2IS_OFF_X                  0x11U
typedef struct {
  uint8_t offx                : 8;
} l2g2is_off_x_t;

#define L2G2IS_OFF_Y                  0x12U
typedef struct {
  uint8_t offy                : 8;
} l2g2is_off_y_t;

#define L2G2IS_ORIENT_CONFIG          0x10U
typedef struct {
  uint8_t orient              : 1;
  uint8_t not_used_01         : 3;
  uint8_t sign_y              : 1;
  uint8_t sign_x              : 1;
  uint8_t not_used_02         : 2;
} l2g2is_orient_config_t;

/**
  * @defgroup L2G2IS_Register_Union
  * @brief    This union group all the registers that has a bit-field
  *           description.
  *           This union is useful but not need by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */

typedef union{
  l2g2is_status_reg_t                 status_reg;
  l2g2is_ctrl_reg1_t                  ctrl_reg1;
  l2g2is_ctrl_reg2_t                  ctrl_reg2;
  l2g2is_ctrl_reg3_t                  ctrl_reg3;
  l2g2is_ctrl_reg4_t                  ctrl_reg4;
  l2g2is_off_x_t                      off_x;
  l2g2is_off_y_t                      off_y;
  l2g2is_orient_config_t              orient_config;
  bitwise_t                           bitwise;
  uint8_t                             byte;
} l2g2is_reg_t;

/**
  * @}
  *
  */

int32_t l2g2is_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data, uint16_t len);
int32_t l2g2is_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data, uint16_t len);

extern float_t l2g2is_from_fs100dps_to_mdps(int16_t lsb);
extern float_t l2g2is_from_fs200dps_to_mdps(int16_t lsb);

extern float_t l2g2is_from_lsb_to_celsius(int16_t lsb);

int32_t l2g2is_gy_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  L2G2IS_GY_OFF    = 0,
  L2G2IS_GY_SLEEP  = 2,
  L2G2IS_GY_9k33Hz = 3,
} l2g2is_gy_data_rate_t;
int32_t l2g2is_gy_data_rate_set(stmdev_ctx_t *ctx, l2g2is_gy_data_rate_t val);
int32_t l2g2is_gy_data_rate_get(stmdev_ctx_t *ctx, l2g2is_gy_data_rate_t *val);

//----------------ORIENT FUNCTION---------------------------------------
int32_t l2g2is_orient_config_set(stmdev_ctx_t *ctx, l2g2is_orient_config_t val);
int32_t l2g2is_orient_config_get(stmdev_ctx_t *ctx, l2g2is_orient_config_t *val);


int32_t l2g2is_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t l2g2is_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef struct {
  uint8_t offx             : 7;
  uint8_t offy             : 7;
} l2g2is_off_t;
int32_t l2g2is_angular_rate_offset_set(stmdev_ctx_t *ctx,
                                         l2g2is_off_t val);
int32_t l2g2is_angular_rate_offset_get(stmdev_ctx_t *ctx,
                                         l2g2is_off_t *val);

typedef enum {
  L2G2IS_100dps   = 0,
  L2G2IS_200dps   = 1,
} l2g2is_gy_fs_t;
int32_t l2g2is_gy_full_scale_set(stmdev_ctx_t *ctx,
                                   l2g2is_gy_fs_t val);
int32_t l2g2is_gy_full_scale_get(stmdev_ctx_t *ctx,
                                   l2g2is_gy_fs_t *val);

int32_t l2g2is_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t l2g2is_angular_rate_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t l2g2is_dev_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

typedef struct {
  uint8_t xyda             : 1;
} l2g2is_dev_status_t;
int32_t l2g2is_dev_status_get(stmdev_ctx_t *ctx,
                                l2g2is_dev_status_t *val);

typedef enum {
  L2G2IS_LSB_LOW_ADDRESS = 0,
  L2G2IS_MSB_LOW_ADDRESS = 1,
} l2g2is_ble_t;
int32_t l2g2is_dev_data_format_set(stmdev_ctx_t *ctx,
                                     l2g2is_ble_t val);
int32_t l2g2is_dev_data_format_get(stmdev_ctx_t *ctx,
                                     l2g2is_ble_t *val);

int32_t l2g2is_dev_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t l2g2is_dev_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t l2g2is_dev_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t l2g2is_dev_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  L2G2IS_HPF_BYPASS    = 0x00,
  L2G2IS_HPF_BW_23mHz  = 0x80,
  L2G2IS_HPF_BW_91mHz  = 0x81,
  L2G2IS_HPF_BW_324mHz = 0x82,
  L2G2IS_HPF_BW_1Hz457 = 0x83,
} l2g2is_gy_hp_bw_t;
int32_t l2g2is_gy_filter_hp_bandwidth_set(stmdev_ctx_t *ctx,
                                            l2g2is_gy_hp_bw_t val);
int32_t l2g2is_gy_filter_hp_bandwidth_get(stmdev_ctx_t *ctx,
                                            l2g2is_gy_hp_bw_t *val);

int32_t l2g2is_gy_filter_hp_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t l2g2is_gy_filter_hp_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
  L2G2IS_LPF_BW_290Hz  = 0x00,
  L2G2IS_LPF_BW_210Hz  = 0x01,
  L2G2IS_LPF_BW_160Hz  = 0x02,
  L2G2IS_LPF_BW_450Hz  = 0x03,
  L2G2IS_LPF_BW_1150Hz = 0x04,
} l2g2is_gy_lp_bw_t;
int32_t l2g2is_gy_filter_lp_bandwidth_set(stmdev_ctx_t *ctx,
                                            l2g2is_gy_lp_bw_t val);
int32_t l2g2is_gy_filter_lp_bandwidth_get(stmdev_ctx_t *ctx,
                                            l2g2is_gy_lp_bw_t *val);

typedef enum {
  L2G2IS_SPI_4_WIRE = 0,
  L2G2IS_SPI_3_WIRE = 1,
} l2g2is_sim_t;
int32_t l2g2is_spi_mode_set(stmdev_ctx_t *ctx, l2g2is_sim_t val);
int32_t l2g2is_spi_mode_get(stmdev_ctx_t *ctx, l2g2is_sim_t *val);

typedef enum {
  L2G2IS_INT_PULSED = 1,
  L2G2IS_INT_LATCHED = 0,
} l2g2is_lir_t;
int32_t l2g2is_pin_notification_set(stmdev_ctx_t *ctx,
                                      l2g2is_lir_t val);
int32_t l2g2is_pin_notification_get(stmdev_ctx_t *ctx,
                                      l2g2is_lir_t *val);

typedef enum {
  L2G2IS_ACTIVE_HIGH = 0,
  L2G2IS_ACTIVE_LOW = 1,
} l2g2is_pin_pol_t;
int32_t l2g2is_pin_polarity_set(stmdev_ctx_t *ctx,
                                  l2g2is_pin_pol_t val);
int32_t l2g2is_pin_polarity_get(stmdev_ctx_t *ctx,
                                  l2g2is_pin_pol_t *val);

typedef enum {
  L2G2IS_PUSH_PULL = 0,
  L2G2IS_OPEN_DRAIN = 1,
} l2g2is_pp_od_t;
int32_t l2g2is_pin_mode_set(stmdev_ctx_t *ctx, l2g2is_pp_od_t val);
int32_t l2g2is_pin_mode_get(stmdev_ctx_t *ctx, l2g2is_pp_od_t *val);

typedef struct {
  uint8_t temp_data_on_drdy             : 1;
  uint8_t drdy_en                       : 1;
} l2g2is_pin_drdy_route_t;
int32_t l2g2is_pin_drdy_route_set(stmdev_ctx_t *ctx,
                                    l2g2is_pin_drdy_route_t val);
int32_t l2g2is_pin_drdy_route_get(stmdev_ctx_t *ctx,
                                    l2g2is_pin_drdy_route_t *val);

typedef enum {
  L2G2IS_ST_DISABLE  = 0x00,
  L2G2IS_ST_POSITIVE = 0x02,
  L2G2IS_ST_NEGATIVE = 0x03,
} l2g2is_gy_self_test_t;
int32_t l2g2is_gy_self_test_set(stmdev_ctx_t *ctx,
                                  l2g2is_gy_self_test_t val);
int32_t l2g2is_gy_self_test_get(stmdev_ctx_t *ctx,
                                  l2g2is_gy_self_test_t *val);

/**
  *@}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /* L2G2IS_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
