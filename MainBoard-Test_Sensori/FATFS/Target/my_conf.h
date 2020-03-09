/**
  ******************************************************************************
  * @file    adafruit_802_conf.h
  * @author  MCD Application Team
  * @brief   This file includes the nucleo configuration and errno files
  *
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
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MY_CONF_H
#define MY_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_nucleo_conf.h"
#include "stm32g4xx_nucleo_errno.h"
#include "stm32g4xx_nucleo_bus.h"

#define BUS_SPIx_Init           BSP_SPI2_Init
#define BUS_SPIx_Recv           BSP_SPI2_Recv
#define BUS_SPIx_Send           BSP_SPI2_Send
#define BUS_SPIx_SendRecv       BSP_SPI2_SendRecv


/**
  * @brief  SD Control Interface pins (shield D4)
  */
#define MY_SD_CS_PIN                                GPIO_PIN_2
#define MY_SD_CS_GPIO_PORT                          GPIOB
#define MY_SD_CS_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define MY_SD_CS_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_DISABLE()

/**
  * @brief  SD Control Lines management
  */
#define MY_SD_CS_LOW()       HAL_GPIO_WritePin(MY_SD_CS_GPIO_PORT, MY_SD_CS_PIN, GPIO_PIN_RESET)
#define MY_SD_CS_HIGH()      HAL_GPIO_WritePin(MY_SD_CS_GPIO_PORT, MY_SD_CS_PIN, GPIO_PIN_SET)


#ifdef __cplusplus
}
#endif

#endif /* MY_CONF_H */
