/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FILTERS.h"
#include "READ_SENSORS.h"
#include "CanWriteBack.h"
#include "CanWrite.h"
#include "CanRead.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	SYS_START	= 0,
	SYS_ACC 	= 1,
	SYS_BRAKE 	= 2,
	SYS_DEBUG 	= 3,
} SysPhaseTypedef;

typedef enum{
	FALSE 	= 0,
	TRUE 	= 1,
} Boolean;

struct ControllerStruct {
	double 	Torque;
	double  Torque_filt;
	double 	Speed;
	double  Speed_filt;
	double  Speed_filt0;
	double  Acc;
	double  Acc_filt;
	Boolean FW;
	Boolean RW;
	Boolean N;
	Boolean FootSwitch;
	Boolean SeatSwitch;
	char 	Status;
	int 	Time;
	Boolean ToggleBit;
	unsigned char Fault;
};

struct SysStatusStruct {
	double Speed;
	double SpeedW;
	double Pressure;
	double Distance;
	long int DistanceCnt;
	unsigned int Cnt;
	int SysPhase;
	unsigned char Throttle;
};

struct ABSControlStruct {
	double controlTorque;
	double Fx;
	double Fx0;
	double FxFilt;
	double FxFilt0;
	double errFx;
	double FxD;
	double FxD_filt;
	double DFDT;
	double DFDT_filt;
	double Fmax;
	double Tmax;
	int k;
	int ks;
	int phase;
};

struct SpeedStruct {
	char enable;
	double v;
	double v_ref;
	double a;
	double a_ref;
	double errI;
	double errP;
	double kp;
	double ki;
	double kd;
	double force_max;	// max driving force
	double d_v;
};

struct ModelloKart{
	double integrale;
	double omega_modello;
};
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Led6_Pin GPIO_PIN_2
#define Led6_GPIO_Port GPIOE
#define Button2_Pin GPIO_PIN_4
#define Button2_GPIO_Port GPIOF
#define Button1_Pin GPIO_PIN_5
#define Button1_GPIO_Port GPIOF
#define SpeedLDir_Pin GPIO_PIN_13
#define SpeedLDir_GPIO_Port GPIOE
#define SpeedRDir_Pin GPIO_PIN_14
#define SpeedRDir_GPIO_Port GPIOE
#define GPIO_Output_Pin GPIO_PIN_14
#define GPIO_Output_GPIO_Port GPIOB
#define Led2_Pin GPIO_PIN_2
#define Led2_GPIO_Port GPIOG
#define Led4_Pin GPIO_PIN_3
#define Led4_GPIO_Port GPIOG
#define AzKey_Pin GPIO_PIN_7
#define AzKey_GPIO_Port GPIOG
#define Led1_Pin GPIO_PIN_5
#define Led1_GPIO_Port GPIOD
#define Led3_Pin GPIO_PIN_6
#define Led3_GPIO_Port GPIOD
#define Led5_Pin GPIO_PIN_7
#define Led5_GPIO_Port GPIOD
#define PowerRelay_Pin GPIO_PIN_7
#define PowerRelay_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
