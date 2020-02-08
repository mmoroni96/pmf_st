/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_fatfs.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int32_t ProcessStatus = 0;
uint8_t ubKeyNumber = 0x0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
char buffer[100];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t id;
uint8_t DataSpi[8];
struct{
	int16_t Acc_x;
	int16_t Acc_y;
	int16_t Acc_z;
	int16_t Gir_x;
	int16_t Gir_y;
	int16_t T_a;
	int16_t T_g;
	int16_t T_b;
	uint32_t Pres;
	uint16_t Hum;
	uint16_t Responce_Time_millis;
	uint32_t Timer;
	uint8_t ID;
    }Data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Success_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* Initialize the micro SD Card */
  if(MY_SD_Init(0) != BSP_ERROR_NONE){
	  Error_Handler();
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_SPI2_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  TxHeader.Identifier = 0x0;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    TxData[0] = ubKeyNumber++;
    TxData[1] = 0xAD;
    TxData[2] = 0xDE;
    TxData[3] = 0xAD;
    TxData[4] = 0xBE;
    TxData[5] = 0xEF;
    TxData[6] = 0xFA;
    TxData[7] = 0xCE;
    uint16_t aug=3456;
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
        {
          /* Start Error */
          Error_Handler();
        }

      if ( HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
          /* Notification Error */
          Error_Handler();
        }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*ProcessStatus = MX_FATFS_Process();
	 Call middleware background task */
	/*if (ProcessStatus == APP_ERROR)
	{
	  Error_Handler();
	}
	else if (ProcessStatus == APP_OK)
	{
	  Success_Handler();
	}
*/

	uint8_t workBuffer[_MAX_SS];
	FATFS USERFatFs;    /* File system object for USER logical drive */
	FIL USERFile;       /* File  object for USER */
	char USERPath[4];   /* USER logical drive path */
	FRESULT res; /* FatFs function common result code */
	if(MY_SD_GetCardState(0) == BSP_ERROR_NONE){
		res = f_mkfs(USERPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));

		if (res != FR_OK){
			Error_Handler();
		}/*
		uint32_t byteswritten, bytesread; /* File write/read counts
		uint8_t wtext[] = "This is STM32 working with FatFs and uSD diskio driver"; /* File write buffer */
		uint8_t rtext[100]; /* File read buffer */
		uint8_t path[] = "STM32.TXT";

		/* Register the file system object to the FatFs module */
		res = f_mount(&USERFatFs, (TCHAR const*)USERPath, 0);
		if(res == FR_OK){}
		else while(1);
		/* Create and Open a new text file object with write access */
		for(uint32_t  e=0;e<1;e++){
		res = f_open(&USERFile, &path, FA_WRITE | FA_OPEN_ALWAYS );
		for(uint32_t  i=0;i<100000;i++){
			BYTE readBuf[30];
			strncpy((char*)readBuf, "1616161616", 10);
			UINT bytesWrote;
			res = f_write(&USERFile, readBuf, 10,&bytesWrote);
			}

		res = f_close(&USERFile);

		}
		HAL_Delay(50);



	}
	else{
		Error_Handler();
	}

		   if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
		  	{
		  	  //Transmission request Error
		  	  Error_Handler();
		  	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*hcan, uint32_t RxFifo0ITs)
{
	HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&RxHeader,RxData);

	if(RxHeader.Identifier<100){
		Data.ID=(uint8_t)RxHeader.Identifier;
		Data.Timer=RxData[0]+RxData[1]*256+RxData[2]*256*256+RxData[3]*256*256*256;
		Data.Gir_x=RxData[4]+RxData[5]*256;
		Data.Gir_y=RxData[6]+RxData[7]*256;
	}
	else{
		Data.ID=(uint8_t)RxHeader.Identifier&0x0FFFFFFF;
		Data.Acc_x=RxData[0]+RxData[1]*256;
		Data.Acc_y=RxData[2]+RxData[3]*256;
		Data.Acc_z=RxData[4]+RxData[5]*256;
		Data.T_b=RxData[6]+RxData[7]*256;
	}

}
void Success_Handler(void)
{
  //printf("** Success. ** \n\r");
  while(1)
  {
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	//printf("** Error. ** \n\r");
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
