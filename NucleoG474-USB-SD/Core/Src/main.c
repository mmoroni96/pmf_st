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
#include "Sigma2_Def.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union{
	uint8_t		Data8u[8];
	uint16_t	Data16u[4];
	int16_t		Data16[4];
}data_typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t chartotime(char* , uint8_t, uint8_t  );
int32_t chartocurr(char*, uint8_t, uint8_t );
FRESULT scrivi(BYTE* , uint8_t);
uint32_t PuntaeSepara(char*);
int32_t ProcessStatus = 0;
uint8_t ubKeyNumber = 0x0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
data_typedef rxData;
char buffer[100];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t id;
uint8_t DataSpi[8];
uint8_t readBuff[64];
uint8_t br;
int32_t curr;
uint32_t indice =0;
uint32_t time;
uint32_t indox=0;
uint8_t flag = 0;

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
uint32_t ID = 0x00;
MS_typedef ms;
CS_typedef cs;
DS_typedef ds;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Success_Handler(void);
void readSigmaData(void);
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

  /* Initialize interrupts */
  //MX_NVIC_Init();
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

      uint8_t workBuffer[_MAX_SS];
      FATFS USERFatFs;    /* File system object for USER logical drive */
      FIL USERFile,readFile,writeFile;       /* File  object for USER */
      char USERPath[4];   /* USER logical drive path */
      FRESULT res,res1; /* FatFs function common result code */
      uint8_t path1[] = "STM32.TXT";

      if(MY_SD_GetCardState(0) == BSP_ERROR_NONE){
    	  res = f_mkfs(USERPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));
    	  if (res != FR_OK){
			Error_Handler();
    	  }

      }
      res = f_mount(&USERFatFs, (TCHAR const*)USERPath, 0);
      res = f_open(&writeFile, &path1, FA_CREATE_ALWAYS);
      res = f_close(&writeFile);
      if(flag == 0){
		  flag = 1;
		  MX_NVIC_Init();
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


	/*if(MY_SD_GetCardState(0) == BSP_ERROR_NONE){
		//res = f_mkfs(USERPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));

		if (res != FR_OK){
			Error_Handler();
		}/*
		uint32_t byteswritten, bytesread; /* File write/read counts
		uint8_t wtext[] = "This is STM32 working with FatFs and uSD diskio driver"; /* File write buffer */
		/*uint8_t rtext[100]; /* File read buffer */
		/*uint8_t path0[] = "current.TXT";



		/* Register the file system object to the FatFs module */
		//res = f_mount(&USERFatFs, (TCHAR const*)USERPath, 0);



		/*if(res == FR_OK){}
		else while(1);
		/* Create and Open a new text file object with write access */
		/*for(uint32_t  e=0;e<10;e++){

		res = f_open(&readFile, &path0, FA_READ );


		f_lseek(&readFile, indice);
		for(uint32_t  i=0;i<1;i++){
			BYTE readBuf[30];
			strncpy((char*)readBuf, "1616161616", 10);
			UINT bytesWrote;


			//res = f_write(&USERFile, readBuf, 10,&bytesWrote);

			res = f_read(&readFile,readBuff, 34, &br);
			indice =indice+ PuntaeSepara(readBuff);

			}

		res = f_close(&readFile);

		}
		HAL_Delay(50);
*/


	//}
	/*else{
		Error_Handler();
	}*/

		   /*if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
		  	{
		  	  //Transmission request Error
		  	  Error_Handler();
		  	}*/
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* FDCAN1_IT0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*hcan, uint32_t RxFifo0ITs){
	if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&RxHeader,rxData.Data8u) != HAL_OK){
		/* Transmission request Error */
		Error_Handler();
	}
	ID = RxHeader.Identifier;
	readSigmaData();
    char datoGrezzo[5];
    datoGrezzo[0] = cs.ControllerTemperature;
    datoGrezzo[1] = ',';
    datoGrezzo[2] = cs.MotorTemperature;
    /*datoGrezzo[0] = 'A';
	datoGrezzo[1] = ',';
	datoGrezzo[2] = 'B';*/
	datoGrezzo[3] = 0x0d;
	datoGrezzo[4] = 0x0a;
    if(flag == 1){
    	scrivi(&datoGrezzo[0], sizeof(datoGrezzo));
    }

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void readSigmaData(void){
	switch(ID){
	case MS:
			ms.MotorSpeed			= rxData.Data16u[0];
			ms.MotorCurrent			= rxData.Data16[1];
			ms.MotorVoltage			= rxData.Data8u[4];
			ms.BatteryVoltage		= rxData.Data8u[5];
			ms.BatteryCurrent		= rxData.Data16[4];
	break;
	case DS:
			ds.ActualTorque			= rxData.Data16[0];
			ds.ActualSpeed			= rxData.Data16[1];
			ds.DriveStatusIndicator	= rxData.Data8u[4] & 0x0F;
			ds.SpeedLimitIndicator	= rxData.Data8u[4] >> 4;
			ds.TorqueLimitIndicator	= rxData.Data8u[5] & 0x0F;
			ds.MotorLimitIndicator	= rxData.Data8u[5] >> 4;
			ds.FaultCode			= rxData.Data8u[6];
			ds.Code					= rxData.Data8u[7];
	break;
	case CS:
			cs.ControllerTemperature= rxData.Data8u[0];
			cs.MotorTemperature		= rxData.Data8u[1];
			cs.BDI					= rxData.Data8u[2];
			cs.FaultSubCode			= rxData.Data8u[3]<<8 | rxData.Data8u[4];
	break;
	}
}

void Success_Handler(void)
{
  //printf("** Success. ** \n\r");
  while(1)
  {
  }
}
uint32_t chartotime(char* buff,uint8_t off, uint8_t leng ){
	char str[8];
	for(int i=off;i<leng+off;i++){
    str[i]=buff[i];
	}
	return (uint32_t)atoi(str);
}
int32_t chartocurr(char* buff,uint8_t off, uint8_t leng ){
	char str[8];
	for(int i=0;i<leng;i++){
    str[i]=buff[i+off];
	}
	return (int32_t)atoi(str);
}
uint32_t PuntaeSepara(char* buff){
	uint8_t h;
	uint8_t e;
	for(h=0;h<64;h++){
		if(readBuff[h]==','){
			time=chartotime(readBuff,0,h);
			break;
		}
	}
	for(e=h;e<64;e++){
		if(readBuff[e]==0xd&&readBuff[e+1]==0xa){
			curr=chartocurr(readBuff,h+1,e-h);
			break;
		}
	}
	return (uint32_t)(e+2);//aggiungo i due caratteri di terminazione
}
FRESULT scrivi(BYTE* readBuf, uint8_t size){
	FRESULT res1;
	FIL writeFile;       /* File  object for USER */
	FATFS USERFatFs;    /* File system object for USER logical drive */
	FIL USERFile;     /* File  object for USER */
	char USERPath[4];   /* USER logical drive path */
	uint8_t bytesWrote;
	uint8_t path1[] = "STM32.TXT";
	res1 = f_mount(&USERFatFs, (TCHAR const*)USERPath, 0);
	//res1 = f_open(&writeFile, &path1, FA_CREATE_ALWAYS);
	//res1 = f_close(&writeFile);
	res1 = f_open(&writeFile, &path1, FA_WRITE | FA_OPEN_ALWAYS);
	f_lseek(&writeFile, indox);
	indox=indox+5;
	res1 = f_write(&writeFile, readBuf, size, &bytesWrote);
	res1 = f_close(&writeFile);
	return res1;
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
