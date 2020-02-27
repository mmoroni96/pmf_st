/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "cryp.h"
#include "dma.h"
#include "fatfs.h"
#include "fdcan.h"
#include "hash.h"
#include "i2c.h"
#include "iwdg.h"
#include "lwip.h"
#include "quadspi.h"
#include "rng.h"
#include "rtc.h"
#include "sdmmc.h"
#include "spi.h"
//#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dmc_fat.h"
#include <dmc_rtc.h>
#include <dmc_leds.h>
#include <dmc_dipswitch.h>
#include <dmc_mcu.h>
#include <dmc_net.h>
#include <dmc_print.h>
#include <dmc_rtc_mcp79412.h>
#include <KSZ8851SNL_0.h>
#include <KSZ8851SNL_1.h>
#include <string.h>
#include <dmc_tcp.h>
#include "fatfs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   (uint8_t) 192
#define IP_ADDR1   (uint8_t) 168
#define IP_ADDR2   (uint8_t) 25
#define IP_ADDR3   (uint8_t) 238

/*NETMASK*/
#define NETMASK_ADDR0   (uint8_t) 255
#define NETMASK_ADDR1   (uint8_t) 255
#define NETMASK_ADDR2   (uint8_t) 255
#define NETMASK_ADDR3   (uint8_t) 0

/*Gateway Address*/
#define GW_ADDR0   (uint8_t) 192
#define GW_ADDR1   (uint8_t) 168
#define GW_ADDR2   (uint8_t) 25
#define GW_ADDR3   (uint8_t) 253

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
struct netif gnetif;
static void Netif_Config(void);

/* USER CODE BEGIN PV */
uint32_t msTick = 0;
uint32_t msTickPrevious = 0;
uint32_t msTickPrevious2 = 0;
uint32_t Interval = 500;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void HAL_IncTicks(void);
void HAL_StartTicks(void);
uint8_t HAL_GetTicks(uint32_t ms);
void HAL_StartTicks2(void);
uint8_t HAL_GetTicks2(uint32_t ms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if defined ( __CC_ARM )  /* MDK ARM Compiler */

#elif defined ( __GNUC__ ) /* GNU Compiler */

#endif

/* Global Vars */
RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;
time_t timestamp;
struct tm currTime;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  DmcLedsOn();

  MX_UART7_Init();
  MX_USART2_UART_Init();

  HAL_GPIO_WritePin(UART7_TXE_GPIO_Port, UART7_TXE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(USART2_TXE_GPIO_Port, USART2_TXE_Pin, GPIO_PIN_SET);

  dmc_put_sethuart(&huart2);

  dmc_puts(TERMINAL_DEFAULT);

  dmc_puts("\n");
  dmc_puts("--------------------------------------------------------------------------------\n");
  dmc_puts("MX_DMA_Init\n");
  MX_DMA_Init();
  dmc_puts("MX_QUADSPI_Init\n");
  MX_QUADSPI_Init();
//  dmc_puts("MX_IWDG1_Init\n");
//  MX_IWDG1_Init();
  dmc_puts("MX_SPI1_Init\n");
  MX_SPI1_Init();
  dmc_puts("MX_SPI4_Init\n");
  MX_SPI4_Init();
  dmc_puts("MX_FDCAN1_Init\n");
  MX_FDCAN1_Init();
  dmc_puts("MX_FDCAN2_Init\n");
  MX_FDCAN2_Init();
  dmc_puts("MX_I2C4_Init\n");
  MX_I2C4_Init();
  dmc_puts("MX_RTC_Init\n");
  MX_RTC_Init();
//  dmc_puts("MX_CRYP_Init\n");
//  MX_CRYP_Init();
//  dmc_puts("MX_HASH_Init\n");
//  MX_HASH_Init();
//  dmc_puts("MX_RNG_Init\n");
//  MX_RNG_Init();
//  dmc_puts("MX_CRC_Init\n");
//  MX_CRC_Init();



  dmc_puts("MX_SDMMC1_SD_Init\n");
  MX_SDMMC1_SD_Init();  // Problem
// ERROR (write)
//  File: ..\..\..\Src\sdmmc.c
//  Line: 42
//  Error: 10

  dmc_puts("MX_FATFS_Init\n");
  MX_FATFS_Init();

  FRESULT res;
  FATFS MMCFatFs;
  FIL myFile;
  FATFS fs2;
  DSTATUS status;
  BYTE work[1024]; /* Work area (larger is better for processing time) */

  dmc_puts("----------------------------------------\n");

//  char USERPath[4];   /* USER logical drive path */

  dmc_puts("SDPath: ");
  dmc_puts(SDPath);
  dmc_putcr();


  /* Disk status */
  dmc_puts("Disk status: ");
  status = disk_status(0);
  if (status != RES_OK)
  {
    ShowDiskStatus(status);
  }
  else
  {
    dmc_puts("OK\n");
  }

  dmc_puts("Initialize disk: ");
  status = disk_initialize(0);
  if (status != RES_OK)
  {
    ShowDiskStatus(status);
  }
  else
  {
    dmc_puts("OK\n");
  }

  /* Disk status */
  dmc_puts("Disk status: ");
  status = disk_status(0);
  if (status != RES_OK)
  {
    ShowDiskStatus(status);
  }
  else
  {
    dmc_puts("OK\n");
  }

  uint8_t pWData[512];
  uint8_t pRData[512];
  uint16_t BlockSize = 512;
  uint32_t BlockNbr = 62688;
  uint32_t ReadAddr = 0;
  uint32_t WriteAddr = 0;
  uint32_t NumOfBlocks = 1;
  uint32_t Timeout = 1000;
  uint8_t SD_status = 0;

  for (uint16_t j = 0; j < BlockNbr; j++)
  {
    WriteAddr = j;
    ReadAddr = j;

    dmc_puthex8(ReadAddr);
    dmc_putc(' ');

    for (uint16_t i = 0; i < BlockSize; i++)
    {
      pWData[i] = (i & 0xff);
    }

    dmc_puts("BSP_SD_WriteBlocks: ");
    SD_status = BSP_SD_WriteBlocks((uint32_t *)pWData, WriteAddr, NumOfBlocks, Timeout);
    dmc_puts("WR SD_status: ");
    dmc_putint(SD_status);
    dmc_putc(' ');

    dmc_puts("BSP_SD_ReadBlocks: ");
    SD_status = BSP_SD_ReadBlocks((uint32_t *)pRData, ReadAddr, NumOfBlocks, Timeout);
    dmc_puts("RD SD_status: ");
    dmc_putint(SD_status);
    dmc_putc(' ');

    uint8_t match = 1;
    for (uint16_t i = 0; i < BlockSize; i++)
    {
      if (pWData[i] != pRData[i])
      {
        match = 0;
      }
    }
    dmc_puts("match: ");
    dmc_putint(match);
    dmc_putc(' ');

    for (uint16_t i = 0; i < BlockSize; i+=32)
    {
      dmc_puthex2(pRData[i]);
      dmc_putc(' ');
    }
    dmc_putcr();
  }

//  /* Format Disk */
//  /* Create FAT volume */
//  // The f_mkfs function creates an FAT/exFAT volume on the logical drive.
//  dmc_puts("Format disk\n");
//  printf(TERMINAL_GREEN);
//  res = f_mkfs(SDPath, FM_FAT, 0, work, sizeof(work));
////  res = f_mkfs(SDPath, FM_FAT32, 0, work, sizeof(work));
//  printf(TERMINAL_DEFAULT);
//  if (res != FR_OK)
//  {
//    ShowFatFsError(res);
//  }
//  else
//  {
//    dmc_puts("OK\n");
//  }
//
  /* Mount */
  dmc_puts("Mount disk: ");
  res = f_mount(&fs2, SDPath, 1);
  if (res != FR_OK)
  {
    ShowFatFsError(res);
  }
  else
  {
    dmc_puts("OK\n");
  }

//
//    /* Get volume label of the default drive */
////  printf("Get volume label\n");
//    char label[12];
//    res = f_getlabel(USERPath, label, 0);
//    if (res != FR_OK)
//    {
//      ShowFatFsError(res);
//    }
//    printf(TERMINAL_LIGHT_GREEN);
//  printf("Volume label: %s\n", label);
//    printf(TERMINAL_DEFAULT);
//
//  /* Get volume information and free clusters of drive 1 */
////  printf("Disk space:\n");
//  DWORD fre_clust, fre_sect, tot_sect;
//  res = f_getfree(USERPath, &fre_clust, &fs2);
//  if (res != FR_OK)
//  {
//      ShowFatFsError(res);
//  }
//  /* Get total sectors and free sectors */
//  tot_sect = (fs2->n_fatent - 2) * fs2->csize;
//  fre_sect = fre_clust * fs2->csize;
//  /* Print the free space (assuming 512 bytes/sector) */
//    printf(TERMINAL_LIGHT_GREEN);
//  printf("%10lu kB total disk space\n", tot_sect / 2);  //   15386720 kB total disk space
//  printf("%10lu kB available\n", fre_sect / 2);     //   15386688 kB available
//    printf(TERMINAL_DEFAULT);

  while(1)
  {
    ;
  }


}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_FDCAN
                              |RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_SPI4
                              |RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_I2C4|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_QSPI;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 128;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 64;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_PLL2;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector
  */
  HAL_PWREx_EnableUSBVoltageDetector();
}

/* USER CODE BEGIN 4 */
void HAL_IncTicks(void)
{
  msTick += (uint32_t) 1;
}

void HAL_StartTicks(void)
{
  msTickPrevious = msTick;
}

uint8_t HAL_GetTicks(uint32_t ms)
{
  if ((msTick - msTickPrevious) >= ms)
  {
    msTickPrevious = msTick;
    return TRUE;
  }
  return FALSE;
}

void HAL_StartTicks2(void)
{
  msTickPrevious2 = msTick;
}

uint8_t HAL_GetTicks2(uint32_t ms)
{
  if ((msTick - msTickPrevious2) >= ms)
  {
    msTickPrevious2 = msTick;
    return TRUE;
  }
  return FALSE;
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Initialize and configure the Region and the memory to be protected */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Initialize and configure the Region and the memory to be protected */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30044000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Initialize and configure the Region and the memory to be protected */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
//  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
//  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
//  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  DMC_ErrorHandler(file, line);
//
//  dmc_puts("_Error_Handler(");
//  dmc_puts(file);
//  dmc_puts(", ");
//  dmc_putint(line);
//  dmc_puts(")\n");
//  while (1)
//  {
////    printf("_Error_Handler(%s, %d)\n", file, line);
//  }
  /* USER CODE END Error_Handler_Debug */
}

void _Error_Handler2(char *file, int line, uint error)
{
  DMC_ErrorHandler2(file, line, error);
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
