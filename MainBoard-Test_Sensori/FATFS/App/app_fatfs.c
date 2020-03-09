/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   app_fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "app_fatfs.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  APPLICATION_IDLE = 0,
  APPLICATION_INIT,
  APPLICATION_RUNNING,
  APPLICATION_SD_UNPLUGGED,
}FS_FileOperationsTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FATFS_MKFS_ALLOWED 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FATFS USERFatFs;    /* File system object for USER logical drive */
FIL USERFile;       /* File  object for USER */
char USERPath[4];   /* USER logical drive path */
/* USER CODE BEGIN PV */
FS_FileOperationsTypeDef Appli_state = APPLICATION_IDLE;
uint8_t workBuffer[_MAX_SS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static int32_t FS_FileOperations(void);
/* USER CODE END PFP */

/**
  * @brief  FatFs initialization
  * @param  None
  * @retval Initialization result 
  */
int32_t MX_FATFS_Init(void) 
{
  /*## FatFS: Link the disk I/O driver(s)  ###########################*/
 
if (FATFS_LinkDriver(&USER_Driver, USERPath) != 0)
  /* USER CODE BEGIN FATFS_Init */
  {
    return APP_ERROR;
  }
  else
  {
    Appli_state = APPLICATION_INIT;
    return APP_OK;
  }
  /* USER CODE END FATFS_Init */
}

/**
  * @brief  FatFs application main process
  * @param  None
  * @retval Process result 
  */
int32_t MX_FATFS_Process(void)
{
  /* USER CODE BEGIN FATFS_Process */
  int32_t process_res = APP_OK;  
  /* Mass Storage Application State Machine */
	switch(Appli_state)
	{
	case APPLICATION_INIT:
	  if(MY_SD_GetCardState(0) == BSP_ERROR_NONE)
	  {
	#if FATFS_MKFS_ALLOWED
		FRESULT res;

		res = f_mkfs(USERPath, FM_FAT32, 0, workBuffer, sizeof(workBuffer));
		//res = FR_OK;
		if (res != FR_OK)
		{
		  process_res = APP_ERROR;
		}
		else
		{
		  process_res = APP_INIT;
		  Appli_state = APPLICATION_RUNNING;
		}
	#else
		process_res = APP_INIT;
		Appli_state = APPLICATION_RUNNING;
	#endif
	  }
	  else
	  {
	  process_res = APP_ERROR;

	  }

	  break;
	case APPLICATION_RUNNING:
		process_res = FS_FileOperations();
		Appli_state = APPLICATION_IDLE;
	  break;

	case APPLICATION_IDLE:
	default:
	  break;
	}
  return process_res;
  /* USER CODE END FATFS_Process */
}  

/**
  * @brief  Gets Time from RTC (generated when FS_NORTC==0; see ff.c)
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */  
}

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief File system : file operation
  * @retval File operation result
  */
static int32_t FS_FileOperations(void)
{
  FRESULT res; /* FatFs function common result code */
  uint32_t byteswritten, bytesread; /* File write/read counts */
  uint8_t wtext[] = "This is STM32 working with FatFs and uSD diskio driver"; /* File write buffer */
  uint8_t rtext[100]; /* File read buffer */
  uint8_t path[] = "STM32.TXT";

  /* Register the file system object to the FatFs module */
  res = f_mount(&USERFatFs, (TCHAR const*)USERPath, 0);
  if(res == FR_OK)
  {
    /* Create and Open a new text file object with write access */
	res = f_open(&USERFile, &path, FA_CREATE_ALWAYS | FA_WRITE);
    if(res == FR_OK)
    {
      /* Write data to the text file */
      res = f_write(&USERFile, wtext, sizeof(wtext), (void *)&byteswritten);

      if((byteswritten > 0) && (res == FR_OK))
      {
        /* Close the open text file */
        res = f_close(&USERFile);
        if(res == FR_OK){
        	HAL_Delay(50);
        }

        /* Open the text file object with read access */
        res = f_open(&USERFile, &path, FA_READ);
        //if(1)
        if(res == FR_OK)
        {
          /* Read data from the text file */
          res = f_read(&USERFile, rtext, sizeof(rtext), (void *)&bytesread);

          if((bytesread > 0) && (res == FR_OK))
          {
            /* Close the open text file */
            f_close(&USERFile);

            /* Compare read data with the expected data */
            if((bytesread == byteswritten))
            {
              /* Success of the demo: no error occurrence */
              return 0;
            }
          }
        }
      }
    }
  }
  /* Error */
  return -1;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
