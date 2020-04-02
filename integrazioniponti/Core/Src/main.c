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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hts221_reg.h"
#include "lps22hb_reg.h"
#include "l2g2is_reg.h"
#include "iis3dhhc_reg.h"

#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define T_VALUE 5        //NUMERO DI VOLTE CHE VIENE PROVATA LA COMUNICAZIONE CON I SENSORI
#define NO_ERR 0x00      //NESSUN ERRORE
#define ACC_ERR 0x01	 //ERRORE MODULO ACCELLEROMETRO
#define GYR_ERR 0x02	 //ERRORE MODULO GIROSCOPIO
#define PRE_ERR 0x04     //ERRORE MODULO PRESSIONE
#define HUM_ERR 0x08	 //ERRORE MODULO UMIDITA'
#define FIR_P 0x00000000 //MASCHERA PER PRIMO PACCHETTO
#define SEC_P 0x00000100 //MASCHERA PER SECONDO PACCHETTO
#define THI_P 0x00000300 //MASCHERA PER TERZO PACCHETTO
#define STD_MODE 1		 //MODALITA' STANDARD
#define STR_MODE 0       //MODALITA' AVVIO E FINE

#define CS_up_GPIO_Port GPIOB
#define CS_ACC		cs_1_Pin
#define CS_GIR		cs_2_Pin
#define TX_BUF_DIM      1000
#define START_SAMPLE 2
//ACCELEROMETRO
typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef union{
  int16_t i16bit;
  uint8_t u8bit[2];
} axis1bit16_t;
//GIROSCOPIO



typedef union{
  int16_t out_x;
  uint8_t u8_x[2];
} my_data;

//BAROMETRO
typedef union{
  int32_t i32bit;
  uint8_t u8bit[4];
} axis1bit32_t;
typedef struct{
		int16_t x;
		int16_t y;

	}Giro_Off;
l2g2is_off_t setOffset;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
uint8_t timeStart;
uint8_t Err=0;
//ACCELEROMETRO
static axis3bit16_t data_raw_acceleration;
//static axis1bit16_t data_raw_temperature;

//static float temperature_degC;
float temperature_raw;
static uint8_t whoamI, rst;

//GIROSCOPIO
static axis3bit16_t data_raw_angular_rate;

static axis1bit16_t data_raw_temperature;

static l2g2is_dev_status_t reg_gir;

//BAROMETRO

static axis1bit32_t data_raw_pressure;
static axis1bit16_t data_raw_temperature;


//HUMIDITY
static axis1bit16_t data_raw_humidity;
//CONTROLL BUS


//static uint8_t tx_buffer[TX_BUF_DIM];
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
    }Data[2];
uint8_t ID=0; //ID BOARD
uint8_t IDF=0;//ID Fake per schedulare invio pacchetti nel tempo
uint8_t flagStartData=0;//serve ad abilitare la trasmissione dei dati non importanti come la pressione e l'umidità che saranno letti solo all'inizio e alla fine
uint8_t flagStartAcc=0;//permette di leggere prima i dati e poi inviarli
uint8_t a=0;
uint8_t read[2];
uint32_t timer;
//CAN
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData0[8];
uint32_t TxMailbox;
CAN_FilterTypeDef sFilterConfig;
//ACCELEROMETRO
stmdev_ctx_t dev_ctx_acc;
iis3dhhc_reg_t reg_acc;
//GIROSCOPIO
stmdev_ctx_t dev_ctx_gir;
//UMIDITA'
stmdev_ctx_t dev_ctx_hum;
hts221_reg_t reg_hum;
//BAROMETER
stmdev_ctx_t dev_ctx_bar;
uint8_t reg_bar;
uint8_t rst_bar;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_write_gir(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read_gir(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_write_bar(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read_bar(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_write_hum(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static int32_t platform_read_hum(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static uint8_t get_data();
uint8_t make_giro_offset(stmdev_ctx_t *ctx,Giro_Off *giro);









typedef struct {
  float x0;
  float y0;
  float x1;
  float y1;
} lin_t;
float linear_interpolation(lin_t *lin, int16_t x)
{
 return ((lin->y1 - lin->y0) * x +  ((lin->x1 * lin->y0) - (lin->x0 * lin->y1)))
        / (lin->x1 - lin->x0);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/*
	  		*  Check device ID*/


  /* USER CODE END 1 */
  

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_CAN_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	TxHeader.StdId=0x312;
	TxHeader.ExtId=0x010;
	TxHeader.RTR=CAN_RTR_DATA;
	TxHeader.IDE=CAN_ID_STD;
	TxHeader.DLC=8;
	TxHeader.TransmitGlobalTime=DISABLE;

	//SETTAGGIO FILTRI E INIZIALIZZAZIONE CAN
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	sFilterConfig.FilterIdHigh=0x313<<5;
	sFilterConfig.FilterIdLow=0;
	sFilterConfig.FilterMaskIdHigh=0;
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan,&sFilterConfig);
	HAL_CAN_Start(&hcan);

	HAL_Delay(100);

	//RICONOSCIMENTO ID TRAMITE PONTICELLO
	if(HAL_GPIO_ReadPin(GPIOB,I0_Pin)){ID=ID|0x01;}
	if(HAL_GPIO_ReadPin(GPIOB,I1_Pin)){ID=ID|0x02;}
	if(HAL_GPIO_ReadPin(GPIOB,I2_Pin)){ID=ID|0x04;}
	if(HAL_GPIO_ReadPin(GPIOB,I3_Pin)){ID=ID|0x08;}
	if(HAL_GPIO_ReadPin(GPIOA,I4_Pin)){ID=ID|0x10;}
	//OGNI ID VIENE TRASFORMATO IN UN RANGE DA 0 A 9 E SERVIRA' PER TEMPORIZARE L'INVIO
	if(ID>9)IDF=ID-10;
	if(ID>19)IDF=ID-20;
	htim1.Init.Period = 1200+IDF*350;

	//TRASMISSIONE SPI DISABILITATA
	HAL_GPIO_WritePin(GPIOB, cs_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, cs_2_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

	//DEFINIZIONE DEI DEVICE
	//ACCELEROMETRO

	dev_ctx_acc.write_reg = platform_write;
	dev_ctx_acc.read_reg = platform_read;
	dev_ctx_acc.handle = &hspi1;
	//GIROSCOPIO

	dev_ctx_gir.write_reg = platform_write_gir;
	dev_ctx_gir.read_reg = platform_read_gir;
	dev_ctx_gir.handle = &hspi1;
	//UMIDITA'

	dev_ctx_hum.write_reg = platform_write_hum;
	dev_ctx_hum.read_reg = platform_read_hum;
	dev_ctx_hum.handle = &hi2c1;
	//BAROMETRO

	dev_ctx_bar.write_reg = platform_write_bar;
	dev_ctx_bar.read_reg = platform_read_bar;
	dev_ctx_bar.handle = &hi2c1;

	//INIZIALIZZAZIONE ACCELEROMETRO E VERIFICA DELLA CONNESSIONE AL BUS
	whoamI = 0;
	timeStart=STR_MODE;//ATTIVO START PACCHETTI PRESSIONE E UMIDITA'
	iis3dhhc_device_id_get(&dev_ctx_acc, &whoamI);
	while(timeStart>T_VALUE){// CONTROLLO SE E' POSSIBILE LA COMUNICAZIONE CON IL SENSORE
		if ( whoamI != IIS3DHHC_ID){
			break;
		}
	}
    if(timeStart>T_VALUE){
    	Err=Err|ACC_ERR;
    }
	iis3dhhc_reset_set(&dev_ctx_acc, PROPERTY_ENABLE);//RIPRISTINO SETTAGGI DEFAULT
	do {
	  iis3dhhc_reset_get(&dev_ctx_acc, &rst);
	} while (rst);
	iis3dhhc_block_data_update_set(&dev_ctx_acc, PROPERTY_ENABLE);//ABLITAZIONE DATAUPDATE
	iis3dhhc_data_rate_set(&dev_ctx_acc, IIS3DHHC_1kHz1);//SETTAGGIO FREQUENZA DI CAMPIONAMENTO
	iis3dhhc_filter_config_set(&dev_ctx_acc,3);//SETTAGGIO FILTRO FFR A 235HZ
	iis3dhhc_offset_temp_comp_set(&dev_ctx_acc, PROPERTY_ENABLE);//ABLITIAZIONE COMPENSAZIONE TEMPERATURA
	/*uint8_t speed= 0x01;
	uint8_t add=0x24|0x80;
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_ACC, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &add, 1, 10);
	if(HAL_SPI_Receive(&hspi1, &read[0], 1, 10)==HAL_OK)a=1;
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_ACC, GPIO_PIN_SET);
	add=0x24;
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_ACC, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &add, 1, 10);
	HAL_SPI_Transmit(&hspi1, &speed, 1, 10);
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_ACC, GPIO_PIN_SET);
	add=0x24|0x80;
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_ACC, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &add, 1, 10);
	if(HAL_SPI_Receive(&hspi1, &read[1], 1, 10)==HAL_OK)a=1;
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_ACC, GPIO_PIN_SET);*/



	//INIZIALIZZAZIONE GIROSCOPIO E VERIFICA DELLA CONNESSIONE AL BUS
	timeStart=0;
	l2g2is_dev_id_get(&dev_ctx_gir, &whoamI);
	while(timeStart>T_VALUE){
		if ( whoamI == L2G2IS_ID ){// CONTROLLO SE E' POSSIBILE LA COMUNICAZIONE CON IL SENSORE
			break;
		}
	}
	if(timeStart>T_VALUE){
	    Err=Err|GYR_ERR;
	}
	l2g2is_dev_reset_set(&dev_ctx_gir, PROPERTY_ENABLE);//RIPRISTINO SETTAGGI DEFAULT
	do {
	  l2g2is_dev_reset_get(&dev_ctx_gir, &rst);
	} while (rst);
	l2g2is_gy_full_scale_set(&dev_ctx_gir, L2G2IS_200dps);//SETTAGGIO SCALA DI MISURA -200DPS +200DPS
    //l2g2is_gy_filter_lp_bandwidth_set(&dev_ctx, L2G2IS_LPF_BW_160Hz);//SETTAGGIO FILTRO PASSA BASSO
	//l2g2is_gy_filter_hp_bandwidth_set(&dev_ctx, L2G2IS_HPF_BYPASS);//SETTAGGIO FILTRO PASSA ALTO
	setOffset.offx = 0x00;//SETTAGGIO OFFSET ASSE X
	setOffset.offy = 0x00;//SETTAGGIO OFFSET ASSE Y
	l2g2is_angular_rate_offset_set(&dev_ctx_gir, setOffset);//SETTAGGIO OFFSET ANGULAR RATE
	l2g2is_gy_data_rate_set(&dev_ctx_gir, L2G2IS_GY_9k33Hz);//SETTAGGIO FREQUENZA DI CAMPIONAMENTO DEL SENSORE
    //Data.Responce_Time_millis=make_giro_offset(&dev_ctx_gir,&giro); OFFSET NON FUNZIONANTE



	//INIZIALIZZAZIONE BAROMETRO E VERIFICA DELLA CONNESSIONE AL BUS
	whoamI=0;
	timeStart=0;
	lps22hb_device_id_get(&dev_ctx_bar,&whoamI);
    while(timeStart>T_VALUE){
    		if ( whoamI == LPS22HB_ID ){// CONTROLLO SE E' POSSIBILE LA COMUNICAZIONE CON IL SENSORE
    			break;
    		}
    	}
    	if(timeStart>T_VALUE){
    	    Err=Err|PRE_ERR;
    	}
	lps22hb_reset_set(&dev_ctx_bar, PROPERTY_ENABLE);//RIPRISTINO SETTAGGI DEFAULT
	do {
	lps22hb_reset_get(&dev_ctx_bar, &rst_bar);
	} while (rst_bar);
	//lps22hb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);//ABLITAZIONE DATAUPDATE
	lps22hb_low_pass_filter_mode_set(&dev_ctx_bar, LPS22HB_LPF_ODR_DIV_2);//SETTAGGIO LOW PASS FILTER
	//lps22hb_drdy_on_int_set(&dev_ctx, PROPERTY_ENABLE);//INTERRUPT FOR DATA READY INT_DRDY
	lps22hb_data_rate_set(&dev_ctx_bar, LPS22HB_ODR_10_Hz);//SETTAGGIO FREQUENZA DI CAMPIONAMENTO DEL SENSORE




	//INIZIALIZZAZIONE SENSORE DI UMIDITA' E VERIFICA DELLA CONNESSIONE AL BUS
	whoamI = 0;
	timeStart=0;
	hts221_device_id_get(&dev_ctx_hum, &whoamI);
	while(timeStart>T_VALUE){
			if ( whoamI == HTS221_ID ){// CONTROLLO SE E' POSSIBILE LA COMUNICAZIONE CON IL SENSORE
				break;
			}
		}
	if(timeStart>T_VALUE){
		Err=Err|HUM_ERR;
	}
	//CALIBRAZIONE SENSORE UMIDITA'
	axis1bit16_t coeff;
	lin_t lin_hum;
	hts221_hum_adc_point_0_get(&dev_ctx_hum, coeff.u8bit);
	lin_hum.x0 = (float)coeff.i16bit;
	hts221_hum_rh_point_0_get(&dev_ctx_hum, coeff.u8bit);
	lin_hum.y0 = (float)coeff.u8bit[0];
	hts221_hum_adc_point_1_get(&dev_ctx_hum, coeff.u8bit);
	lin_hum.x1 = (float)coeff.i16bit;
	hts221_hum_rh_point_1_get(&dev_ctx_hum, coeff.u8bit);
	lin_hum.y1 = (float)coeff.u8bit[0];

	/* Read temperature calibration coefficient */
	lin_t lin_temp;
	hts221_temp_adc_point_0_get(&dev_ctx_hum, coeff.u8bit);
	lin_temp.x0 = (float)coeff.i16bit;
	hts221_temp_deg_point_0_get(&dev_ctx_hum, coeff.u8bit);
	lin_temp.y0 = (float)coeff.u8bit[0];
	hts221_temp_adc_point_1_get(&dev_ctx_hum, coeff.u8bit);
	lin_temp.x1 = (float)coeff.i16bit;
	hts221_temp_deg_point_1_get(&dev_ctx_hum, coeff.u8bit);
	lin_temp.y1 = (float)coeff.u8bit[0];

	hts221_block_data_update_set(&dev_ctx_hum, PROPERTY_ENABLE);//ABLITAZIONE DATAUPDATE
	hts221_data_rate_set(&dev_ctx_hum, HTS221_ODR_1Hz);//SETTAGGIO DATARATE
	hts221_power_on_set(&dev_ctx_hum, PROPERTY_ENABLE);//ACCENSIONE INVIO DATI DISPOSITIVO




	//HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);//AVVIO DEL TIMER 3 E CONSEGUENTE AVVIO DEL PROGRAMMA
	HAL_TIM_Base_Start_IT(&htim16);//AVVIO DEL TIMER 16 PER INVIO DATI PRESSIONE E UMIDITA'
	int i=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 2;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 14999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, cs_1_Pin|cs_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : cs_1_Pin cs_2_Pin */
  GPIO_InitStruct.Pin = cs_1_Pin|cs_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : D6_Pin */
  GPIO_InitStruct.Pin = D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D5_Pin */
  GPIO_InitStruct.Pin = D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I4_Pin */
  GPIO_InitStruct.Pin = I4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(I4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I3_Pin I2_Pin I1_Pin I0_Pin */
  GPIO_InitStruct.Pin = I3_Pin|I2_Pin|I1_Pin|I0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_ACC, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, 1, 10);
	HAL_SPI_Transmit(handle, bufp, len, 10);
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_ACC, GPIO_PIN_SET);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	/* Read command */
	reg |= 0x80;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_ACC, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 10);
    HAL_SPI_Receive(handle, bufp, len, 10);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_ACC, GPIO_PIN_SET);
  return 0;
}


static int32_t platform_write_gir(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{	if (len>1)reg |= 0x40;
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_GIR, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, 1, 10);
	HAL_SPI_Transmit(handle, bufp, len, 10);
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_GIR, GPIO_PIN_SET);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_gir(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	/* Read command */
	reg |= 0x80;
	if (len>1)reg |= 0xC0;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_GIR, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 10);
    HAL_SPI_Receive(handle, bufp, len, 10);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_GIR, GPIO_PIN_SET);
  return 0;
}
static int32_t platform_write_bar(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Write(handle, LPS22HB_I2C_ADD_H, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
  }

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_bar(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  if (handle == &hi2c1)
  {
    HAL_I2C_Mem_Read(handle, LPS22HB_I2C_ADD_H, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
  }

  return 0;
}
static int32_t platform_write_hum(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  if (handle == &hi2c1)
  {
    /* Write multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Write(handle, HTS221_I2C_ADDRESS, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
  }

  return 0;
}



/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read_hum(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
  if (handle == &hi2c1)
  {
    /* Read multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Read(handle, HTS221_I2C_ADDRESS, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 100);
  }

  return 0;
}
uint8_t make_giro_offset(stmdev_ctx_t *ctx,Giro_Off *giro){
	l2g2is_dev_status_t reg_gir;

	l2g2is_dev_status_get(ctx, &reg_gir);
	for(int i=0;i<16;i++){
	if ( reg_gir.xyda )
			  	  {
			  		/* Read imu data */
			  		memset(data_raw_angular_rate.u8bit, 0x00, 2 * sizeof(int16_t));

			  		l2g2is_angular_rate_raw_get(ctx, data_raw_angular_rate.u8bit);

			  		giro->x=data_raw_angular_rate.i16bit[0]+giro->x;
			  		giro->y=data_raw_angular_rate.i16bit[1]+giro->y;
			  	  HAL_Delay(1);}
	else return 1;
}

	giro->x=giro->x/16;
	giro->y=giro->y/16;
return 0;}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==TIM16) //check if the interrupt comes from TIM2
        {
    	HAL_TIM_Base_Stop_IT(&htim16);
    	flagStartData=STD_MODE;
        }
    if (htim->Instance==TIM3) //check if the interrupt comes from TIM2
            {
    		TIM1->CNT=0;
    		HAL_TIM_Base_Start_IT(&htim2);
    		HAL_TIM_Base_Start_IT(&htim1);
    		get_data();
            }
    if (htim->Instance==TIM1) //check if the interrupt comes from TIM2
            {
    		timer++;//TIMER INVIO PACCHETTI TICKER
    		//Data[0].Responce_Time_millis=TIM2->CNT;
    		switch (flagStartData){
    			case STD_MODE:{
					HAL_TIM_Base_Stop(&htim1);
					TxHeader.StdId=(ID)|FIR_P;
					TxData0[0]=(int8_t)(timer  & 0x000000FF);
					TxData0[1]=(int8_t)((timer & 0x0000FF00)>>8);
					TxData0[2]=(int8_t)((timer & 0x00FF0000)>>16);
					TxData0[3]=(int8_t)(Data[0].Gir_x & 0x00FF);
					TxData0[4]=(int8_t)((Data[0].Gir_x & 0xFF00 )>> 8);
					TxData0[5]=(int8_t)(Data[0].Gir_y  & 0x00FF);
					TxData0[6]=(int8_t)((Data[0].Gir_y & 0xFF00 )>> 8);
					TxData0[7]=Err;
					HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData0,&TxMailbox);//INVIO PRIMO MESSAGGIO
					TxHeader.StdId=(ID)|SEC_P;

					TxData0[0]=(int8_t)(Data[0].Acc_x & 0x00FF);
					TxData0[1]=(int8_t)((Data[0].Acc_x & 0xFF00 )>> 8);
					TxData0[2]=(int8_t)(Data[0].Acc_y & 0x00FF);
					TxData0[3]=(int8_t)((Data[0].Acc_y & 0xFF00 )>> 8);
					TxData0[4]=(int8_t)(Data[0].Acc_z & 0x00FF);
					TxData0[5]=(int8_t)((Data[0].Acc_z & 0xFF00 )>> 8);
					TxData0[6]=(int8_t)(Data[0].T_b & 0x00FF);
					TxData0[7]=(int8_t)((Data[0].T_b & 0xFF00 )>> 8);
					HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData0,&TxMailbox);//IONVIO SECONDO MESSAGGIO
					break;
    			}
    			case STR_MODE:{
					TxHeader.StdId=(ID)|THI_P;
					TxData0[0]=(int8_t)(Data[0].Pres  & 0x000000FF);
					TxData0[1]=(int8_t)((Data[0].Pres & 0x0000FF00)>>8);
					TxData0[2]=(int8_t)((Data[0].Pres & 0x00FF0000)>>16);
					TxData0[3]=(int8_t)((Data[0].Pres & 0xFF000000)>>24);
					TxData0[4]=(int8_t)(Data[0].Hum  & 0x000000FF);
					TxData0[5]=(int8_t)((Data[0].Hum & 0x0000FF00)>>8);
					TxData0[6]=0x00;
					TxData0[7]=0x00;
					HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData0,&TxMailbox);//INVIO TERZO MESSAGGIO SOLO INIZIO E FINE
					break;
    			}
    			default:break;
    			}
     }


}
uint8_t get_data(){
	TIM2->CNT=0;
	//DECIDE CHE LETTURE ANDARE A FARE E PERMETTER DI LEGGERE I DATI NON ESSENZIALI SOLO ALL'INIZIO E ALLA FINE
	switch (flagStartData){
    	case STD_MODE:{
    		//ACCELLERETION FROM ACCELEROMETER
			iis3dhhc_status_get(&dev_ctx_acc, &reg_acc.status);
			if (reg_acc.status.zyxda){
				memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
				iis3dhhc_acceleration_raw_get(&dev_ctx_acc, data_raw_acceleration.u8bit);
				Data[0].Acc_x=data_raw_acceleration.i16bit[0];
				Data[0].Acc_y=data_raw_acceleration.i16bit[1];
				Data[0].Acc_z=data_raw_acceleration.i16bit[2];
				//memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
				//iis3dhhc_temperature_raw_get(&dev_ctx_acc, data_raw_temperature.u8bit);
				//data_raw_temperature.i16bit = data_raw_temperature.i16bit >> 4;
				//Data[0].T_a=data_raw_temperature.i16bit+25*16;
			}
			//ANGULAR RATE FROM GYROSCOPE
			l2g2is_dev_status_get(&dev_ctx_gir, &reg_gir);
			if ( reg_gir.xyda ){
				memset(data_raw_angular_rate.u8bit, 0x00, 2 * sizeof(int16_t));
				l2g2is_angular_rate_raw_get(&dev_ctx_gir, data_raw_angular_rate.u8bit);
				Data[0].Gir_x=data_raw_angular_rate.i16bit[0];
				Data[0].Gir_y=data_raw_angular_rate.i16bit[1];
				//memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
				//l2g2is_temperature_raw_get(&dev_ctx_gir, data_raw_temperature.u8bit);
				//data_raw_temperature.i16bit = data_raw_temperature.i16bit >> 4;
				//Data[0].T_g=data_raw_temperature.i16bit+25*1/0.0625;
			}
			//TEMPERATURE OF PRESSURE SENSOR
			lps22hb_press_data_ready_get(&dev_ctx_bar, &reg_bar);
			if (reg_bar){
				memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
				lps22hb_temperature_raw_get(&dev_ctx_bar, data_raw_temperature.u8bit);
				Data[0].T_b=data_raw_temperature.i16bit;
			}
			break;
    	}
    	case STR_MODE:{
			//BAROMETRO

			//dev_ctx_bar.write_reg = platform_write_bar;
			//dev_ctx_bar.read_reg = platform_read_bar;
			//dev_ctx_bar.handle = &hi2c1;


			// Read output only if new value is available
			lps22hb_press_data_ready_get(&dev_ctx_bar, &reg_bar);
			if (reg_bar){
				memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
				lps22hb_pressure_raw_get(&dev_ctx_bar, data_raw_pressure.u8bit);
				Data[0].Pres=data_raw_pressure.i32bit;
				//pressure_hPa = lps22hb_from_lsb_to_hpa(data_raw_pressure.i32bit);
				memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
				lps22hb_temperature_raw_get(&dev_ctx_bar, data_raw_temperature.u8bit);
				Data[0].T_b=data_raw_temperature.i16bit;
				//temperature_degC = lps22hb_from_lsb_to_degc(data_raw_temperature.i16bit);
			}

				//HUMIDITY
			hts221_status_get(&dev_ctx_hum, &reg_hum.status_reg);
			if (reg_hum.status_reg.h_da)
				{
				// Read humidity data
				memset(data_raw_humidity.u8bit, 0x00, sizeof(int16_t));
				hts221_humidity_raw_get(&dev_ctx_hum, data_raw_humidity.u8bit);
				//humidity_perc = linear_interpolation(&lin_hum, data_raw_humidity.i16bit);
				Data[0].Hum= data_raw_humidity.i16bit;
				//if (humidity_perc < 0) humidity_perc = 0;
				//if (humidity_perc > 100) humidity_perc = 100;
				}
			break;
    	}
    	default:break;
	}
	Data[0].Responce_Time_millis=TIM2->CNT;





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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
