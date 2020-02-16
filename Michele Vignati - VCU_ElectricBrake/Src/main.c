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
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CanRead.h"
#include "CanWrite.h"
#include "CanWriteBack.h"
#include "CtrlSpeed.h"
#include "Controllo.h"
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
SysPhaseTypedef SysPhase = SYS_START;
int SysPhaseCount = 0;
/* ========================================================================================
 * Timing
 * ======================================================================================== */

char 		enable10ms = 0;
char 		conta100 = 0;
long int 	tempo 	= 0; 	// tempo dall'avvio in secondi
int 		tempo1s = 0; 	// variabile d'appoggio per incrementare il tempo
int			BrakeTime = 0;

/* ========================================================================================
 * CAN-BUS communication
 * ======================================================================================== */
CAN_TxHeaderTypeDef CAN_TxHeader0 	= {.StdId = 0x162, .IDE = CAN_ID_STD, .DLC = 8};
unsigned char  		CAN_TxMex0[8] 	= {0};

CAN_TxHeaderTypeDef CAN_TxHeader1 	= {.StdId = 0x163, .IDE = CAN_ID_STD, .DLC = 8};
unsigned char  		CAN_TxMex1[8] 	= {0};

CAN_TxHeaderTypeDef CAN_TxHeader2 	= {.StdId = 0x164, .IDE = CAN_ID_STD, .DLC = 8};
unsigned char  		CAN_TxMex2[8] 	= {0};

uint32_t			CAN_TxMailBoxN 	= 0x00000001U;
uint32_t 			CAN_RxMailBoxN;
uint32_t 			CAN_RxFifo0		= CAN_RX_FIFO0;
uint32_t 			CAN_RxFifo1		= CAN_RX_FIFO1;
CAN_RxHeaderTypeDef CAN_RxHeader0;
CAN_RxHeaderTypeDef CAN_RxHeader1;
uint8_t 			CAN_RxMex0[8]   = {0};
uint8_t 			CAN_RxMex1[8]   = {0};
CAN_FilterTypeDef   CAN_RxFilter0;
CAN_FilterTypeDef   CAN_RxFilter1;
HAL_StatusTypeDef   CAN_Mex0Available;
HAL_StatusTypeDef   CAN_Mex1Available;
HAL_StatusTypeDef 	CAN_WriteError;

struct CanStruct  CAN_MexIn = {.ActualTorque = 0, .ActualSpeed = 0, .DriveStatus = 0, .ComErrorCan = 0, .NewMexCan = 0}; //  to read

struct ControllerStruct ControlRef 	  = {.Torque = 0, .FW = 0, .RW = 0, .FootSwitch = 0, .SeatSwitch = 0, .Status = 0, .Time = 0, .ToggleBit = 0};
struct ControllerStruct ControlStatus = {.Torque = 0, .Torque_filt = 0, .Speed = 0, .Speed_filt = 0, .FW = 0, .RW = 0, .FootSwitch = 0, .SeatSwitch = 0, .Status = 0, .Time = 0, .Acc = 0, .Acc_filt = 0};
struct SysStatusStruct  SysStatus 	  = {.Speed = 0, .SpeedW = 0, .Pressure = 0, .SysPhase = 0 };

struct ABSControlStruct ABSControl = {.controlTorque = 0, .Fx=0, .Fx0 = 0, .FxFilt =0, .FxFilt0=0, .errFx = 0, .FxD = 0, .FxD_filt = 0, .DFDT=0, .DFDT_filt=0, .Fmax = 0, .Tmax = 0 , .k=0, .ks=0, .phase=1};

struct SpeedStruct controlSpeed ={.v = 0, .kp = 5.0, .ki = 1.0, .kd = 0, .force_max = 50, .errI = 0, .a_ref = 0, .d_v = 0 };

char ToggleBit = 1;
int ii = 0;
double v0 = 0; //velocità allo step precedente

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if(!HAL_CAN_GetRxMessage(&hcan1, CAN_RxFifo0, &CAN_RxHeader0, CAN_RxMex0)){
		CAN_MexIn.NewMexCan = 1;
	}
}
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */


	TIM3->CNT=0x0000;
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	/* ========================================================================================
	 * Enable TIM6 counter and interrupt
	 * ======================================================================================== */
	TIM7->CR1 	|= 0x1;								/* Enable TIM6 counter                    */
	TIM7->DIER 	|= 0x1;								/* Enable TIM6 interrupt                  */
	/* ========================================================================================
	 * set CAN
	 * ======================================================================================== */
	// CAN Filter config
	CAN_RxFilter0.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN_RxFilter0.FilterIdLow 		= 0x132<<5;
	//CAN_RxFilter0.FilterMaskIdLow  	= 0x142<<5;
	CAN_RxFilter0.FilterIdHigh 		= 0x142<<5;
	CAN_RxFilter0.FilterMode 		= CAN_FILTERMODE_IDLIST;
	CAN_RxFilter0.FilterScale 		= CAN_FILTERSCALE_16BIT;
	CAN_RxFilter0.FilterActivation   = ENABLE;
	CAN_RxFilter0.FilterBank 		= 0;
	HAL_CAN_ConfigFilter(&hcan1, &CAN_RxFilter0);

	HAL_CAN_Start(&hcan1);

	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	//HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (enable10ms == 1 ){
			enable10ms = 0;
			tempo1s++;
			if (tempo1s>9){
				tempo1s=0;
				tempo++;
			}

			READ_SENSORS(&SysStatus, 0.01);



			switch (SysPhase){
			case (SYS_START) :
				conta100++;
				if (conta100>9){
					conta100 = 0;
					HAL_GPIO_TogglePin(Led5_GPIO_Port,Led5_Pin);
				}
				if(tempo>10){
					HAL_GPIO_WritePin(AzKey_GPIO_Port,AzKey_Pin,SET);
				}
				ControlStatus.Time++;

				switch (ControlStatus.Status){
					case 1: // SeatSwitch
						HAL_GPIO_WritePin(Led1_GPIO_Port,Led1_Pin,SET);
						if (ControlStatus.Time>100){
							ControlRef.SeatSwitch = 1;
						}
						if ((ControlStatus.N) & (ControlStatus.Time>200)){
							ControlStatus.Status 	= 2;
							ControlStatus.Time 		= 0;
							HAL_GPIO_WritePin(Led1_GPIO_Port,Led1_Pin,RESET);
						}
						break;
					case 2: // FootSwitch
						ControlRef.FootSwitch = 1;
						HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,SET);

						if ((ControlStatus.N) & (ControlStatus.Time>100)){
							ControlStatus.Status 	= 3;
							ControlStatus.Time 		= 0;
							HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,RESET);
						}
						break;
					case 3: // Forward
						ControlRef.FW = 1;
						HAL_GPIO_WritePin(Led3_GPIO_Port,Led3_Pin,SET);

						if ((ControlStatus.FW) & (ControlStatus.Time>100)){
							ControlStatus.Status 	= 4;
							ControlStatus.Time   	= 0;
							HAL_GPIO_WritePin(Led3_GPIO_Port,Led3_Pin,RESET);
						}
						break;
					case 4: // init ends
						HAL_GPIO_WritePin(Led4_GPIO_Port,Led4_Pin,SET);
						if(HAL_GPIO_ReadPin(Button1_GPIO_Port,Button1_Pin) & (ControlStatus.Time>50)){
							SysPhase = SYS_ACC;
							ControlStatus.Time 		= 0;
							HAL_GPIO_WritePin(Led4_GPIO_Port,Led4_Pin,RESET);
						}
						break;
					default:
						HAL_GPIO_WritePin(Led1_GPIO_Port,Led1_Pin,RESET);
						HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,RESET);
						HAL_GPIO_WritePin(Led3_GPIO_Port,Led3_Pin,RESET);
						HAL_GPIO_WritePin(Led4_GPIO_Port,Led4_Pin,RESET);
						ControlRef.FootSwitch = 0;
						ControlRef.SeatSwitch = 0;
						ControlRef.FW = 0;
						ControlRef.RW = 0;
						break;
				} //chiude lo switch interno
				if (HAL_GPIO_ReadPin(Button1_GPIO_Port,Button1_Pin) & (ControlStatus.Status == 0)){
					ControlStatus.Status = 1;
				}
				break;

			case (SYS_ACC) :
				conta100++;

				if (conta100>10){
					conta100 = 0;
					HAL_GPIO_TogglePin(Led5_GPIO_Port,Led5_Pin);
				}


				if (ControlStatus.Speed_filt < 1){

					controlSpeed.enable = TRUE;

					controlSpeed.v_ref += 1;
					if (controlSpeed.v_ref > 2 ){
						controlSpeed.v_ref = 2;
					}

					controlSpeed.v = ControlStatus.Speed_filt;
					CtrlSpeed(&controlSpeed ,&ControlRef,0.01);

				}else{
					ControlRef.Torque += 0.2;
					if (ControlRef.Torque > 28 ){
						ControlRef.Torque = 28; // [N/m]
					}

					Controllo(&ControlStatus,&ABSControl,&ControlRef);
				}

				FILTER1Velo(&ControlStatus, 0.05, 0.01);
				FILTER1Torque(&ControlStatus, 0.05, 0.01);

				//controlSpeed.v = ControlStatus.Speed_filt;

				ControlStatus.Acc = (ControlStatus.Speed_filt - ControlStatus.Speed_filt0)/0.01; // Acc in m/s^2
				FILTER1Acc(&ControlStatus, 0.05, 0.01);

				//FILTER1Fx(&ABSControl, 3, 0.01);

				FILTER1(&ABSControl.DFDT, &ABSControl.DFDT_filt, 0.05, 0.01);
				FILTER1(&ABSControl.FxD, &ABSControl.FxD_filt, 0.075, 0.01);


				HAL_GPIO_WritePin(Led1_GPIO_Port,Led1_Pin,SET);

				ControlStatus.Time++;


				if ( ( HAL_GPIO_ReadPin(Button1_GPIO_Port,Button1_Pin) | ControlStatus.Speed_filt > 21 ) & (ControlStatus.Time>50)) {
					SysPhase = SYS_BRAKE;
					ControlRef.Torque = 0;
					ControlStatus.Time 		= 0;
					HAL_GPIO_WritePin(Led1_GPIO_Port,Led1_Pin,RESET);
					HAL_GPIO_WritePin(Led5_GPIO_Port,Led5_Pin,RESET);
				}

				break;

			case (SYS_BRAKE) :

				conta100++;
				if (conta100>10){
					conta100 = 0;
					HAL_GPIO_TogglePin(Led5_GPIO_Port,Led5_Pin);
				}

				BrakeTime++;

				if (BrakeTime < 60){
					ControlRef.Torque =0;

				}else{

				ControlRef.Torque -= 0.2;
				if (ControlRef.Torque < -28){
					ControlRef.Torque = -28;
				}

				Controllo(&ControlStatus,&ABSControl,&ControlRef);
				}

				ControlRef.Torque = 0;
				/*if (ControlStatus.Speed_filt < 2){
						SysPhase = SYS_ACC;
						ControlStatus.Time 		= 0;
						HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,RESET);
						HAL_GPIO_WritePin(Led5_GPIO_Port,Led5_Pin,RESET);
					}*/

					/*controlSpeed.v_ref -= 0.1;
					if (controlSpeed.v_ref <  0){
					 controlSpeed.v_ref = 0;
					}*/


				FILTER1Velo(&ControlStatus, 0.05, 0.01);
				FILTER1Torque(&ControlStatus, 0.05, 0.01);

				//controlSpeed.v = ControlStatus.Speed_filt;

				ControlStatus.Acc = (ControlStatus.Speed_filt - ControlStatus.Speed_filt0)/0.01; // Acc in m/s^2
				FILTER1Acc(&ControlStatus, 0.05, 0.01);

				//FILTER1Fx(&ABSControl, 3, 0.01);

				FILTER1(&ABSControl.DFDT, &ABSControl.DFDT_filt, 0.05, 0.01);
				FILTER1(&ABSControl.FxD, &ABSControl.FxD_filt, 0.075, 0.01);


				//controlSpeed.enable = TRUE;
				//CtrlSpeed(&controlSpeed,&ControlRef,0.01);

				HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,SET);

				ControlStatus.Time++;

				if (HAL_GPIO_ReadPin(Button1_GPIO_Port,Button1_Pin) & (ControlStatus.Time>50)) {
					SysPhase = SYS_ACC;
					ControlStatus.Time 		= 0;
					HAL_GPIO_WritePin(Led2_GPIO_Port,Led2_Pin,RESET);
					HAL_GPIO_WritePin(Led5_GPIO_Port,Led5_Pin,RESET);
				}

				break;

			case (SYS_DEBUG) :
				HAL_GPIO_WritePin(Led2_GPIO_Port, Led2_Pin,RESET);
				HAL_GPIO_TogglePin(Led3_GPIO_Port,Led3_Pin);
				break;
			default:
				break;

			} //Chiude switch esterno

			//CanWrite(&CAN_TxHeader0, CAN_TxMex0, &CAN_TxMailBoxN, &ControlRef);

			SysStatus.SysPhase = SysPhase;

			//CAN_WriteError = (CanWriteBack(&CAN_TxHeader1, CAN_TxMex1, &CAN_TxMailBoxN, &ControlStatus, &ControlRef, &SysStatus));

			CanWriteBack(&CAN_TxHeader1, &CAN_TxHeader2, CAN_TxMex1, CAN_TxMex2, &CAN_TxMailBoxN, &ControlStatus, &ControlRef, &SysStatus, &controlSpeed, &ABSControl);


			if (ControlStatus.Status != 0){
				CAN_WriteError = CanWrite(&CAN_TxHeader0, CAN_TxMex0, &CAN_TxMailBoxN, &ControlRef);

				if (CAN_WriteError){
					HAL_GPIO_WritePin(Led5_GPIO_Port,Led5_Pin,SET);
				}

				if (ControlStatus.Fault == 0x1D){
					ControlStatus.Status = 0;
				}

				CAN_MexIn.ComErrorCan ++;
				if (CAN_MexIn.ComErrorCan>9){
					HAL_GPIO_WritePin(Led5_GPIO_Port, Led5_Pin,SET);
					ControlStatus.Status = 0;
				}
			}

		} //IF contatore 10ms

		if (CAN_MexIn.NewMexCan){
			CanRead(&CAN_MexIn, &CAN_RxHeader0, CAN_RxMex0, &ControlStatus);
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
