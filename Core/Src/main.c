/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "Variables.h"
#include "EncoderOperation.h"
#include "MotorOperation.h"
#include "UsDelay.h"
#include <stdio.h>

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


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Freeman for SWO DEBUG
//int _write(int file , char *ptr, int len)
//{
//    int i = 0;
//    for(i = 0; i<len; i++)
//        ITM_SendChar((*ptr++));
//    return len;
//}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t tempStepNumber = 0;

  uint16_t arrayARR[] = {8000,4000,2000,1000,500,250};
  uint8_t idxARR = 0;




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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /*-------------- Freeman Initialization Fcns ----------------*/
  // Enable channel & counter
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);

  LL_TIM_EnableCounter(TIM3);

  // Freeman added in 20220630 Enable TIM1
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_EnableCounter(TIM1);

  // Enable TIM2
  LL_TIM_EnableIT_UPDATE(TIM2);
  LL_TIM_EnableCounter(TIM2);

  // Enable UART
  LL_USART_EnableIT_RXNE(USART1);
//  LL_USART_EnableIT_IDLE(USART1);
  LL_USART_EnableIT_RXNE(USART3);
//  LL_USART_EnableIT_IDLE(USART3);
  LL_USART_EnableIT_IDLE(USART3);

  // 485 communication control: RE#DE pin LOW: RECEIVE / HIGH: TRANSMIT
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);

  // SPI1 init
  LL_SPI_Enable(SPI1);

  /*========Encoder Calibration Routine =======*/
  if(parameterTuningEnable != 0){
	  if(CAL_calibrateEncoder == 1)
	    {
	  	  LL_TIM_DisableIT_UPDATE(TIM1);
	  	  LL_TIM_DisableCounter(TIM1);
	  	  LL_TIM_DisableIT_UPDATE(TIM2);
	  	  LL_TIM_DisableCounter(TIM2);

	  	  calibrateEncoder();

	  	  CAL_calibrateEncoder = 0;

	  	  LL_TIM_EnableIT_UPDATE(TIM1);
	  	  LL_TIM_EnableIT_UPDATE(TIM2);
	  	  LL_TIM_EnableCounter(TIM1);
	  	  LL_TIM_EnableCounter(TIM2);
	    }
  }

  /*========= EOF Calibration Routine =========*/

  /*========= PID Init ============*/
  if(parameterTuningEnable != 0){
	  initPID(&pidSpeed, recKp, recKi, recKd, recFilterEnableFlag, recFilterCoef, 10, 80);
	  initPID(&pidEffort, recKp, recKi, recKd, recFilterEnableFlag, recFilterCoef, -50, 50);
  }else{
	  // For Manual Tuning in Cube IDE
	  initPID(&pidSpeed, 1, 10, 0, 0, 0, -100, 100);
	  initPID(&pidEffort, 1, 80, 0, 0, 0, -80, 80);
  }


  /*========= EOF PID Init ============*/

  /*-------------- EOF Freeman Initialization Fcns ---------------*/
  moveMotor(0, 50);
  commandOut = 40;
  roundNumber = 0;
  filteredAngle = 0;
  fullAngle = 0;
  fullAngle_k_1 = 0;

  angularSpeedCmdRPM = 1000;	// RPM unit

  testARR = 8000;
  LL_TIM_SetAutoReload(TIM2, testARR);
  LL_TIM_EnableARRPreload(TIM2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // 20kHz TIM1.... 1ms Interrupt to read encoder and calculate speed
	  if(counterTIM1_5k > 19)
	  {
		  counterTIM1_5k = 0;

		  filteredAngle = encoderFilter(5);

		  if(filteredAngle - filteredAngle_k_1 > 8192)	roundNumber --;
		  if(filteredAngle - filteredAngle_k_1 < -8192)	roundNumber ++;

		  fullAngle = 16384 * roundNumber + filteredAngle;

		  deltaAngle = fullAngle - fullAngle_k_1;
		  // Convert to ticks per second
		  angularSpeedTicks = deltaAngle * (HAL_RCC_GetSysClockFreq() / LL_TIM_GetPrescaler(TIM1) / LL_TIM_GetAutoReload(TIM1) / 20);
		  // Convert to RPM
		  angularSpeedRPM = ticks2rpm(angularSpeedTicks);

		  deltaARR = controlPID(&pidSpeed, angularSpeedCmdRPM, angularSpeedRPM);

		  testARR -= deltaARR;
		  if(deltaARR > 2000) deltaARR = 1000;
		  if(deltaARR < -2000) deltaARR = -1000;

		  if(testARR < arrayARR[5]) testARR = arrayARR[5];
		  if(testARR > arrayARR[0]) testARR = arrayARR[0];

		  LL_TIM_SetAutoReload(TIM2, testARR);
		  LL_TIM_EnableARRPreload(TIM2);


		  // Update history value
		  fullAngle_k_1 = fullAngle;
		  angularSpeedRPM_k_1 = angularSpeedRPM;

	  }

	  // 5k Interrupt to call speed regulation algorithm
	  if(counterTIM1_1k > 3)
	  {
		  counterTIM1_1k = 0;

//		  flashAngle = * (volatile uint16_t*)((readAngle()>>2)*2 + 0x08008000);




		  if(parameterTuningEnable != 0){
			  commandAngle = recCommandAngle;
		  }else{
			  // Manual Control
			  commandAngle = commandAngle;
		  }

		  errorOut = commandAngle - fullAngle;

		  if(recStepSizeIdx == 0) CAL_stepSize = 81.92f;
		  if(recStepSizeIdx == 1) CAL_stepSize = 40.96f;
		  if(recStepSizeIdx == 2) CAL_stepSize = 10.24f;
		  if(recStepSizeIdx == 3) CAL_stepSize = 5.12f;
		  if(recStepSizeIdx == 4) CAL_stepSize = 2.56f;
	  }

	  // 10kHz
	  if(counterTIM2_5k > 0)	// 500Hz
	  {
		  counterTIM2_5k = 0;
		  tempStepNumber ++;

		  commandOut = controlPID(&pidEffort, commandAngle, fullAngle);

		  moveMotor(5.12f * tempStepNumber, 40);
		  LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

//		  if(CAL_releaseMotor == 0){
//
//			  if(commandOut >= 0){
//				  tempStepNumber --;
////				  if(commandOut < 2) commandOut = 2;
//	//			  if(commandOut > 50) commandOut = 50;
//				  if(parameterTuningEnable != 0){
//					  moveMotor(CAL_stepSize * stepNumber, commandOut);
//				  }else{
//					  moveMotor(5.12f * stepNumber, commandOut);
//				  }
//				  LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
//			  }
//			  if(commandOut < 0){
//				  tempStepNumber ++;
////				  if(commandOut > -2) commandOut = -2;
//	//			  if(commandOut < -50) commandOut = -50;
//				  if(parameterTuningEnable != 0){
//					  moveMotor(CAL_stepSize * stepNumber, -commandOut);
//				  }else{
//					  moveMotor(5.12f * stepNumber, -commandOut);
//				  }
//				  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
//			  }
//		  }

		  // Unload ALL Control Effect, Let the Motor Free Running
		  if(CAL_releaseMotor != 0) moveMotor(0, 0);




	  }

	  // 2000 for 0.1 sec interrupt
	  if(counterTIM2_1k > 39999)
	  {
		  counterTIM2_1k = 0;
//		  testAngle = readAngle();
		  printf("======");
		  printf("Current ARR value is: %d\n", testARR);

//		  LL_GPIO_TogglePin(LED_PB0_GPIO_Port, LED_PB0_Pin);
//		  LL_GPIO_TogglePin(LED_PB1_GPIO_Port, LED_PB1_Pin);

//		  LL_TIM_EnableARRPreload(TIM2);
//		  LL_TIM_SetAutoReload(TIM2, testARR);
//		  LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
		  while(!LL_GPIO_IsOutputPinSet(GPIOB, LL_GPIO_PIN_12));

		  while(!LL_USART_IsActiveFlag_TXE(USART3));
		  LL_USART_TransmitData8(USART3, 0x07);
		  //uDelay(100);
		  while(!LL_USART_IsActiveFlag_TXE(USART3));
		  LL_USART_TransmitData8(USART3, 0x06);
		  while(!LL_USART_IsActiveFlag_TXE(USART3));
		  LL_USART_TransmitData8(USART3, 0x06);

		  // Data 1
		  for(int8_t i=3; i>=0; i--){
			  while(!LL_USART_IsActiveFlag_TXE(USART3));
			  LL_USART_TransmitData8(USART3, (fullAngle >> (i * 8)) & 0xFF);
		  }

		  // Data 2
		  for(int8_t j=3; j>-1; --j){
			  while(!LL_USART_IsActiveFlag_TXE(USART3));
			  LL_USART_TransmitData8(USART3, (commandOut >> (j * 8)) & 0xFF);
		  }

		  // Data 3
		  for(int8_t k=3; k>-1; --k){
			  while(!LL_USART_IsActiveFlag_TXE(USART3));
			  LL_USART_TransmitData8(USART3, (errorOut >> (k * 8)) & 0xFF);
		  }

		  while(!LL_USART_IsActiveFlag_TXE(USART3));
		  // Extra byte NEEDED to protect data integrity
		  LL_USART_TransmitData8(USART3, 0x09);
		  while(!LL_USART_IsActiveFlag_TXE(USART3));

		  // Pull DOWN to wait for receiving
		  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);
	  }


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __disable_irq();
  while (1)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
