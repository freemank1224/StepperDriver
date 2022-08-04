/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//extern int16_t dutyLED;
//extern int16_t dutyCmd;
//extern uint16_t testAngle;
//extern int32_t filteredAngle;
//extern float CAL_stepSize;
//extern uint16_t amplitude;
//extern int16_t dirCmd;
//extern int16_t stepNumber;
//extern uint16_t counterTIM1_5k;
//extern uint16_t counterTIM1_1k;
//extern uint16_t counterTIM2_20k;
//extern uint16_t counterTIM2_1k;
//extern uint16_t counterTIM2_5k;
//extern uint16_t counterTIM2_speed;
//
//extern int16_t CAL_closedLoop;
//extern int16_t CAL_enableMotor;
////extern int16_t CAL_dirCmd;
//extern int16_t CAL_calibrateEncoder;


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//uint16_t readAngle(void);
//void calibrateEncoder(void);
//void moveMotor(int32_t theta, uint16_t effort);
//void fullStep(void);
//
//void flashUnlock(void);
//void flashLock(void);
//uint8_t flashGetStatus(void);
//uint8_t flashWaitDone(uint16_t time);
//void flashErase32K(void);
//uint8_t flashWriteHalfWord(uint32_t faddr, uint16_t data);
//uint16_t flashReadHalfWord(uint32_t faddr);




/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NSS_Pin LL_GPIO_PIN_3
#define NSS_GPIO_Port GPIOA
#define LED_PB0_Pin LL_GPIO_PIN_0
#define LED_PB0_GPIO_Port GPIOB
#define LED_PB1_Pin LL_GPIO_PIN_1
#define LED_PB1_GPIO_Port GPIOB
#define LED1_Pin LL_GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define PWM1_Pin LL_GPIO_PIN_4
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin LL_GPIO_PIN_5
#define PWM2_GPIO_Port GPIOB
#define INB2_Pin LL_GPIO_PIN_6
#define INB2_GPIO_Port GPIOB
#define INB1_Pin LL_GPIO_PIN_7
#define INB1_GPIO_Port GPIOB
#define INA1_Pin LL_GPIO_PIN_8
#define INA1_GPIO_Port GPIOB
#define INA2_Pin LL_GPIO_PIN_9
#define INA2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
