/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef enum {
    uint8,
    uint16,
    float32
} DataType;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
void Lekk();
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*hadc);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_Temp_1_Pin GPIO_PIN_0
#define ADC_Temp_1_GPIO_Port GPIOA
#define ADC_Temp_2_Pin GPIO_PIN_1
#define ADC_Temp_2_GPIO_Port GPIOA
#define ADC_Temp_3_Pin GPIO_PIN_2
#define ADC_Temp_3_GPIO_Port GPIOA
#define ADC_Temp_4_Pin GPIO_PIN_3
#define ADC_Temp_4_GPIO_Port GPIOA
#define Lekasje_probe_1_Pin GPIO_PIN_11
#define Lekasje_probe_1_GPIO_Port GPIOB
#define Lekasje_probe_2_Pin GPIO_PIN_12
#define Lekasje_probe_2_GPIO_Port GPIOB
#define Lekasje_b_nd_Pin GPIO_PIN_13
#define Lekasje_b_nd_GPIO_Port GPIOB
#define Lekasje_probe_3_Pin GPIO_PIN_14
#define Lekasje_probe_3_GPIO_Port GPIOB
#define Test_Knapp_Pin GPIO_PIN_15
#define Test_Knapp_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
