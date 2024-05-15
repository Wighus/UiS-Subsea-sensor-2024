/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
// 0x0 til 0x7FF (Max ID mulig) -> 2047 Id-er, (11 bits)
// 0x0 til 0x03F -> 64 Id-er
enum {
  Kommunikasjon_start = 0x000,
  Lytting = Kommunikasjon_start + 0x009,
  Kommunikasjon_slutt = 0x03F,
  Regulering_start = 0x040,
  Regulering_slutt = 0x07F,
  Sensor_start = 0x080,
  Sensor_slutt = 0x0BF,
  Manipulator_start = 0x0C0,
  Manipulator_slutt = 0x0FF,
  Kraft_start = 0x100,
  Kraft_slutt = 0x13F,

  // For testing
  test_start = 0x200,
  test_slutt = 0x2FF
};


/*Id Liste for verdier som skal sendes ut til CANBussen*/
enum {
  test = Kommunikasjon_start

};
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CANFD_Init(void);
int SendData(uint16_t adr, void* data, uint8_t size, DataType dataType);
int hexToValues(void* voidData, const uint8_t* hexData, uint8_t sizeBytes, DataType dataType);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

