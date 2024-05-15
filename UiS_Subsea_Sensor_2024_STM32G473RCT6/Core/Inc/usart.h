/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define SRC_GLOB_VAR_H_

#define RX_BUFFER_SIZE 128
#define MAX_NUM_WORDS 11
#define MAX_WORD_LENGTH 128
#define MAX_AMOUNT_BUFFERS 5


struct m_wrz{
	  char vx[MAX_WORD_LENGTH];
	  char vy[MAX_WORD_LENGTH];
	  char vz[MAX_WORD_LENGTH];
	  char valid[MAX_WORD_LENGTH];
	  char altitude[MAX_WORD_LENGTH];
	  char merit_figure[MAX_WORD_LENGTH];
	  char covariance[MAX_WORD_LENGTH];
	  char time_of_reflection[MAX_WORD_LENGTH];
	  char time_of_transmission[MAX_WORD_LENGTH];
	  char time_since_last_rep[MAX_WORD_LENGTH];
	  char status[MAX_WORD_LENGTH];
};

struct m_wrp{
	  char time_stamp[MAX_WORD_LENGTH];
	  char px[MAX_WORD_LENGTH];
	  char py[MAX_WORD_LENGTH];
	  char pz[MAX_WORD_LENGTH];
	  char pos_std[MAX_WORD_LENGTH];
	  char roll[MAX_WORD_LENGTH];
	  char pitch[MAX_WORD_LENGTH];
	  char yaw[MAX_WORD_LENGTH];
	  char status[MAX_WORD_LENGTH];
};

struct m_wru{
	  char id[3][MAX_WORD_LENGTH];
	  char velocity[3][MAX_WORD_LENGTH];
	  char distance[3][MAX_WORD_LENGTH];
	  char rssi[3][MAX_WORD_LENGTH];
	  char nsd[3][MAX_WORD_LENGTH];
};
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

//#define RX_BUFFER_SIZE 128
//#define MAX_AMOUNT_BUFFERS 5

//typedef struct {
//	  char vx;
//	  char vy;
//	  char vz;
//	  char valid;
//	  char altitude;
//	  char merit_figure;
//	  char covariance;
//	  char time_of_reflection;
//	  char time_of_transmission;
//	  char time_since_last_rep;
//	  char status;
//}msg_wrz;
//
//typedef struct {
//	  char time_stamp;
//	  char px;
//	  char py;
//	  char pz;
//	  char pos_std;
//	  char roll;
//	  char pitch;
//	  char yaw;
//	  char status;
//}msg_wrp;
//
//typedef struct {
//	  char id[3];
//	  char velocity[3];
//	  char distance[3];
//	  char rssi[3];
//	  char nsd[3];
//}msg_wru;



/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void Parce_msg(void);
void Process_msg(void);
void UART1_update(uint8_t maske);
void UART1_sendcmd(const char *string);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

