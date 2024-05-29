/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "glob_var.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern struct m_wrz wrz;
extern struct m_wru wru;
extern struct m_wrp wrp;
extern uint8_t send_msg;
extern uint8_t new_uart;
extern uint8_t dvl_mask;
extern uint8_t dvl_mask2;
extern char rx_buffer[MAX_AMOUNT_BUFFERS][RX_BUFFER_SIZE];
extern char DVL_Parced[MAX_AMOUNT_BUFFERS][MAX_NUM_WORDS][MAX_WORD_LENGTH];

/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */


  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**USART1 GPIO Configuration
  PC4   ------> USART1_TX
  PC5   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* USER CODE BEGIN 1 */
void Parse_msg(void){
	// Initialiserer indexer
	static uint8_t read_index = 0;
	uint8_t write_index = 0;
	// Definerer hvor strenger skal spittes
	const char delim[] = ",";
	// Henter streng fra buffer
	char* str= rx_buffer[read_index];
	//
	// Splitter streng
	char* token = strtok(str,delim);
	// For alle ord i streng
	while(token != NULL){
		strcpy(DVL_Parced[read_index][write_index++], token);
		token = strtok(NULL,delim);
		if (write_index>MAX_WORD_LENGTH){
			return;
		}
	}
	// Inkrementerer og sjekker for overflow
	read_index++;
	if (read_index>=MAX_AMOUNT_BUFFERS){
		read_index = 0;
	}
}

void Process_msg(void){
	// Index for kontroll av avlest melding
	static int process_index = 0;
	// Sjekk om ny melding er splittet
	if (new_uart > 0){
		while (new_uart-- >=1){
			Parse_msg();
			int word=0;
			/* Alle motatte meldinger starter med W(waterlinked) og R(Response)
			vi trenger derfor bare se på det tredje tegnet for å finne ut hvilken meldingstype det er
			send_msg er en bitmaske som beskriver hvilke meldinger som er blitt oppdatert*/

			switch (DVL_Parced[process_index][0][2]) {
			  case 'z': // Velocity report wrz

				  strcpy(wrz.vx,DVL_Parced[process_index][1]);
				  strcpy(wrz.vy,DVL_Parced[process_index][2]);
				  strcpy(wrz.valid,DVL_Parced[process_index][3]);
				  strcpy(wrz.altitude,DVL_Parced[process_index][4]);
				  strcpy(wrz.merit_figure,DVL_Parced[process_index][5]);
				  strcpy(wrz.covariance,DVL_Parced[process_index][6]);
				  strcpy(wrz.time_of_reflection,DVL_Parced[process_index][7]);
				  strcpy(wrz.time_since_last_rep,DVL_Parced[process_index][9]);
				  strcpy(wrz.status,DVL_Parced[process_index][10]);

				  send_msg |= 0x1;

				break;
			  case 'u':   // Transducer report wru
				  // Finner ut hvilken transducer rapporten gjelder og lagrer data i struct
				  uint8_t tra_num = DVL_Parced[process_index][1][0] - '0';
				  strcpy(wru.velocity[tra_num],DVL_Parced[process_index][1]);
				  strcpy(wru.distance[tra_num],DVL_Parced[process_index][2]);
				  strcpy(wru.rssi[tra_num],DVL_Parced[process_index][3]);
				  strcpy(wru.nsd[tra_num],DVL_Parced[process_index][4]);

				  send_msg |= 0x2;
				break;
			  case 'p':   // Dead reckogning report wrp
				  strcpy(wrp.time_stamp,DVL_Parced[process_index][1]);
				  strcpy(wrp.px,DVL_Parced[process_index][2]);
				  strcpy(wrp.py,DVL_Parced[process_index][3]);
				  strcpy(wrp.pz,DVL_Parced[process_index][4]);
				  strcpy(wrp.pos_std,DVL_Parced[process_index][5]);
				  strcpy(wrp.roll,DVL_Parced[process_index][6]);
				  strcpy(wrp.pitch,DVL_Parced[process_index][7]);
				  strcpy(wrp.yaw,DVL_Parced[process_index][8]);
				  strcpy(wrp.status,DVL_Parced[process_index][9]);

				  send_msg |= 0x4;
				break;
			  case '?':   // Malformed request, Request failed Unable to read request wr?

				  send_msg |= 0x8;
				break;
			  case '!':   // Malformed request, Request failed, Checksum error wr!

				  send_msg |= 0x10;

				break;
			  case 'a':   // Command recived and successfully applied wra
				  send_msg |= 0x20;

				break;
			  case 'n':   // Command recived but failed to apply wrn

				  send_msg |= 0x40;

				break;
			  default:  // Feil i mottak av melding

			if (process_index++>MAX_AMOUNT_BUFFERS){//Overflow
				process_index=0;
				}
			}
		}
	}
}

void UART1_update(uint8_t cmd){
	static uint8_t current_config = 0x01;
	if (cmd&0x04){
		char cal_gyro[] = "wrg";
		UART1_sendcmd(cal_gyro);
		}
	if (cmd&0x08){
		char reset_dr[]= "wcr";
			UART1_sendcmd(reset_dr);
	}

	//[][][][fresh_water][Reset_DR][Calibrate_gyro][Dark_mode_enable],[Acoustic_enable]
	//wcs = wrc,[speed_of_sound],[mounting_rotation_offset],[acoustic_enabled],[dark_mode_enabled],[range_mode],[periodic_cycling_enabled]
	current_config = (cmd & 0x13);
	switch (current_config & 0x13){
		case 0b00000000:
			const char fresh_nn[] = "wcs,1481,,n,n,,";
			UART1_sendcmd(fresh_nn);
		case 0b00000001:
			const char fresh_yn[] = "wcs,1481,,y,n,,";
			UART1_sendcmd(fresh_yn);
		case 0b00000010:
			const char fresh_ny[] = "wcs,1481,,n,y,,";
			UART1_sendcmd(fresh_ny);
		case 0b00000011:
			const char fresh_yy[] = "wcs,1481,,y,y,,";
			UART1_sendcmd(fresh_yy);
		case 0b0001000:
			const char salt_nn[] = "wcs,1570,,n,n,,";
			UART1_sendcmd(salt_nn);
		case 0b00010001:
			const char salt_yn[] = "wcs,1570,,y,n,,";
			UART1_sendcmd(salt_yn);
		case 0b00010010:
			const char salt_ny[] = "wcs,1570,,n,y,,";
			UART1_sendcmd(salt_ny);
		case 0b00010011:
			const char salt_yy[] = "wcs,1570,,y,y,,";
			UART1_sendcmd(salt_yy);
		default:
			break;
	}


}

void UART1_sendcmd(const char *string){
	while(!LL_USART_IsActiveFlag_TXE(USART1)) {

	    }
	    while(*string != '\0') {
	        // Send tegn i strengen over UART1
	        LL_USART_TransmitData8(USART1, *string);

	        // Vent til sending av tegnet er ferdig
	        while(!LL_USART_IsActiveFlag_TC(USART1)) {
	        }

	        // Gå til neste tegn i strengen
	        string++;
	    }
	    // Send carrige return ('\r') når hele strengen er sendt
	        LL_USART_TransmitData8(USART1, '\r');

	        // Vent til hele sendingen er fullført
	        while(!LL_USART_IsActiveFlag_TC(USART1)) {

	        }
		// Send line feed ('\n') når hele strengen er sendt
		    LL_USART_TransmitData8(USART1, '\n');

		    // Vent til hele sendingen er fullført
		    while(!LL_USART_IsActiveFlag_TC(USART1)) {

		    }
	}



/* USER CODE END 1 */
