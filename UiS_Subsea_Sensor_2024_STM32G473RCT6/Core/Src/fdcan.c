/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include <stdlib.h>
#include <string.h>

extern uint8_t dvl_cmd;

/* ----------CANFD---------- */
void CANFD_Init(void);
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
/* __________CANFD__________ */
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 44;
  hfdcan1.Init.NominalTimeSeg1 = 125;
  hfdcan1.Init.NominalTimeSeg2 = 44;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 12;
  hfdcan1.Init.DataTimeSeg1 = 21;
  hfdcan1.Init.DataTimeSeg2 = 12;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* --------------------CANFD-------------------- */
void CANFD_Init(void) {

	/* ID reception filter: Rx FIFO 0 */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	/*Id-er innenfor ID1 og ID2 slippes gjennom*/
	sFilterConfig.FilterID1 = Kommunikasjon_start;
	sFilterConfig.FilterID2 = Kommunikasjon_slutt;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK){
		Error_Handler();
	}

	/* ID reception filter: Rx FIFO 1 */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;

	/*Id-er innenfor ID1 og ID2 slippes gjennom*/
	sFilterConfig.FilterID1 = test_start;
	sFilterConfig.FilterID2 = test_slutt;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK){
		Error_Handler();
	}

	/*Configurerer det globale filteret*/
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK){
 		Error_Handler();
 	}

	/*Start CAN: Registere for initialisering av perifer og filter verdier låses*/
 	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
 		Error_Handler();
 	}

	/*Stop CAN: Registere åpnes for endring*/
 	//if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK){
 	//	Error_Handler();
 	//}

	/*Interrupt handler aktivering for innkommende meldinger*/
 	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
 		Error_Handler();
 	}
 	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
 	   Error_Handler();
 	}
	/* ____________________CANFD____________________ */
}

/*SendData Funksjon: Konverter og sender data på CANFD bussen
 *adr: 		Adressen til CANFD meldingen i uint16_t format
 *data: 	Liste av dataen som skal sendes i meldingen
 *size: 	Antall bytes "data" inneholder, uint8_t format
 *			Aksepterte verdier: 1-8, 12, 16, 20, 24, 32, 48, 64
 *datatype: Format "data" har
 *			Aksepterte verdier: uint8, uint16, float*/
int SendData(uint16_t adr, void* data, uint8_t size, DataType dataType) {
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) <= 0) {
        return 2;
    }
    /*CANFD melding Datalengde velges*/
	uint8_t data_L;
	switch(size){
	  case 1:
		  data_L = 0x01;
		  break;
	  case 2:
		  data_L = 0x02;
		  break;
	  case 3:
		  data_L = 0x03;
		  break;
	  case 4:	// 4 bytes	-> 4 uint8, 2 uint16, 1 float
		  data_L = 0x04;
		  break;
	  case 5:
		  data_L = 0x05;
		  break;
	  case 6:
		  data_L = 0x06;
		  break;
	  case 7:
		  data_L = 0x07;
		  break;
	  case 8:	// 8 bytes	-> 8 uint8, 4 uint16, 2 float
		  data_L = 0x08;
		  break;
	  case 12:	// 12 bytes	-> 3 float
		  data_L = 0x09;
		  break;
	  case 16:	// 16 bytes	-> 4 float
		  data_L = 0x0A;
		  break;
	  case 20:	// 20 bytes	-> 5 float
		  data_L = 0x0B;
		  break;
	  case 24:	// 24 bytes	-> 6 float
		  data_L = 0x0C;
		  break;
	  case 32:	// 32 bytes	-> 8 float
		  data_L = 0x0D;
		  break;
	  case 48:	// 48 bytes	-> 12 float
		  data_L = 0x0E;
		  break;
	  case 64:	// 64 bytes	-> 16 float
		  data_L = 0x0F;
		  break;
	  default:	// 0 bytes	-> 0 float
		  data_L = 0x00;
		  return 5;
		  break;
	}

	/*Header innstillinger:
	 * Identifier: Valgt Adresse til meldingen
	 * IdType: Bruker 11bit id, Ikke 29bit
	 * DataLength: 1-64 bytes
	 * ErrorStateIndicator: Sender ut Error meldinger på bussen
	 * BitrRateSwitch: Aktivert
	 * melding format: FDCAN
	 * TxEvent: Aktiverer buffer for å hente sendt data
	 * MessageMarker: Markør for melding (satt til adresse)*/
	TxHeader.Identifier = adr;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = data_L; // FDCAN_DLC_BYTES_1
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	TxHeader.MessageMarker = adr;

	/*Variable initialisering før "case"
	 * Lengde på buffer lister settes
	 * 8 bytes -> uint8 liste med 8 plasser
	 * 8 bytes -> uint16 liste med 4 plasser
	 * 8 bytes -> float liste med 2 plasser*/
	uint8_t* dataInHex;
	uint8_t buffer8[size];
	uint16_t buffer16[size/2];
	uint32_t bufferfloat[size/4];

	/*Velger riktig prossess etter hva dataType som formateres til hex*/
	switch (dataType){
		case uint8:
			/* uint8_t til Hex
			 * dataInHex: Lenge = antall bytes, lagrer dataen i big endian format*/
			memcpy(buffer8, data, size);
			dataInHex = malloc(size);
		    if (!dataInHex) {
		    	return 3;
		    }
			for (int i = 0; i < size; i++) {
			  dataInHex[i] = buffer8[i];
			}
			break;
		case uint16:
			// uint16_t til Hex
			memcpy(buffer16, data, size);

			dataInHex = malloc(size*2);
		    if (!dataInHex) {
		    	return 3;
		    }
			for (int i = 0; i < size; i++) {
				dataInHex[i*2 + 0] = (buffer16[i] >> 8) & 0xFF;
				dataInHex[i*2 + 1] =  buffer16[i] & 0xFF;
			}
			break;
		case float32:
			/* float til Hex
			 * bufferfloat: binær reprensentasjon av float listen,
			 * dataInHex: Bruker big endian*/
			memcpy(bufferfloat, data, size);
			dataInHex = malloc(size*4);
		    if (!dataInHex) {
		    	return 3;
		    }
			for (int i = 0; i < size; i++) {
			  dataInHex[i*4 + 0] = (bufferfloat[i] >> 24) & 0xFF;
			  dataInHex[i*4 + 1] = (bufferfloat[i] >> 16) & 0xFF;
			  dataInHex[i*4 + 2] = (bufferfloat[i] >> 8) & 0xFF;
			  dataInHex[i*4 + 3] = bufferfloat[i] & 0xFF;
			}
			break;
		default:
			return 4;
			break;
	}

	/*Legger til meldingen i Tx buffer, Returnerer 0 hvis error*/
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, dataInHex) != HAL_OK) {
         free(dataInHex);
         return 0;
     }
    /* Stopper Malloc-ed variabler fra å lekke minne og returnerer 1 (melding sendt)*/
	free(dataInHex);
	return 1;
}
/* HexToValues funksjon: CANFD buss hex data til c kode
 * voidData: 	verdilisten som dataen settes i etter konverteringen
 * hexData: 	Data som skal konverteres
 * sizeBytes: 	Lengde på meldingen i bytes, må samsvare med byte lengde til "voidData"
 * dataType: 	Velger datatypen "hexData" skal konverteres til, må samsvare med "voidData"
 * 				Aksepterte verdier: uint8, uint16, float*/
int hexToValues(void* voidData, const uint8_t* hexData, uint8_t sizeBytes, DataType dataType){
	/*Velger metode etter oppgitt dataType*/
	switch(dataType) {
		case uint8:
			/* uint8Data som peker på listen i "voidData" som får de konverterte verdiene
			 * Big endian brukes*/
			uint8_t* uint8Data = (uint8_t*)voidData;
		    for (uint8_t i = 0; i < sizeBytes; i++) {
		    	uint8Data[i] = hexData[i];
		    }
			break;
		case uint16:
			/* Kjekker om byte størrelsen går opp: 7%2=1 -> returnerer 2
			 * uint16Data som peker på listen i "voidData" som får de konverterte verdiene
			 * Big endian brukes*/
			if (sizeBytes % 2 != 0) return 2;
			uint16_t* uint16Data = (uint16_t*)voidData;
		    uint8_t uint16Nr = sizeBytes / 2;
		    for (uint8_t i = 0; i < uint16Nr; i++) {
		    	uint8_t bytes[2];
		    	bytes[0] = hexData[i * 2 + 1];
		    	bytes[1] = hexData[i * 2 + 0];

		    	uint16_t uint16Value;
		    	memcpy(&uint16Value, bytes, 2);
		    	uint16Data[i] = uint16Value;
		    }
			break;
		case float32:
			/* Byte størrelsen sjekk: 6%4=2 -> returnerer 3
			 * floatData som peker på listen i "voidData" som får de konverterte verdiene
			 * Big endian brukes */
			if (sizeBytes % 4 != 0) return 3;
			float* floatData = (float*)voidData;
		    uint8_t floatNr = sizeBytes / 4;
		    for (uint8_t i = 0; i < floatNr; i++) {
		        uint8_t bytes[4];
		        bytes[0] = hexData[i * 4 + 3];
		        bytes[1] = hexData[i * 4 + 2];
		        bytes[2] = hexData[i * 4 + 1];
		        bytes[3] = hexData[i * 4];
		        float floatValue;
		        memcpy(&floatValue, bytes, 4);
		        floatData[i] = floatValue;
		    }
			break;
		default:
			return 0;
			break;
	}
	/*Ved fullført konvertering returner 1*/
	return 1;
}

/* USER CODE END 1 */
