/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fdcan.h"
#include "glob_var.h"
#include "usart.h"
#include "MS5837-30BA.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t pressure_ready = 0;
float watertemp = 0;
float waterpressure = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */
extern struct m_wrz wrz;
extern struct m_wru wru;
extern struct m_wrp wrp;
extern uint8_t send_msg;
extern uint8_t new_uart;
extern uint8_t dvl_mask;
extern uint8_t dvl_mask2;
extern uint8_t dvl_cmd;
extern uint8_t send_dvl;
extern char rx_buffer[MAX_AMOUNT_BUFFERS][RX_BUFFER_SIZE];
extern char DVL_Parced[MAX_AMOUNT_BUFFERS][MAX_NUM_WORDS][MAX_WORD_LENGTH];
extern ms5837_t pressuresensor;

extern uint8_t lekkasje;
extern uint8_t temp;

uint8_t temp1,temp2,temp3,vanntemp,trykk,dvl_cmd = 0;


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  static uint8_t ms1_ticker, ms10_ticker, ms100_ticker = 0;

  //Skjer hvert 10ms
  if(ms1_ticker++>9){//if_ms			/* Teller til opp til 5 slik at det går 5ms for hver gang Lekk() kjører */
	  ms1_ticker = 0;
	  Lekk();

	  if(send_dvl){//if_dvl
			  UART1_update(dvl_cmd);
			  send_dvl=0;

			  Process_msg();
			  while (send_msg > 0){//_while
//			  	  	Hastighet
			  	  	if (send_msg & 0x01){//_if
			  	  			  float Vx = atof(wrz.vx);
			  	  			  float Vy = atof(wrz.vy);
			  	  			  float Vz = atof(wrz.vz);
			  	  			  float altitude = atof(wrz.altitude);
			  	  			  float V_fom = atof(wrz.merit_figure);
			  	  			  float Tid_siste = atof(wrz.time_since_last_rep);
			  	  			  float poisition_msg[] = {Vx,Vy,Vz,altitude,V_fom, Tid_siste};

			  	  			  SendData((Sensor_start + 0x01), poisition_msg, sizeof(poisition_msg), float32);
			  	  			  send_msg &= 0xFE; //Nullstill respektiv bit i maske

			  	  	}//_if
			  	  		  //Posisjon
			  	  	else if (send_msg & 0x04){//_if_else
			  	  			  float time_stamp = atof(wrp.time_stamp);
			  	  			  float Px = atof(wrp.px);
			  	  			  float Py = atof(wrp.py);
			  	  			  float Pz = atof(wrp.pz);
			  	  			  float Roll = atof(wrp.roll);
			  	  			  float Pitch = atof(wrp.pitch);
			  	  			  float Yaw = atof(wrp.yaw);
			  	  			  float P_fom = atof(wrp.pos_std);

			  	  			  float heading_msg[] = {Px,Py,Pz,Roll,Pitch,Yaw,P_fom,time_stamp};
			  	  			  SendData(Sensor_start, heading_msg, sizeof(heading_msg), float32);
			  	  			  send_msg &= 0xFB; //Nullstill respektiv bit i maske

			  	  	}//_if_else
			  	  		  //Temp & Trykk
			  	  	else if (send_msg & 0x80){//_if_else
			  	  			  uint8_t tp_msg[] = {temp1,temp2,temp3,vanntemp,trykk};
			  	  			  SendData((Sensor_start + 0x02), tp_msg, sizeof(tp_msg), uint8);
			  	  			  send_msg &= 0x7F;
			  	  	}//_if_else

			  	  		  //Feilmelding og "Catch-all"
			  	  	else if (send_msg){//_if_else
			  	  			  uint8_t error_msg[] = {dvl_mask,dvl_mask2,lekkasje};
			  	  			  SendData((Sensor_start + 0x03), error_msg, sizeof(error_msg), uint8);
			  	  			  send_msg &= 0x85; //Nullstill alle bit som ikke stemmer med andre masker
			  	  	}//_if_else
			  }//_while
	  }//if_dvl
	  				  ms10_ticker++;


	//-------------------Skjer hvert 100ms
					  if (ms10_ticker++ > 99){//_100ms
						  ms10_ticker = 0;

//						  if (pressure_ready++>0){
//								 //Hent Trykk
//								 ms5837_read_conversion(&pressuresensor);
//								 ms5837_start_conversion(&pressuresensor, SENSOR_PRESSURE, OSR_4096);
//					  	  }
//					  		else{
//
//								 //Hent Vanntemp
//								 pressure_ready = 0;
//								 ms5837_read_conversion(&pressuresensor);
//								 ms5837_calculate(&pressuresensor);
//								 ms5837_start_conversion(&pressuresensor, SENSOR_PRESSURE, OSR_4096);
//								 watertemp = ms5837_temperature_celcius(&pressuresensor);
//								 waterpressure = ms5837_pressure_bar(&pressuresensor);
//						  }

	//----------------Skjer hvert 1sek
						  if(ms100_ticker++>9){ //_1s
							uint8_t status_msg[] = {0x00,0x00,lekkasje};
							ms100_ticker = 0;
							SendData((Sensor_start + 0x03), status_msg, sizeof(status_msg), uint8);
						  } //_1s


					  }//_100ms

  }//if_ms

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 1.
  */
void FDCAN1_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */

  /* USER CODE END FDCAN1_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 1 */

  /* USER CODE END FDCAN1_IT1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

	  if (LL_USART_IsActiveFlag_RXNE(USART1)) {
	      // Les motatt byte og initaliser variabler
	      char rx_data = LL_USART_ReceiveData8(USART1);
	      static uint8_t rx_buffer_nr = 0, rx_index = 0;
	      // Sjekk om ny byte er Line feed eller Carriage return
	      if ((rx_data == '\r') || (rx_data == '\n')) {
	    	  // Avslutt streng med \0 streng terminering
	    	  rx_buffer[rx_buffer_nr++][rx_index] = '\0';
	          // Reset buffer index for neste melding
	          rx_index = 0;
	          new_uart++;
	          if (new_uart>10){
	            new_uart=10;
	            }
	          if(rx_buffer_nr>=MAX_AMOUNT_BUFFERS){
	             rx_buffer_nr = 0;
	            }

	        }
	        else {
	            // Lagre motatt byte i buffer
	            rx_buffer[rx_buffer_nr][rx_index++] = rx_data;
	        	}
	            // Se etter buffer overflow
	        if (rx_index >= RX_BUFFER_SIZE) {
	            // Reset buffer index
	            rx_index = 0;
	        }


	  }

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles I2C3 event interrupt / I2C3 wake-up interrupt through EXTI line 27.
  */
void I2C3_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_EV_IRQn 0 */

  /* USER CODE END I2C3_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_EV_IRQn 1 */

  /* USER CODE END I2C3_EV_IRQn 1 */
}

/* USER CODE BEGIN 1 */
extern FDCAN_RxHeaderTypeDef RxHeader;

uint16_t idValue;
uint8_t RxData[64];

// Definering av verdilister som hentes fra CANFD nettverk
uint8_t testV[3];
uint8_t ping[7];
uint8_t CAN_buffer[6];

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	// Går gjennom alle meldinger i fifo bufferen
	while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);

		idValue = (uint16_t)(RxHeader.Identifier & 0xFFF);
	    switch (idValue) {
	        case Lytting:
	        	hexToValues(ping, RxData, sizeof(ping), uint8);
	        	ping[1] = ping[1] +1;
	        	SendData(Sensor_slutt, ping, sizeof(ping), uint8);
	            break;
	        case Kommunikasjon_start:
	        	hexToValues(CAN_buffer, RxData, sizeof(CAN_buffer), uint8);
	        	dvl_cmd = CAN_buffer[0];
	        	send_dvl = 1;
				break;
	        default:
	            break;
	    }
	}
}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	// Går gjennom alle meldinger i fifo bufferen
	while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1) > 0) {
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData);

		idValue = (uint16_t)(RxHeader.Identifier & 0xFFF);
	    switch (idValue) {
	    	case 1:
	    		break;
	    	default:
	            break;
	    }
	}
}




/* USER CODE END 1 */
