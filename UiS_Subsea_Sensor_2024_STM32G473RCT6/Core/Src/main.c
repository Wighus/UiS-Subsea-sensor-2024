/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "cordic.h"
#include "crc.h"
#include "dma.h"
#include "fdcan.h"
#include "fmac.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "MS5837-30BA.h"
//#include "arm_math.h"
#include "EMA.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define numChannels 4
#define __VREFANALOG_VOLTAGE_ 3300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//-----------------lekkasje------------
uint8_t lekkasje = 0;

//-----------------Temp----------------
EMA ema_filter;

uint8_t convCompleted = 0;


//uint16_t temp1 = 0;
//uint16_t temp2 = 0;
//uint16_t temp3 = 0;
//uint16_t temp4 = 0;

uint16_t tempBuf[numChannels];
uint16_t filtrertTemp[numChannels];

//-----------------Trykk---------------------

//static const uint8_t pressure_sensor_adress = 0x76 << 1;//7Bit adresse plus les/skriv bit
//static const uint8_t reset_pressure_sens = 0x1E;
//static const uint8_t press_sens_temp_reg = 0x40;
//static const uint8_t press_sens_pres_reg = 0x42;
//static const uint8_t pressuresens_read = 0x00;

//-----------------DVL-----------------------
struct m_wrz wrz;
struct m_wru wru;
struct m_wrp wrp;
uint8_t send_msg = 0;
uint8_t send_dvl = 0;
uint8_t new_uart = 0;
uint8_t dvl_mask = 0;
uint8_t dvl_mask2 = 0;
char dvl_conf[] = "wcs,1481,,y,n,,";
char rx_buffer[MAX_AMOUNT_BUFFERS][RX_BUFFER_SIZE];
char DVL_Parced[MAX_AMOUNT_BUFFERS][MAX_NUM_WORDS][MAX_WORD_LENGTH];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

	//-------------------Temp---------------
	float alpha = 0.5f; // Definer alfa-verdi for EMA-filter

	//------------TRYKK----------------

	HAL_StatusTypeDef i2c_ret;
	uint8_t i2c_buffer[10];

	//-------------------DVL----------------
	char rx_buffer[MAX_AMOUNT_BUFFERS][RX_BUFFER_SIZE];
	char DVL_Parced[MAX_AMOUNT_BUFFERS][RX_BUFFER_SIZE];



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  EMA_Init(&ema_filter, alpha);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_CORDIC_Init();
  MX_CRC_Init();
  MX_FMAC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //-----------CANFD Oppstart--------------
  CANFD_Init();

  //-----------UART Oppstart---------------
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_ERROR(USART1);
  //------------Temp oppstart---------------------------

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) tempBuf, numChannels);
  HAL_TIM_Base_Start(&htim3);
  //-----------pressuresensor/I2C Oppstart----

 // Velger samplingsrate (se enum i MS5837-30BA.c)
  ms5837_t pressuresensor; // Initialiserer en pressuresensor struct
  pressuresensor.i2c_address = 0x76; // Setter adressen til riktig adresse
  //Nullstill sensor
  ms5837_reset(&pressuresensor); // Nullstiller pressuresensoren
  HAL_Delay(30);
  ms5837_read_calibration_data(&pressuresensor); // Henter ut kalibreringsverdier
  ms5837_start_conversion(&pressuresensor, SENSOR_PRESSURE, OSR_4096);
  // Read calibration data

  //--------------------------------------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //------------TRYKK---------------

	  //-----------TEMP-----------------
	  while (convCompleted !=1){

	  	  }

	  	  convCompleted = 0;

	  	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)tempBuf, numChannels);




	  //pressuresensor
//	  if (ms5837_read_calibration_data(&pressuresensor)) {
//	      // Klarer å lese
//		  ms5837_start_conversion(&pressuresensor, type, osr); // Begynner konvertering av verdi
//		  ms5837_read_conversion(&pressuresensor); // Leser konvertert verdi
//		  ms5837_calculate(&pressuresensor); // Beregner faktisk for verdi temp og trykk
//	  } else {
//	      // Klarer ikke å lese
//	  }


//		uint8_t liste1[] = {0x18, 0x34, 0x56, 0x34, 0x56, 0x34, 0x34, 0x22,0x12, 0x34, 0x56, 0x34, 0x56, 0x34, 0x34, 0x22,0x12, 0x34, 0x56, 0x34, 0x56, 0x34, 0x34, 0x22,0x12, 0x34, 0x56, 0x34, 0x56, 0x34, 0x34, 0x22,0x12, 0x34, 0x56, 0x34, 0x56, 0x34, 0x34, 0x22,0x12, 0x34, 0x56, 0x34, 0x56, 0x34, 0x34, 0x22,0x12, 0x34, 0x56, 0x34, 0x56, 0x34, 0x34, 0x22,0x12, 0x34, 0x56, 0x34, 0x56, 0x34, 0x34, 0x22};
//			SendData(0x310, liste1, sizeof(liste1), uint8);
//			HAL_Delay(300);



	  }

			/* hi2c er hvilken i2c modul som skal brukes
			 * adresse7bit er shiftet til venstre for å read(0) eller write(1) som lsb
			 * adressen hvor kommandoen som skal sendes er lagret
			 * antall byte som sendes
			 * Timeout
			 * */

//
//	  void ms5837_i2c_read(ms5837_t *sensor, uint8_t command, uint8_t *data, uint8_t num_bytes);
//	  void ms5837_i2c_write(ms5837_t *sensor, uint8_t command);


//			HAL_I2C_Master_Transmit(&hi2c3,0x76<<1,&command, sizeof(command), Timeout);
//			HAL_I2C_Master_Receive(&hi2c3,((0x76<<1)|0x1), &command, sizeof(command), Timeout);
//
//			HAL_I2C_Mem_Write(&hi2c3, 0xED, 0, 3, &pData, 1, Timeout);
//			HAL_I2C_Mem_Read(&hi2c3, 0xEC, 0, 1, &pData, 3, Timeout);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*hadc){

	convCompleted =1;
	HAL_ADC_Stop_DMA(&hadc1);


	  for (int i = 0; i < numChannels; i++) {
		  // Konverter ADC-verdier til mv
	    float input_value = __HAL_ADC_CALC_DATA_TO_VOLTAGE(__VREFANALOG_VOLTAGE_, tempBuf[i], ADC_RESOLUTION12b);
	    filtrertTemp[i] = EMA_Update(&ema_filter, input_value);


	    // Konvertere fra mv til celsius:
	    // EKSEMPEL --- float temperatureC = (voltage - 0.5) * 100; --- 0.5 er offsettet til
	    	// sensor i eksempel da den gir ut 0.5V ved 0 grader C, og den gir ut 10 mv/C
	   // float celsius = (filtrertTemp[i] - offset) *100;  // annet tall enn 100?

	  }

}

/*Sjekker først om en av pinnene har logisk høy verdi */
void Lekk(void){
	if((GPIOB->IDR & LL_GPIO_PIN_11) || (GPIOB->IDR & LL_GPIO_PIN_12) || (GPIOB->IDR & LL_GPIO_PIN_13) || (GPIOB->IDR & LL_GPIO_PIN_14)  	|| (GPIOB->IDR & LL_GPIO_PIN_15)) {

						/*Dersom en av pinnene er høy lages en maske som forteller hvor lekkasjen er. */
		  		  	  if(GPIOB->IDR & LL_GPIO_PIN_11){
		  		  		  lekkasje |= 0x1;
		  		  	  }
		  		  	  if(GPIOB->IDR & LL_GPIO_PIN_12){
		  		  		  lekkasje |= 0x2;
		  		  	  }
		  		  	  if(GPIOB->IDR & LL_GPIO_PIN_13){
		  		  		  lekkasje |= 0x4;
		  		  	  }
		  		  	  if(GPIOB->IDR & LL_GPIO_PIN_14){
		  		  		  lekkasje |= 0x8;
		  		  	  }

		  		  	  //knapp kun for testing
//		  		  	  if(GPIOB->IDR & LL_GPIO_PIN_15){
//		  		  		  lekkasje |= 0x10;
//		  		  	  }
	}
	  if (lekkasje > 0){
		  send_msg |= 0x4;
	  }
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
