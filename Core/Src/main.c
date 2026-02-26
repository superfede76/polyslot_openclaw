/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "MAX31865.h"
#include "MCP3204.h"
#include "pulse_counter.h"
#include "modbusSlave.h"
#include "polyslot.h"
#include "digitalIO.h"
#include "powermeter.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define EEPROM_ADDRESS  (0x50 << 1)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SLOT_CS slot_cs_array[MAX_CS] = {
		{NSS_A_GPIO_Port, NSS_A_Pin},
		{NSS_B_GPIO_Port, NSS_B_Pin},
		{NSS_C_GPIO_Port, NSS_C_Pin},
		{NSS_D_GPIO_Port, NSS_D_Pin},
		{NSS_E_GPIO_Port, NSS_E_Pin},
		{NSS_F_GPIO_Port, NSS_F_Pin},
		{NSS_G_GPIO_Port, NSS_G_Pin},
		{NSS_H_GPIO_Port, NSS_H_Pin}
};




bool coils_table[TOTAL_COILS];
bool contact_table[TOTAL_CONTACTS];
bool inputReg_table[TOTAL_INPUT_REGS];


enum slot installed_slot[8];

uint8_t RxData[256];
uint8_t TxData[256];

Max31865_t  pt100[MAX_CS][MAX_DEMUX];
Mcp3204_t analog[MAX_CS];
DigitalIO_t digitalIO[MAX_CS];
pulse_t counter[MAX_CS];
powermeter_t powermeter[MAX_CS];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint8_t slaveId = Modbus_GetSlaveId();
	if (RxData[0] == slaveId)
	{
		switch (RxData[1]){
		case 0x01: 	//OK
			readCoils();
			break;
		case 0x02:	//OK
			readInputs();
			break;
		case 0x03:	//ok
			readHoldingRegs();
			break;
		case 0x04:	//OK
			readInputRegs();
			break;
		case 0x05:	//ok
			writeSingleCoil();
			break;
		case 0x06:	//ok
			writeSingleReg();
			break;
		case 0x0F:	//ok //15
			writeMultiCoils();
			break;
		case 0x10:	//ok //16
			writeHoldingRegs();
			break;
		default:
			modbusException(ILLEGAL_FUNCTION);
			break;
		}
	}

	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 256);
}

void slot_poll(){
	uint8_t vartype=0;
	for (uint8_t i = 0; i < NUM_SLOT; i++) {
		HAL_GPIO_WritePin(slot_cs_array[i].NSS_Port, slot_cs_array[i].NSS_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);

		vartype = HAL_GPIO_ReadPin(ID3_GPIO_Port, ID3_Pin)<<3;
		vartype |= (HAL_GPIO_ReadPin(ID2_GPIO_Port, ID2_Pin)<<2);
		vartype |= (HAL_GPIO_ReadPin(ID1_GPIO_Port, ID1_Pin)<<1);
		vartype |= HAL_GPIO_ReadPin(ID0_GPIO_Port, ID0_Pin);


		HAL_GPIO_WritePin(slot_cs_array[i].NSS_Port, slot_cs_array[i].NSS_Pin, GPIO_PIN_SET);
		HAL_Delay(10);

		switch(vartype){
		case 7: //cassandra
			installed_slot[i]=cassandra;
			//azione
			break;
		case 8: //power meter
			installed_slot[i] = powermeter_board;
			// Abilita i registri di input per questo slot nella zona powermeter: indici da INPUTREG_ANALOG_COUNT + i*PM_REGS_PER_SLOT
			memset(inputReg_table + INPUTREG_ANALOG_COUNT + (i * PM_REGS_PER_SLOT),true,PM_REGS_PER_SLOT);
			// Inizializza driver powermeter per questo slot
			powermeter_init(&powermeter[i],&hspi1,slot_cs_array[i].NSS_Port,slot_cs_array[i].NSS_Pin,i);
			break;
		case 9: //analog input
			installed_slot[i]=analog_board;
			memset(inputReg_table+(i*REG_PER_SLOT),true,REG_PER_SLOT);
			Mcp3204_init(&(analog[i]),&hspi1,slot_cs_array[i].NSS_Port,slot_cs_array[i].NSS_Pin);
			break;
		case 10: //PT100
			installed_slot[i]=pt100_board;
			memset(inputReg_table+(i*REG_PER_SLOT),true,REG_PER_SLOT);
			Max31865_init(&(pt100[i][0]),&hspi1,slot_cs_array[i].NSS_Port,slot_cs_array[i].NSS_Pin,0,MAX31865_2WIRE,FILT50HZ);
			Max31865_init(&(pt100[i][1]),&hspi1,slot_cs_array[i].NSS_Port,slot_cs_array[i].NSS_Pin,1,MAX31865_2WIRE,FILT50HZ);
			Max31865_init(&(pt100[i][2]),&hspi1,slot_cs_array[i].NSS_Port,slot_cs_array[i].NSS_Pin,2,MAX31865_2WIRE,FILT50HZ);
			Max31865_init(&(pt100[i][3]),&hspi1,slot_cs_array[i].NSS_Port,slot_cs_array[i].NSS_Pin,3,MAX31865_2WIRE,FILT50HZ);
			break;
		case 11: //pulse counter
			installed_slot[i]=pulsecounter;
			memset(inputReg_table+(i*REG_PER_SLOT),true,REG_PER_SLOT);
			counter_init(&(counter[i]),&hspi1,slot_cs_array[i].NSS_Port,slot_cs_array[i].NSS_Pin);
			break;
		case 12: //mixed IO
			installed_slot[i]=mixedIO;
			memset(coils_table+(i*BIT_PER_SLOT),true,HALF_BIT_PER_SLOT);
			memset(contact_table+(i*BIT_PER_SLOT)+HALF_BIT_PER_SLOT,true,HALF_BIT_PER_SLOT );
			digitalIO_Init(&(digitalIO[i]),&hspi1,slot_cs_array[i].NSS_Port,slot_cs_array[i].NSS_Pin, MIXED);
			break;
		case 13: //digital output
			installed_slot[i]=output;
			memset(coils_table+(i*BIT_PER_SLOT),true,BIT_PER_SLOT );
			digitalIO_Init(&(digitalIO[i]),&hspi1,slot_cs_array[i].NSS_Port,slot_cs_array[i].NSS_Pin, OUTPUT);
			break;
		case 14: //digital input
			installed_slot[i]=input;
			memset(contact_table+(i*BIT_PER_SLOT),true,BIT_PER_SLOT );
			digitalIO_Init(&(digitalIO[i]),&hspi1,slot_cs_array[i].NSS_Port,slot_cs_array[i].NSS_Pin, INPUT);
			break;
		default:
			installed_slot[i]=none;
			//azione
			break;
		}
	}
}



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
  MX_I2C1_Init();
  Modbus_LoadSlaveIdFromEeprom();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LS_EN_GPIO_Port, LS_EN_Pin, GPIO_PIN_SET); //abilita i buffer dal micro verso la carrier board

	HAL_GPIO_WritePin(NSS_A_GPIO_Port, NSS_A_Pin, GPIO_PIN_SET); //disabilito tutti gli slot
	HAL_GPIO_WritePin(NSS_B_GPIO_Port, NSS_B_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NSS_C_GPIO_Port, NSS_C_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NSS_D_GPIO_Port, NSS_D_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NSS_E_GPIO_Port, NSS_E_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NSS_F_GPIO_Port, NSS_F_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NSS_G_GPIO_Port, NSS_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NSS_H_GPIO_Port, NSS_H_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(RS485_TX_EN_GPIO_Port,RS485_TX_EN_Pin,GPIO_PIN_SET); //deve essere sempre abilitato perchè il dispositivo è uno slave


	slot_poll();
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 256);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_C_Pin|NSS_D_Pin|NSS_E_Pin|NSS_F_Pin
                          |NSS_G_Pin|NSS_H_Pin|DEMUX_A_Pin|DEMUX_B_Pin
                          |NSS_A_Pin|NSS_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LS_EN_Pin|RS485_TX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ID0_Pin ID1_Pin ID2_Pin ID3_Pin
                           DRDY_Pin */
  GPIO_InitStruct.Pin = ID0_Pin|ID1_Pin|ID2_Pin|ID3_Pin
                          |DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_C_Pin NSS_D_Pin NSS_E_Pin NSS_F_Pin
                           NSS_G_Pin NSS_H_Pin DEMUX_A_Pin DEMUX_B_Pin
                           NSS_A_Pin NSS_B_Pin */
  GPIO_InitStruct.Pin = NSS_C_Pin|NSS_D_Pin|NSS_E_Pin|NSS_F_Pin
                          |NSS_G_Pin|NSS_H_Pin|DEMUX_A_Pin|DEMUX_B_Pin
                          |NSS_A_Pin|NSS_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LS_EN_Pin RS485_TX_EN_Pin */
  GPIO_InitStruct.Pin = LS_EN_Pin|RS485_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
int _write(int file, char *ptr, int len)
{
	(void)file;
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}
*/
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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
#ifdef USE_FULL_ASSERT
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
