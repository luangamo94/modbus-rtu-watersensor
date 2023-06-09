/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "modbus_crc.h"
#include "stdio.h"
#include "string.h"
//#include "Hex2Int.h"


#include <stdio.h>
#include <ctype.h>
#include "string.h"
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t TXdata[24];        
uint8_t RXdata[24];
uint16_t data[10];
float DO[10];
//char TXdata[24];        
//char RXdata[24];
//int data[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/*
int htoi(uint8_t);
int getRawInt(uint8_t);
*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{	
	//  trong truong hop doc thanh ghi Holding register moi dc su dung
	
	data[0] = RXdata[3]<<8  |  RXdata[4];
	data[1] = RXdata[5]<<8  |  RXdata[6];
	data[2] = RXdata[7]<<8  |  RXdata[8];
	data[3] = RXdata[9]<<8  |  RXdata[10];
	data[4] = RXdata[11]<<8 |  RXdata[12]; */
	/*
	DO[0] = (RXdata[3]<<24)  | (RXdata[4]<<16)  | (RXdata[5]<<8)  | RXdata[6];
	DO[1] = (RXdata[7]<<24)  | (RXdata[8]<<16)  | (RXdata[9]<<8)  | RXdata[10];
	DO[2] = (RXdata[11]<<24) | (RXdata[12]<<16) | (RXdata[13]<<8) | RXdata[14];   
	
	*/
//}
/*
uint8_t ReadDO(uint8_t tx[], uint8_t numOftx, uint8_t addr, uint8_t quantity)
{
	tx[0] = 0x01;    // slave address
	tx[1] = 0x03;			  // Function code for Read Holding Registers
	tx[2] = 0x00;
	tx[3] = 0x00;
}
*/

void send_data(uint8_t *data)
{
		HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart1, data, 8, 1000);
		HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);		
}
/*
int my_strlen(uint8_t chuoi[])
{
	int dem = 0;
	while (chuoi[dem]  != '\0')
	{
		dem++;
	}
	return dem;
}

int htoi(uint8_t hex[])
{
	int LEN = my_strlen(hex) - 1;
	//he so
	int power = 1;
	// decimal
	int dec = 0;
	int i;
	for(i = LEN; i >= 0; i--)
	{
		dec += getRawInt(hex[i]) * power;
		power *= 16;
	}
	return dec;
}

int getRawInt(uint8_t c)
{
	if(isalpha(c))
	{
		// vi du c = 'C' -> c - 'A' +10 = 'C' - 'A' +10 = 12
		return toupper(c) - 'A' + 10;
	}
	return c - '0';
}
*/


//void readRegister()
//{
//	
//}

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	//HAL_UARTEx_ReceiveToIdle_IT(&huart2, RXdata, 20);
	
	TXdata[0] = 0x01;  // slave address
	TXdata[1] = 0x03;  // Function code for Read Holding Registers
//	TXdata[1] = 0x01;  // Function code for Read Coils
/*
   * The function code 0x03 means we are reading Holding Registers
   * The Register address ranges from 40001 - 50000
   * The register address we input can range from 0-9999 (0x00-0x270F)
   * Here 0 corresponds to the Address 40001 and 9999 corresponds to 50000
   * Although we can only read 125 registers sequentially at once
   */
//  TxData[2] = 0;
//  TxData[3] = 0x04;
//  //The Register address will be 00000000 00000100 = 4 + 40001 = 40005

//xac dinh dia chi thanh ghi bat dau doc
	TXdata[2] = 0x0A;
	TXdata[3] = 0x00;

// xac dinh so thanh ghi can doc
	TXdata[4] = 0x00;
	TXdata[5] = 0x01;


// xac dinh so coil can doc
// no. of coils to read is 00000000 00000100 = 8 coils = 1 byte
//	TXdata[4] = 0;
//	TXdata[5] = 0x02;


/*
	int16_t crc = crc16(TXdata, 6);
	TXdata[6] = crc&0xFF;   // CRC LOW
	TXdata[7] = (crc>>8)&0xFF;  // CRC HIGH
*/
	TXdata[6] = 0x87;   // CRC LOW
	TXdata[7] = 0xD2;

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
//		int value1 = 0;
//		value1 = htoi(RXdata[]);
		send_data(TXdata);
		//HAL_Delay(1000);
		HAL_UART_Receive(&huart1, RXdata, 8,500);
		HAL_Delay(1000);
		
//	union{
//     char RX[4];
//     float fl;
//		} u;
//	u.RX[3] = RXdata[3];
//	u.RX[2] = RXdata[4];
//	u.RX[1] = RXdata[5];
//	u.RX[0] = RXdata[6];
//	//print("%f\n", u.fl);	
//	float conv_ed = u.fl;
		
		
	//	int number = stoi(&RXdata[3],0, 16);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 19200;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TX_EN_Pin */
  GPIO_InitStruct.Pin = TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TX_EN_GPIO_Port, &GPIO_InitStruct);

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
