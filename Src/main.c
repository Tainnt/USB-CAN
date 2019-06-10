/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "stdio.h"
#include "queue.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_BAUDRATE_BS1 				CAN_BS1_12TQ	// Using for set-up CAN baudrate
#define CAN_BAUDRATE_BS2 				CAN_BS2_5TQ		// Using for set-up CAN baudrate
#define RETRY_TIME		 					50						// Retry time if transmit not success

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CanTxMsgTypeDef TxMsg;							
CanRxMsgTypeDef RxMsg;							 
CAN_FilterConfTypeDef sFilterConfig;		

Queue queue;																	// Queue for USB data and CAN data
uint16_t msgId;																// CAN message ID
bool filterIdMode;														// True: enable filter ID mode, false: disable filter ID mode
uint16_t filterId;														// In filter ID mode, CAN only receive message ID equal filterId
uint8_t timeout;															// Timeout for each CAN transmit data
uint8_t baudrate;															// CAN baudrate
uint8_t version	= 25;													// Version of code

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);

bool validateCommand(Message msg);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
 /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE 
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
 HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);

 return ch;
}

/* CAN receive interrupt */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	/* Assign CAN message to queue */
	Message msg;							
	msg.type = CAN_MESSAGE;
	msg.length = hcan->pRxMsg->DLC;
	memcpy(msg.data, hcan->pRxMsg->Data, msg.length);	
	Queue_Push(&queue, &msg);
}

/**
  * @brief CAN Configure Function
  * @param None
  * @retval None
  */

void CAN_Config(void)
{
	/* CAN filter config */
	if(filterIdMode)
	{
		sFilterConfig.FilterNumber=0;
		sFilterConfig.FilterMode=CAN_FILTERMODE_IDLIST;
		sFilterConfig.FilterScale=CAN_FILTERSCALE_16BIT;
		sFilterConfig.FilterIdHigh=filterId<<5;
		sFilterConfig.FilterIdLow=0x0000<<5;
		sFilterConfig.FilterMaskIdHigh=0x0000<<5;
		sFilterConfig.FilterMaskIdLow=0x0000<<5;
		sFilterConfig.FilterFIFOAssignment=0;
		sFilterConfig.FilterActivation=ENABLE;
		sFilterConfig.BankNumber=0;
		HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	}
	else
	{
		sFilterConfig.FilterNumber=0;
		sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh=0;
		sFilterConfig.FilterIdLow=0;
		sFilterConfig.FilterMaskIdHigh=0;
		sFilterConfig.FilterMaskIdLow=0;
		sFilterConfig.FilterFIFOAssignment=0;
		sFilterConfig.FilterActivation=ENABLE;
		sFilterConfig.BankNumber=0;
		HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	}

	/* CAN message config */
	hcan.pTxMsg = &TxMsg;
	hcan.pRxMsg = &RxMsg;
	hcan.pTxMsg->RTR = CAN_RTR_DATA; 
	hcan.pTxMsg->IDE = CAN_ID_STD;
	hcan.pTxMsg->StdId = msgId;
}

/**
  * @brief Validate Command Function
  * @param 
  * @retval 
  */

bool validateCommand(Message msg)
{
	if(Crc8_Calc(msg.data,msg.length - 1) != msg.data[msg.length - 1])
	{
		return false;
	}
	else
	{
		uint8_t result[5];
		switch(msg.data[0])
		{
			case GET_VERSION:
				result[0] = GET_VERSION;
				result[1] = 0x01;
				result[2] = version;
				result[3] = Crc8_Calc(result,3);
				CDC_Transmit_FS(result, 4);
				break;
			case GET_ID:
				result[0] = GET_ID;
				result[1] = 0x02;
				result[2] = (msgId & 0xFF00) >> 8;
				result[3] = msgId & 0x00FF;
				result[4] = Crc8_Calc(result,4);
				CDC_Transmit_FS(result, 5);
				break;
			case SET_ID:
				msgId = ((uint16_t)msg.data[2] << 8) | (uint16_t)msg.data[3];
				CAN_Config();
				result[0] = SET_ID;
				result[1] = 0x00;
				result[2] = Crc8_Calc(result,2);
				CDC_Transmit_FS(result, 3);
				break;
			case GET_FILTER_ID:
				result[0] = GET_FILTER_ID;
				result[1] = 0x02;
				if(filterIdMode)
				{
					result[2] = (filterId & 0xFF00) >> 8;
					result[3] = filterId & 0x00FF;
				}
				else
				{
					result[2] = 0xFF;
					result[3] = 0xFF;
				}
				result[4] = Crc8_Calc(result,4);
				CDC_Transmit_FS(result, 5);
				break;
			case SET_FILTER_ID:
				if(msg.data[2] <= 0x07)
				{
					filterIdMode = true;
					filterId = ((uint16_t)msg.data[2] << 8) | (uint16_t)msg.data[3];
				}
				else
				{
					filterIdMode = false;
					filterId = 0x0000;
				}
				CAN_Config();
				result[0] = SET_FILTER_ID;
				result[1] = 0x00;
				result[2] = Crc8_Calc(result,2);
				CDC_Transmit_FS(result, 3);
				break;
			case GET_TIMEOUT:
				result[0] = GET_TIMEOUT;
				result[1] = 0x01;
				result[2] = timeout;
				result[3] = Crc8_Calc(result,3);
				CDC_Transmit_FS(result, 4);
				break;
			case SET_TIMEOUT:
				timeout = msg.data[2];
				result[0] = SET_TIMEOUT;
				result[1] = 0x00;
				result[2] = Crc8_Calc(result,2);
				CDC_Transmit_FS(result, 3);
				break;
			case GET_BAUDRATE:
				result[0] = GET_BAUDRATE;
				result[1] = 0x01;
				result[2] = baudrate;
				result[3] = Crc8_Calc(result,3);
				CDC_Transmit_FS(result, 4);
				break;
			case SET_BAUDRATE:
				baudrate = msg.data[2];
				MX_CAN_Init();
				result[0] = SET_BAUDRATE;
				result[1] = 0x00;
				result[2] = Crc8_Calc(result,2);
				CDC_Transmit_FS(result, 3);
				break;
			default:
				return false;
		}
		Write_Flash(msgId,filterId,filterIdMode,timeout,baudrate);
		return true;
	}
}

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
	Message  msg;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	Read_Flash(&msgId,&filterId,&filterIdMode,&timeout,&baudrate);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USB_DEVICE_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	CAN_Config();
	/* CAN receive interrupt config */
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(Queue_Pop(&queue, &msg))																								// check message in queue
		{	
			if(msg.type == USB_MESSAGE)																							// check type of message
			{
				if(!validateCommand(msg))																							// validate command from USB
				{
					hcan.pTxMsg->DLC = msg.length;																			// assign length and data from USB to CAN
					for(int i = 0; i < msg.length; i++)
					{
						hcan.pTxMsg->Data[i] = msg.data[i];
					}
					
					HAL_StatusTypeDef canStatus;
					uint8_t time = 0;
					do																																	// check can send status
					{
						canStatus = HAL_CAN_Transmit(&hcan, timeout);											// transmit data by CAN
						time++;
					}while((time<RETRY_TIME) && (canStatus!=HAL_OK));										// retry if CAN send status not success
				}
			}
			else
			{
				uint8_t usbStatus = USBD_BUSY;
				uint8_t time = 0;
				do																																		// check usb send status
				{
					usbStatus = CDC_Transmit_FS((uint8_t *)msg.data, msg.length);				// transmit data by USB
					time++;
				}while((time<RETRY_TIME) && (usbStatus!=USBD_OK));										// retry if USB send status not success
			}
		}
		
		/* Reset watchdog timer */
		HAL_IWDG_Refresh(&hiwdg);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
	hcan.Init.Prescaler = baudrate;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BAUDRATE_BS1;
  hcan.Init.BS2 = CAN_BAUDRATE_BS2;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_STBY_GPIO_Port, CAN_STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_STBY_Pin */
  GPIO_InitStruct.Pin = CAN_STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_STBY_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
