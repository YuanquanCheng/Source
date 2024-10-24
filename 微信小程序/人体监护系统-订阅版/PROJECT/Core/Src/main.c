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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "image.h"
#include "lcd.h"
#include "lcd_init.h"
#include "stdio.h"
#include "string.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{

HAL_UART_Transmit(&huart2,(uint8_t *)&ch, 1, 0xFFFF);
return ch;
}

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

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  uint8_t Rx_data = 0;
	#define BUFFER_SIZE 256
	#define END_CHAR '\n'
	
	uint8_t rx_buffer[BUFFER_SIZE];
	uint8_t rx_index=0;


	void process_data(uint8_t *data,uint16_t length)
{
	if(strstr((char*)data,"\"LEDSTATE\":1"))
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
		//printf("LED1 ON\r\n");
		//printf("send \"on\" ok\r\n");
		memset(rx_buffer,0,BUFFER_SIZE);
		
	}
	else if(strstr((char*)data,"\"LEDSTATE\":0"))
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
		//printf("LED1 OFF\r\n");
		//printf("send \"off\" ok\r\n");
		memset(rx_buffer,0,BUFFER_SIZE);
	}
	if(strstr((char*)data,"\"Switch\":0"))
	{
		//__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1500);
		//printf("send \"on\" ok\r\n");
		memset(rx_buffer,0,BUFFER_SIZE);
		
	}
	else if(strstr((char*)data,"\"Switch\":1"))
	{
		//__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,500);
		//printf("send \"off\" ok\r\n");
		memset(rx_buffer,0,BUFFER_SIZE);
	}
	
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)//仅处理中断1
	{
		if(rx_index < BUFFER_SIZE - 1)//缓冲区还有空间
		{
			if(rx_buffer[rx_index-1] != END_CHAR)//如果上一符号不是结束字符
			{
				rx_buffer[rx_index++]=(uint8_t)(huart->Instance->DR &0X00FF);
				
			}
			if(rx_buffer[rx_index-1] == END_CHAR)//如果上一符号是结束字符
			{
				process_data(rx_buffer ,rx_index-1);
				rx_index=0;
				
			}
		}
		else
		{
			rx_index = 0;
			memset(rx_buffer,0,BUFFER_SIZE);
		}
		HAL_UART_Receive_IT(&huart2,&Rx_data,1);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Fill(0,0,128,160,BLACK);
	
	
	int XMP_ph=0;
	int XMP_water=0;
	int temperature=0;
	int humidity=0;
	
	LCD_ShowString(5,5,(const uint8_t *)"Loopy",GREEN,WHITE,24,1);
	Paint_DrawImage(gImage_LOOPY2,0, 35, 128, 127);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buffer,1);
	HAL_Delay(1000);
	/**
	以下部分填上自己阿里云的信息
	**/
	printf("AT+CWJAP=\"XMP\",\"1234567xmpm2\"\r\n");
	HAL_Delay(3000);
	printf("AT+MQTTUSERCFG=0,1,\"NULL\",\r\n");
	HAL_Delay(1500);
	printf("AT+MQTTCLIENTID=0,\r\n");
	HAL_Delay(1000);
	printf("AT+MQTTCONN=0,,1883,1\r\n");
	HAL_Delay(2000);
	printf("AT+MQTTSUB=0,,1\r\n");
	HAL_Delay(1500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		
		HAL_Delay(200);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
		/**
		以下部分填上自己的阿里云信息
		**/
		printf("AT+MQTTPUB=0,,\"{\\\"tem\\\":%d\\,\\\"hum\\\":%d}\",1,0\r\n",temperature,humidity);
		HAL_Delay(1000);
		temperature++;
		humidity++;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&(ch),1,1000);
	return (ch);
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
