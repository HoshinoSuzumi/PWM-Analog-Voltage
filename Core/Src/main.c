/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_PERIOD 500
#define OUTPUT_MAX_VOLTAGE 5

#define USART_REC_LEN	256

#define ADDR_24LCxx_Write	0xA0
#define ADDR_24LCxx_Read	0xA1
#define MEMADDR_SysConfig 0x00
#define MEMADDR_CHANNEL1	0x08
#define MEMADDR_CHANNEL2	0x10
// #define BufferSize 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OutputChannel1 */
osThreadId_t OutputChannel1Handle;
const osThreadAttr_t OutputChannel1_attributes = {
  .name = "OutputChannel1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for OutputChannel2 */
osThreadId_t OutputChannel2Handle;
const osThreadAttr_t OutputChannel2_attributes = {
  .name = "OutputChannel2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* USER CODE BEGIN PV */
// 			接收状态
// bit15，		接收完成标志
// bit14，		接收到0x0d
// bit13~0，	接收到的有效字节数目
uint16_t USART_RX_STA = 0;
uint8_t USART_RX_BUF[USART_REC_LEN] = {0};
	
const char sep[2] = " ";
char *token;
char *commands[5] = {0};

uint8_t idx = 0;
uint8_t dataLength;

uint8_t E2P_Write_Buffer[8], E2P_Read_Buffer[8];
struct System_Config
{
	bool sync;										// Sync or Async mode
} STRUCT_SYSTEM;
struct Channel_Config
{
	double voltage;								// Voltage
	uint16_t period;							// Period
	uint8_t duty;									// Duty cycle
	uint8_t delay;								// Boot delay
	uint8_t repeat;								// Repeat
} STRUCT_CHANNEL1, STRUCT_CHANNEL2;

uint8_t currentChannel = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void OutputChannel1Task(void *argument);
void OutputChannel2Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Rewrite fputc
int fputc(int ch,FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,0xffff);
	return ch;
}

// USART1 IEQ Handler
void USART_Command_IRQHandler(void)
{
	uint8_t Res;
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE) != RESET))
	{
		Res=USART1->DR; 
		if((USART_RX_STA&0x8000)==0)
		{
			if(USART_RX_STA&0x4000)
			{
				if(Res!=0x0a)USART_RX_STA=0;
				else USART_RX_STA|=0x8000;
			}
			else
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;  
				}		 
			}
		}   		 
	}
	HAL_UART_IRQHandler(&huart1);	
}

/**
  * @brief  Convert analog voltage to PWM duty.
	* @param	int target voltage
  * @retval int Duty
  */
int Voltage_Duty_Convert(double voltage)
{
	if(voltage > OUTPUT_MAX_VOLTAGE) voltage = OUTPUT_MAX_VOLTAGE;
	double ret = voltage / OUTPUT_MAX_VOLTAGE * PWM_PERIOD;
	if(ret > PWM_PERIOD) ret = PWM_PERIOD;
	return ret;
}

uint8_t OUTPUT_CHANNEL[] = {0, TIM_CHANNEL_1, TIM_CHANNEL_2};
void Set_Channel_Voltage(uint8_t channel, double voltage)
{
	__HAL_TIM_SetCompare(&htim3, OUTPUT_CHANNEL[channel], Voltage_Duty_Convert(voltage));
}

void Update_Struct_Config()
{
	uint8_t Config_Buffer[8];
	HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, 1 * 0x08, I2C_MEMADD_SIZE_8BIT, Config_Buffer, 8, 0xff);
	STRUCT_CHANNEL1.voltage	= Config_Buffer[0] / 10.0;
	STRUCT_CHANNEL1.period	= (Config_Buffer[1] << 8) + Config_Buffer[2];
	STRUCT_CHANNEL1.duty		= Config_Buffer[3];
	STRUCT_CHANNEL1.delay		= Config_Buffer[4];
	STRUCT_CHANNEL1.repeat	= Config_Buffer[5];
	
	HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, 2 * 0x08, I2C_MEMADD_SIZE_8BIT, Config_Buffer, 8, 0xff);
	STRUCT_CHANNEL2.voltage	= Config_Buffer[0] / 10.0;
	STRUCT_CHANNEL2.period	= (Config_Buffer[1] << 8) + Config_Buffer[2];
	STRUCT_CHANNEL2.duty		= Config_Buffer[3];
	STRUCT_CHANNEL2.delay		= Config_Buffer[4];
	STRUCT_CHANNEL2.repeat	= Config_Buffer[5];
	
	HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, 0x00, I2C_MEMADD_SIZE_8BIT, Config_Buffer, 1, 0xff);
	STRUCT_SYSTEM.sync = Config_Buffer[0];
}

void onCommandHandler(char *commands[])
{
	if(!strcmp(commands[0], "SET"))
	{
		if(!strcmp(commands[1], "MODE"))
		{
			HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, 0x00, I2C_MEMADD_SIZE_8BIT, (uint8_t[]) {!strcmp(commands[2], "SYNC")}, 1, 1000);
			HAL_Delay(5);
			printf("[-] SWITCH TO %s mode\n", !strcmp(commands[2], "SYNC") ? "sync" : "async");
		}
		else if(!strcmp(commands[1], "CHANNEL"))
		{
			switch(atoi(commands[2]))
			{
				case 1:
				case 2:
					if(!strcmp(commands[3], "VOLTAGE"))
					{
						Set_Channel_Voltage(atoi(commands[2]), atof(commands[4]));
						HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, atoi(commands[2]) * 0x08, I2C_MEMADD_SIZE_8BIT, (uint8_t[]) {atof(commands[4]) * 10}, 1, 1000);
						HAL_Delay(5);
						printf("[-]	SET CHANNEL %d VOLTAGE TO %.2fv\n", atoi(commands[2]), atof(commands[4]));
					}
					else if(!strcmp(commands[3], "PERIOD"))
					{
						uint16_t value = atoi(commands[4]);
						if(value > 0xFFFF) value = 0xFFFF;
						HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, atoi(commands[2]) * 0x08 + 1, I2C_MEMADD_SIZE_8BIT, (uint8_t[]) {(value >> 8) & 0x00FF, value & 0x00FF}, 2, 1000);
						HAL_Delay(10);
						printf("[-]	SET CHANNEL %d PERIOD TO %d min(s)\n", atoi(commands[2]), value);
					}
					else if(!strcmp(commands[3], "DUTY"))
					{
						uint8_t value = atoi(commands[4]) & 0x7F;
						if(value > 0x64) value = 0x64;
						HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, atoi(commands[2]) * 0x08 + 3, I2C_MEMADD_SIZE_8BIT, (uint8_t[]) {value}, 1, 1000);
						HAL_Delay(5);
						printf("[-]	SET CHANNEL %d DUTY TO %d%%\n", atoi(commands[2]), value);
					}
					else if(!strcmp(commands[3], "DUTY_REVERSE"))
					{
						uint8_t value = atoi(commands[4]) & 0x01;
						
						uint8_t buffer[1];
						HAL_I2C_Mem_Read(&hi2c1, ADDR_24LCxx_Read, atoi(commands[2]) * 0x08 + 3, I2C_MEMADD_SIZE_8BIT, buffer, 1, 0xff);
						HAL_Delay(5);
						
						uint8_t memValue = buffer[0];
						bool isReversed = (memValue & 0x80) >> 7;
						if(!(value == isReversed)) memValue ^= 0x80;
						
						HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, atoi(commands[2]) * 0x08 + 3, I2C_MEMADD_SIZE_8BIT, (uint8_t[]) {memValue}, 1, 1000);
						HAL_Delay(5);
						printf("[-]	SET CHANNEL %d DUTY TURNED TO %s\n", atoi(commands[2]), !value ? "H to L" : "L to H");
					}
					else if(!strcmp(commands[3], "DELAY"))
					{
						uint8_t value = atoi(commands[4]);
						HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, atoi(commands[2]) * 0x08 + 4, I2C_MEMADD_SIZE_8BIT, (uint8_t[]) {value}, 1, 1000);
						HAL_Delay(5);
						printf("[-]	SET CHANNEL %d DELAY TO %d sec(s)\n", atoi(commands[2]), value);
					}
					else if(!strcmp(commands[3], "REPEAT"))
					{
						uint8_t value = atoi(commands[4]);
						HAL_I2C_Mem_Write(&hi2c1, ADDR_24LCxx_Write, atoi(commands[2]) * 0x08 + 5, I2C_MEMADD_SIZE_8BIT, (uint8_t[]) {value}, 1, 1000);
						HAL_Delay(5);
						printf("[-]	SET CHANNEL %d REPEAT TO %d time(s)\n", atoi(commands[2]), value);
					}
					else 
					{
						printf("[!]	Unknown command [%s]\n", commands[3]);
					}
					break;
				default:
					printf("[!]	Unknown channel [%s]\n", commands[2]);
			}
		}
		Update_Struct_Config();
	}
	else if(!strcmp(commands[0], "GET"))
	{
		if(!strcmp(commands[1], "CONFIG"))
		{
			switch(atoi(commands[2]))
			{
				case 1:
					printf("[-] CHANNEL 1 Configuration\nVoltage:	%.2fv\nPeriod:		%d min(s)\nDuty:		%d%%%s\nDelay:		%ds\nRepeat:		%d time(s)\n\n",
					STRUCT_CHANNEL1.voltage,
					STRUCT_CHANNEL1.period,
					STRUCT_CHANNEL1.duty & 0x7F,
					(STRUCT_CHANNEL1.duty & 0x80) >> 7 ? "(reversed)" : "",
					STRUCT_CHANNEL1.delay,
					STRUCT_CHANNEL1.repeat);
					break;
				case 2:
					printf("[-] CHANNEL 2 Configuration\nVoltage:	%.2fv\nPeriod:		%d min(s)\nDuty:		%d%%%s\nDelay:		%ds\nRepeat:		%d time(s)\n\n",
					STRUCT_CHANNEL2.voltage,
					STRUCT_CHANNEL2.period,
					STRUCT_CHANNEL2.duty & 0x7F,
					(STRUCT_CHANNEL2.duty & 0x80) >> 7 ? "(reversed)" : "",
					STRUCT_CHANNEL2.delay,
					STRUCT_CHANNEL2.repeat);
					break;
			}
		}
	}
	if(!strcmp(commands[0], "RESET"))
	{
		HAL_NVIC_SystemReset();
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	printf("[!] System is up now!\n\n");
	
	Update_Struct_Config();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of OutputChannel1 */
  OutputChannel1Handle = osThreadNew(OutputChannel1Task, NULL, &OutputChannel1_attributes);

  /* creation of OutputChannel2 */
  OutputChannel2Handle = osThreadNew(OutputChannel2Task, NULL, &OutputChannel2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		if(USART_RX_STA & 0x8000)
		{
			token = strtok((char*) USART_RX_BUF, sep);
			while( token != NULL ) {
				commands[idx++] = token;
				token = strtok(NULL, sep);
			}
			idx = 0;
			onCommandHandler(commands);
			while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != SET);
			memset(USART_RX_BUF, 0, sizeof(USART_RX_BUF));
			USART_RX_STA=0;
		}
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_OutputChannel1Task */
/**
* @brief Function implementing the OutputChannel1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OutputChannel1Task */
void OutputChannel1Task(void *argument)
{
  /* USER CODE BEGIN OutputChannel1Task */
	int Repeat;
	double Period_H, Period_L, percent;
  /* Infinite loop */
  for(;;)
  {
		if(STRUCT_SYSTEM.sync && (currentChannel != 1)) break;
		percent		= (STRUCT_CHANNEL1.duty & 0x7F) / 100.0;
		Period_L	=	STRUCT_CHANNEL1.period * percent;
		Period_H	= STRUCT_CHANNEL1.period - Period_L;
		Repeat		= STRUCT_CHANNEL1.repeat;
    osDelay(STRUCT_CHANNEL1.delay * 1000);
		for(int i = 0; i < Repeat; i++)
		{
			if((STRUCT_CHANNEL1.duty & 0x80) >> 7 == 0)
			{
				Set_Channel_Voltage(1, STRUCT_CHANNEL1.voltage);
				osDelay(Period_H * 1000 * 60);
				Set_Channel_Voltage(1, 0);
				osDelay(Period_L * 1000 * 60);
			}
			else
			{
				Set_Channel_Voltage(1, 0);
				osDelay(Period_L * 1000 * 60);
				Set_Channel_Voltage(1, STRUCT_CHANNEL1.voltage);
				osDelay(Period_H * 1000 * 60);
			}
		}
		currentChannel = 2;
  }
  /* USER CODE END OutputChannel1Task */
}

/* USER CODE BEGIN Header_OutputChannel2Task */
/**
* @brief Function implementing the OutputChannel2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OutputChannel2Task */
void OutputChannel2Task(void *argument)
{
  /* USER CODE BEGIN OutputChannel2Task */
	int Repeat;
	double Period_H, Period_L, percent;
  /* Infinite loop */
  for(;;)
  {
		if(STRUCT_SYSTEM.sync && (currentChannel != 2)) break;
		percent		= (STRUCT_CHANNEL2.duty & 0x7F) / 100.0;
		Period_L	=	STRUCT_CHANNEL2.period * percent;
		Period_H	= STRUCT_CHANNEL2.period - Period_L;
		Repeat		= STRUCT_CHANNEL2.repeat;
    osDelay(STRUCT_CHANNEL2.delay * 1000);
		for(int i = 0; i < Repeat; i++)
		{
			if((STRUCT_CHANNEL2.duty & 0x80) >> 7 == 0)
			{
				Set_Channel_Voltage(2, STRUCT_CHANNEL2.voltage);
				osDelay(Period_H * 1000 * 60);
				Set_Channel_Voltage(2, 0);
				osDelay(Period_L * 1000 * 60);
			}
			else
			{
				Set_Channel_Voltage(2, 0);
				osDelay(Period_L * 1000 * 60);
				Set_Channel_Voltage(2, STRUCT_CHANNEL2.voltage);
				osDelay(Period_H * 1000 * 60);
			}
		}
		currentChannel = 1;
  }
  /* USER CODE END OutputChannel2Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
