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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AlertaMov 500
#define PORT_DHT GPIOE
#define PIN_DHT	GPIO_PIN_11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
volatile bool flagmover,flagbase;
bool initX, initY, initZ;
uint8_t newx, newy, newz;
uint8_t basex;
uint8_t basey;
uint8_t basez;
volatile int counterInterrupt=0;
uint8_t data_ctrl1, address_ctrl1;
uint8_t valuechange, changeaddress;
uint8_t XDA,YDA,ZDA, XYZDA;
uint8_t bigX,bigY,bigZ,smlX,smlY,smlZ,Xadr,Yadr,Zadr;
uint32_t lastX,lastY,lastZ;

//Variables para DHT11
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2,Sum,Check;
uint8_t RH, TEMP;
uint8_t Pres=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){
		flagmover=true;;
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	flagbase=true;
}


//---------------------------Funciones Para DHT------------------

//Funcion para hace retrasos de micro segundos
void Retraso (uint16_t tiempo){
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while((__HAL_TIM_GET_COUNTER(&htim6))< tiempo){};
}


//Funcion para poner un Pin en modo Output
void Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
//Funcion para poner un Pin en modo Input
void Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


//Funciona Para comenzar trasmision.
void DHT11_Start(void){
	Pin_Output(PORT_DHT, PIN_DHT);
	HAL_GPIO_WritePin(PORT_DHT, PIN_DHT, GPIO_PIN_RESET);
	Retraso(18000);
	HAL_GPIO_WritePin(PORT_DHT, PIN_DHT, GPIO_PIN_SET);
	Retraso(20);
	Pin_Input(PORT_DHT, PIN_DHT);
}

//Funcion para confirmar respuesta
uint8_t Conf_Respuesta(void){
	uint8_t Resp = 0;
	Retraso(40);
	if (!(HAL_GPIO_ReadPin(PORT_DHT, PIN_DHT)))
	{
		Retraso(80);
		if((HAL_GPIO_ReadPin(PORT_DHT, PIN_DHT))){Resp = 1;}
		else{ Resp=-1;/* Resp = 255*/}
	}
	while((HAL_GPIO_ReadPin(PORT_DHT, PIN_DHT))){}
	return Resp;
}

//Leer sensor
uint8_t LeerDHT(void){
	uint8_t i = 0,j = 0;
	for(j=0; j<8 ; j++){
		while(!(HAL_GPIO_ReadPin(PORT_DHT, PIN_DHT))){}
		Retraso(40);
		if(!(HAL_GPIO_ReadPin(PORT_DHT, PIN_DHT))){ //Si tras esperar el Pin es 0
			i&= ~(1<<(7-j)); // el bit es cero
		}
		else{
		i|=(1<<(7-j));
		}// si no el bit es 1.
		while((HAL_GPIO_ReadPin(PORT_DHT, PIN_DHT))){}
	}
	return i;
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  // Parte del coidgo para activar el acelerometro
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, 0);
  address_ctrl1 = 0x20;
  data_ctrl1 = 0x47; //Referesco a 800Hz,
  HAL_SPI_Transmit(&hspi1, &address_ctrl1, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, &data_ctrl1, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, 1);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
  //Inicilazion Timer

  HAL_TIM_Base_Start_IT(&htim2);
  initX= false; initY= false; initZ= false;
  flagmover= false; flagbase= false;
  HAL_TIM_Base_Start(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//Poner las bases si el timepo de establecimiento ha pasado
	if(flagbase== true){
		if(initX == false){
			basex=newx;
			initX = true;
		}
		if(initY == false){
			basey=newy;
			initY = true;
		}
		if(initZ == false){
			basez=newz;
			initZ = true;
		}

		//DHT11 Tras esperar
		DHT11_Start();
		Pres= Conf_Respuesta();
		Rh_byte1 = LeerDHT();
		Rh_byte2 = LeerDHT();
		Temp_byte1= LeerDHT();
		Temp_byte2= LeerDHT();
		Sum = LeerDHT();

		Check= Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2;
		RH= Rh_byte1;
		TEMP = Temp_byte1;

	}
	// Comprobar si ha habido un cambio en los valores
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
	 changeaddress= 0x27 | 0x80;
	 HAL_SPI_Transmit(&hspi1, &changeaddress, 1, 50);
	 HAL_SPI_Receive(&hspi1, &valuechange, 1, 50);
	 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
	//Se deben separar los bits importantes, para eso se utiliza el operador & y una mascara
	 XDA= valuechange & 0x01;
	 YDA= valuechange & 0x02;
	 ZDA= valuechange & 0x04;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//Si hay nuevos valores cojerlos.
	if(XDA == 0x01){
		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, 0);
		 Xadr= 0x29 | 0x80;
		 HAL_SPI_Transmit(&hspi1, &Xadr, 1, HAL_MAX_DELAY);
		 HAL_SPI_Receive(&hspi1, &bigX, 1,  HAL_MAX_DELAY);
		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, 1);

		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, 0);
		 Xadr= 0x28 | 0x80;
		 HAL_SPI_Transmit(&hspi1, &Xadr, 1, HAL_MAX_DELAY);
		 HAL_SPI_Receive(&hspi1, &smlX, 1, HAL_MAX_DELAY);
		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, 1);
		 newx= bigX& 0x80;
	}
	if(YDA == 0x02){
		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
		 Yadr= 0x2B | 0x80;
		 HAL_SPI_Transmit(&hspi1, &Yadr, 1, 50);
		 HAL_SPI_Receive(&hspi1, &bigY, 1, 50);
		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);

		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
		 Yadr= 0x2A | 0x80;
		 HAL_SPI_Transmit(&hspi1, &Yadr, 1, 50);
		 HAL_SPI_Receive(&hspi1, &smlY, 1, 50);
		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
		 newy= bigY & 0x80;
	}
	if(ZDA == 0x04){
		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
		 Zadr= 0x2D | 0x80;
		 HAL_SPI_Transmit(&hspi1, &Zadr, 1, 50);
		 HAL_SPI_Receive(&hspi1, &bigZ, 1, 50);
		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);

		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
		 Zadr= 0x2C | 0x80;
		 HAL_SPI_Transmit(&hspi1, &Zadr, 1, 50);
		 HAL_SPI_Receive(&hspi1, &smlZ, 1, 50);
		 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
		 newz= bigZ & 0x80;
	}

	//Se encienden LEDs si se ha movido
	if(newx != basex && initX == true ){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	}
	if(newy != basey && initY == true){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	}
	if(newz != basez && initZ == true){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	}

	//Se apagan los leds tras un rato
	if(flagmover == true){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		flagmover= false;
	}



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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 18750;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 50-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
