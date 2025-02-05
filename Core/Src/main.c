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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// private variables
// for UART
static char buffer[200]; // Declare buffer globally for UART messages

// for ADC readings
volatile uint16_t adcResultsDMA[2];
const int adcChannelCount = sizeof(adcResultsDMA) / sizeof(adcResultsDMA[0]);		// nice way of writing number of ADC channels
volatile int adcConversionComplete = 0;			// set by callback



// Accelerometer_lsm303dlhc constants

// control registers
static const uint8_t CTRL_REG1_A   	 = 0x20;
static const uint8_t CTRL_REG2_A     = 0x21;
static const uint8_t CTRL_REG3_A     = 0x22;
static const uint8_t CTRL_REG4_A     = 0x23;
static const uint8_t CTRL_REG5_A     = 0x24;
static const uint8_t CTRL_REG6_A     = 0x25;

static const uint8_t ODR_400Hz       = 0b0111 << 4;	// 400Hz
static const uint8_t LPen      		 = 0b0001 << 3;	// Low Power
static const uint8_t Zen       		 = 0b0001 << 2;	// activates sensor for z axis
static const uint8_t Yen       		 = 0b0001 << 1;
static const uint8_t Xen       		 = 0b0001 << 0;
static const uint8_t BDU       		 = 0b01 << 7;		// (Block Data Update) - locks data until it is read
static const uint8_t FS_2G     		 = 0b00 << 4;  // +-2g
static const uint8_t FS_4G      	 = 0b01 << 4;  // +-4g
static const uint8_t FS_8G       	 = 0b10 << 4;  // +-8g
static const uint8_t FS_16G    		 = 0b11 << 4;  // +-16g
static const uint8_t HR        		 = 0b01 << 3;		// HR (High Resolution)
static const uint8_t OUT_X_L_A 		 = 0x28;	// base register - lower byte of X

// I2C addresses
static const uint8_t LA_ADDRESS= 0x32;	// i2c address of accelerometer
// I2C READING
uint8_t i2cData[6]; 		// Buffer for 6 8-bit registers



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void initPins()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

int readPinState(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

void writePinState(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState value)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, value);
}

void toggleLED()
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	HAL_Delay(500);
}

void activateLEDusingButton()
{
	if (readPinState(GPIOA, GPIO_PIN_1) == 1)
	{
		writePinState(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	} else
	{
		writePinState(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	}
}

// for reading multiple ADC channels, need to use DMA

// override function to set adcConversionComplete flag and end conversion
// _weak means that it can be overwritten
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adcConversionComplete = 1;
}

// In DMA settings - Memory checked -> increment memory on every writing
void potentiometer(uint16_t adcValue)
{
	// resolution is 12 bit, so we need 16 bits
	HAL_ADC_Start(&hadc);		//starts  ADS on STM

	if (HAL_ADC_PollForConversion(&hadc, 20) == HAL_OK)
	{		// wait until conversion is over
		uint16_t n = HAL_ADC_GetValue(&hadc);		// receiving 12 bits, so we use 16 bits
		// Convert to string and print
		sprintf((char*)buffer,"ADC Value: %hu\n", n);
		HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
	}
	HAL_Delay(500);
	HAL_ADC_Stop(&hadc);
}


float thermistor(uint16_t adcValue)
{
	float raw = (float) adcValue;
	raw = (raw / 4095.0) * 3.3;
	float Rt = 10000.0 * raw / (3.3 - raw);
	float beta = 4300.0;
	float R0 = 8000.0;
	float T0 = 298.15;	// T0 in Kelvin (25°C)

	float temp = beta / (log(Rt/R0) + beta/T0);
    float t = (temp - 273.15);

    return t;
}

void readMultipleADC()
{
	// hadc is adc converter
	HAL_ADC_Start_DMA(&hadc, (uint32_t) adcResultsDMA, adcChannelCount);

	// adcConversionComplete will be set in HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) function, that we are going to override

	while (adcConversionComplete == 0) {}

	adcConversionComplete = 0; 	// to be ready for next conversion

	// potentiometer -> pin D3
	// thermistor -> pin A0

	float temp = thermistor(adcResultsDMA[0]);		// thermistor has bigger priority (smaller pin value)
	sprintf((char*)buffer,"Temperature: %.2f, Potentiometer: %hu\n", temp, adcResultsDMA[1]);
	HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
	HAL_Delay(100);
}


// linear acceleration
//for Compass click - Mikroelektronika
// https://github.com/z49x2vmq/lsm303/blob/master/lsm303dlhc.h
//https://z49x2vmq.github.io/2018/05/26/lsm303-stm32-hal/
void lsm303dlhc_init_la()
{

	// 2-dim array for initialization
	// 1 row: 8 bit register address, 8 bit value to write on that address
	// IMPORTANT - initialize other control registers to 0
    uint8_t init[6][2] =
    {
        {CTRL_REG1_A, ODR_400Hz | Xen | Yen | Zen},			// 400Hz,Low Power,Xen, Yen, Zen activates sensor for all 3 axes
		{CTRL_REG2_A, 0},
		{CTRL_REG3_A, 0},
        {CTRL_REG4_A, FS_4G | HR},								// BDU (Block Data Update) - locks data until it is read, FS_4G - acceleration in ±4g, HR (High Resolution)
		{CTRL_REG5_A, 0},
		{CTRL_REG6_A, 0}
    };

    //sends first row of init
    if (HAL_I2C_Master_Transmit(&hi2c1, LA_ADDRESS, init[0], 2, HAL_MAX_DELAY) != HAL_OK)	// 2 is number of bytes that will be sent
    {
    	 return;
    }
    //sends second row of init
    if (HAL_I2C_Master_Transmit(&hi2c1, LA_ADDRESS, init[1], 2, HAL_MAX_DELAY) != HAL_OK)
    {
    	return;
    }
    //sends third row of init
	if (HAL_I2C_Master_Transmit(&hi2c1, LA_ADDRESS, init[2], 2, HAL_MAX_DELAY) != HAL_OK)
	{
		return;
	}
    //sends fourth row of init
	if (HAL_I2C_Master_Transmit(&hi2c1, LA_ADDRESS, init[3], 2, HAL_MAX_DELAY) != HAL_OK)
	{
		return;
	}
	//sends fifth row of init
	if (HAL_I2C_Master_Transmit(&hi2c1, LA_ADDRESS, init[4], 2, HAL_MAX_DELAY) != HAL_OK)
	{
		return;
	}
}


/*
Linear Acceleration Representation
+-----------------+-----------------+-----------------+
|        X        |        Y        |        Z        |
+-----------------+-----------------+-----------------+
| buf[0] | buf[1] | buf[2] | buf[3] | buf[4] | buf[5] |
+--------+--------+--------+--------+--------+--------+
| X_LOW  | X_HIGH | Y_LOW  | Y_HIGH | Z_LOW  | Z_HIGH |
+--------+--------+--------+--------+--------+--------+
*/

uint8_t* lsm303dlhc_read_la()
{
    uint8_t reg = OUT_X_L_A | 0b10000000;		// register from which we read data
    // 0b10000000 - sets biggetst bit on 1 -> activates auto-increment on reading

    //we say that we want to read data from address in reg in the future
    if (HAL_I2C_Master_Transmit(&hi2c1, LA_ADDRESS, &reg, 1, 1000) != HAL_OK)		// 1000 is timeout
    {
        return;
    }

    // receive data -> all three axes
    if (HAL_I2C_Master_Receive(&hi2c1, LA_ADDRESS, i2cData, 6, 1000) != HAL_OK)
    {
        return;
    }

    // process data
    int16_t x = i2cData[1] << 8 | i2cData[0];
    int16_t y = i2cData[3] << 8 | i2cData[2];
    int16_t z = i2cData[5] << 8 | i2cData[4];

    sprintf(buffer, "Linear Accelerometer (raw data): X value: %06d, Y value: %06d, Z value: %06d\n", x, y, z);
    HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);

    // Values:
	// when stanging ideal is x=0, y=0, z=8192 (2^13)
    // 16 represents raw value in range of ±4g
    // highest bit (15) is reserved for sign, and rest is value, so 2^13 is 1
	// ubrzanje u g je: a = sirova_vrednost/8192
	// ubrzanje u m/s2 je: a = g*9.81

    float x1 = x / 8192.0;
    x1 *= 9.81;
    float y1 = y / 8192.0;
	y1 *= 9.81;
	float z1 = z / 8192.0;
	z1 *= 9.81;

    sprintf(buffer, "Linear Accelerometer (in m/s^2): X value: %.2f, Y value: %.2f, Z value: %.2f\n\n", x1, y1, z1);
    HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
    HAL_Delay(200);

    uint8_t data[] = {(uint8_t)x1, (uint8_t)y1, (uint8_t)z1};

    return data;
}



// CAN KOMUNIKACIJA
CAN_TxHeaderTypeDef   TxHeader;
uint32_t              TxMailbox;

void CAN_Transmit(uint32_t id, uint8_t* data, uint8_t length)
{

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = length;
	TxHeader.ExtId = 0;

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK)
    {
        sprintf(buffer, "CAN Transmit Error\n");
        HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
    }
    else
    {
        sprintf(buffer, "CAN Transmit Successful:\n ID = %hu, data[0] = %d, data[1] = %d, data[2] = %d\n", TxHeader.StdId, data[0], data[1], data[2]);
        HAL_UART_Transmit(&huart2, buffer, strlen(buffer), HAL_MAX_DELAY);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
  HAL_ADC_Start_IT (&hadc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  initPins();

  //call only when Compass click is connected (when accelerometer is connected)
  //lsm303dlhc_init_la();

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  uint8_t TxData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  toggleLED();
	  //activateLEDusingButton();

	  // ADC READING - Potentiometer and Thermistor
	  //readMultipleADC();

	  // I2C reading of accelerometer
	  //lsm303dlhc_read_la();


	  //uint8_t* p = lsm303dlhc_read_la();

	  uint8_t data[] = {(uint8_t)1, (uint8_t)2, (uint8_t)3};
	  CAN_Transmit(0x123, TxData, 8);
//	  CAN_Transmit(0x123, &data, 1);
	  HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */


  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

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
