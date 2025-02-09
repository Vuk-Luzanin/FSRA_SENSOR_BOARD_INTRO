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
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// adresa LCD ekrana na I2C magistrali
#define SLAVE_ADDRESS_LCD 0x4E


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
CAN_FilterTypeDef canfilterconfig;
void configureFilter(){
	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x123<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x123<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
}



CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];

int datacheck = 0;
char buffer[200];
char str[40];

void UART_Print(char *c)
{
	sprintf(buffer, c);
	HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);
	HAL_Delay(1000);
	return;
}



// za prekid - rutina
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		UART_Print("CAN Error \n");
		Error_Handler();
	}
	if ((RxHeader.StdId == 0x123))
	{
		datacheck = 1;
	}
	//UART_Print((char*)RxHeader.StdId);
	UART_Print("RADI \n");

	return;
}


// za slanje jednog karaktera u 2 ciklusa, jer lcd radi sa 4bitnim vrednostima
void lcd_send_data(char data)
{
	// we want to show data byte on display
    char data_h, data_l;
    uint8_t data_t[4];

    // xxxx0000
    data_h = (data & 0xF0);   		// gornja 4 bita
    data_l = ((data << 4) & 0xF0);

    // OR with control bits
    // EN=1, RS=1, RW=0 - Aktivacija LCD-a za primanje podataka
    // EN=0, RS=1, RW=0 - Deaktivacija LCD-a nakon prijema
    // kada en predje sa 1 na 0, lcd prima podatke
    // rs = 1 -> salju se podaci, rs = 0 znaci da se salje komanda
    data_t[0] = data_h | 0x0D; 		// en=1, rs=1
    data_t[1] = data_h | 0x09; 		// en=0, rs=1
    data_t[2] = data_l | 0x0D; 		// en=1, rs=1
    data_t[3] = data_l | 0x09; 		// en=0, rs=1

    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100) ;
}

// za slanje komandi
void lcd_send_cmd(char cmd)
{
    char data_u, data_l;
    uint8_t data_t[4];

    data_u = (cmd & 0xF0);
    data_l = ((cmd << 4) & 0xF0);

    // salju se prvo visi biti, pa nizi
    data_t[0] = data_u | 0x0C; // en=1, rs=0
    data_t[1] = data_u | 0x08; // en=0, rs=0
    data_t[2] = data_l | 0x0C; // en=1, rs=0
    data_t[3] = data_l | 0x08; // en=0, rs=0

    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_init(void) {
    // 4-bit initialization - first set to 8bit - three times as said in datasheet
    HAL_Delay(50); // wait for >40ms to stable voltage
    lcd_send_cmd(0x30);
    HAL_Delay(5); // wait for >4.1ms -> because instructions are slow
    lcd_send_cmd(0x30);
    HAL_Delay(1); // wait for >100us
    lcd_send_cmd(0x30);
    HAL_Delay(10);
    lcd_send_cmd(0x20); // 4-bit mode
    HAL_Delay(10);

    // Display initialization
    // pauze su tu jer ove komande zahtevaju vise vremena
    lcd_send_cmd(0x28); // Function set --> DL=0 (4-bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
    HAL_Delay(1);
    lcd_send_cmd(0x08); // Display on/off control --> D=0 - iskljucuje ekran, C=0 - iskljucuje kursor, B=0 - iskljucuje treperenje kursora
    HAL_Delay(1);
    lcd_send_cmd(0x01); // Clear display
    HAL_Delay(1);
    lcd_send_cmd(0x06); // Entry mode set --> I/D = 1 (increment cursor - ide desno nakon svakog karaktera) & S = 0 (no shift- da se display ne pomera, vec samo kursor)
    HAL_Delay(1);
    lcd_send_cmd(0x0F); // 0x0C - Display on - samo ukljuci ekran ( 0x0E za kursor i 0x0F za kursor koji treperi)
}

void lcd_clear()
{
	lcd_send_cmd(0x80);
	for (int i=0; i<70; i++)
	{
		lcd_send_data(' ');
	}
}

void lcd_send_string(char *str)
{
    while (*str) lcd_send_data(*str++);
}

void setCursor(int a, int b)
{
    switch (b)
    {
        case 0: b = 0x80; break;
        case 1: b = 0xC0; break;
    }
    lcd_send_cmd(b);
    lcd_send_cmd(a);
}

void writeOnLCD(float aCCx, float aCCy, float aCCz)
{
	sprintf(buffer, "CAN X: %.2f", aCCx);
	setCursor(0, 0);
    lcd_send_string(buffer);
    HAL_Delay(500);
    sprintf(buffer, "CAN Y: %.2f", aCCy);
	setCursor(0, 1);
	lcd_send_string(buffer);
	HAL_Delay(500);
    sprintf(buffer, "CAN Z: %.2f", aCCz);
    setCursor(0, 0);
    lcd_send_string(buffer);
    HAL_Delay(500);
    lcd_clear();
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	// dokumentacija za plocu
	//https://www.st.com/resource/en/user_manual/um1974-stm32-nucleo144-boards-mb1137-stmicroelectronics.pdf

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  configureFilter();


  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	  Error_Handler();
	  UART_Print("CAN Error Starting \n");
  }
  else
  {
	  UART_Print("CAN Started \n");
  }

//  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
//	{
//	  Error_Handler();
//	  UART_Print("CAN Error Interrupts \n");
//	}
//  else{
//	  UART_Print("CAN Interrupts Enabled \n");
//  }

  lcd_init();
  // video for LCD display connected to i2c prootcol
  // https://www.youtube.com/watch?v=rfRJGfK2t-A
  // documentation for display
  // https://www.openhacks.com/uploadsproductos/eone-1602a1.pdf
  // https://www.ti.com/lit/ds/symlink/pcf8574.pdf

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // write LCD
//	  setCursor(0, 0);
//	  lcd_send_string("aaa");
//	  HAL_Delay(1000);
//	  setCursor(0, 1);
//	  lcd_send_string("bbbb");
//	  HAL_Delay(1000);

	  // busy wait
	  while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0)
	  {
		  UART_Print("CAN Waiting \n");
	  };

	  if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0)
	  {
		  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		  {
			UART_Print("CAN Error \n");
			Error_Handler();
		  }
		  else
		  {
			  int16_t tmp = ((uint16_t) RxData[1] << 8) | RxData[0];
			  float aCCx = (float)(tmp) / 100.0f;
			  tmp = ((uint16_t) RxData[3] << 8) | RxData[2];
			  float aCCy = (float)(tmp) / 100.0f;
			  tmp = ((uint16_t) RxData[5] << 8) | RxData[4];
			  float aCCz = (float)(tmp) / 100.0f;

			  writeOnLCD(aCCx, aCCy, aCCz);
			  sprintf(buffer,
			      "----------------\n"
			      "CAN Data[0]: %d\n"
			      "CAN Data[1]: %d\n"
			      "CAN Data[2]: %d\n"
			      "CAN Data[3]: %d\n"
			      "CAN Data[4]: %d\n"
			      "CAN Data[5]: %d\n"
			      "CAN Data[6]: %d\n"
			      "CAN Data[7]: %d\n"
				  "CAN X: %.2f\n"
				  "CAN Y: %.2f\n"
				  "CAN Z: %.2f\n",
			      (uint8_t) RxData[0],
			      (uint8_t) RxData[1],
			      (uint8_t) RxData[2],
			      (uint8_t) RxData[3],
			      (uint8_t) RxData[4],
			      (uint8_t) RxData[5],
			      (uint8_t) RxData[6],
			      (uint8_t) RxData[7],
				  aCCx, aCCy, aCCz);

			  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);
		  }
	  }

	  // za prekid
//	  while(datacheck == 0){
//		  UART_Print("CAN Waiting \n");
//	  }
//	  if(datacheck == 1) {
//		  UART_Print("CAN Works \n");
//	  }
	  HAL_Delay(500);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
