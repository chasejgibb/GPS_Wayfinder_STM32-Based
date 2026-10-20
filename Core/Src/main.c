/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <math.h>
#include <stdlib.h>
#include <string.h>

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
#define MAX_MESSAGE_SIZE 200 // max size for gpsMessage
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
double homeX    = 0.0; // variables for grid system and position tracking
double homeY    = 0.0;
double posXLim = 0.0;
double negXLim = 0.0;
double posYLim = 0.0;
double negYLim = 0.0;
double currX    = 0.0;
double currY    = 0.0;
double rawLat = 0.0;
double rawLon = 0.0;
double finalLat = 0.0;
double finalLon = 0.0;

uint8_t gpsByte; // byte from GPS
uint8_t gpsMessage[MAX_MESSAGE_SIZE] = {0}; // array we will put the gpsBytes into and parse information from
uint8_t gpsBufferPosition = 0;
char gpsX[16] = {0}; // we will parse the latitude and longitude values from gpsMessage and place them into here for further analysis
char gpsY[16] = {0}; // after getting these values, we will convert them to doubles and use them in getGPSX and getGPSY


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
double GetGPSX(void); // declare our functions
double GetGPSY(void);
void MakeMap(void);
void SetHome(void);
void MoveCharacter(void);
void BlinkLEDs(void);
double NMEAToDecimal(double coordinate);
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
  HAL_Init();

  SystemClock_Config();
  PeriphCommonClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init(); // start ADC's
  MX_ADC2_Init();
  MX_TIM2_Init(); // start timer 2
  MX_USART1_UART_Init(); // start USART1
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // timer channels for LEDs
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim2); // start timer 16
  HAL_UART_Receive_IT(&huart1, &gpsByte, 1); // put byte received from USART2 into "gpsByte"

  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */
    if (HAL_GPIO_ReadPin(JOY_PUSH_GPIO_Port, JOY_PUSH_Pin) == 0){ // if joy stick button is pushed, will set new home position
    	SetHome();
    	HAL_Delay(10); // delay so you don't set home multiple times on one push
    }
    MoveCharacter(); // first you move, then the system will tell you your location after your movement
    BlinkLEDs();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */
  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_HOME_GPIO_Port, LED_HOME_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : JOY_PUSH_Pin */
  GPIO_InitStruct.Pin = JOY_PUSH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(JOY_PUSH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_HOME_Pin */
  GPIO_InitStruct.Pin = LED_HOME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_HOME_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// This function runs automatically every time one byte is received through UART1
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) // if the interrupt came through UART1, continue
    {
        gpsMessage[gpsBufferPosition] = gpsByte;   // add to gpsMessage arrray
        gpsBufferPosition++;					   // increment buffer position to prepare for next byte
        HAL_UART_Receive_IT(&huart1, &gpsByte, 1); // restart the interrupt reception so the next byte can be received
    }
}

double GetGPSX(void){
	int commaCount = 0;           // Parse data from string received from GPS based on commas to find latitude and longitude
	int buffer = 0;     // use this buffer to keep track the position of what we're reading from gpsMessage
	gpsX[0] = '\0'; // clear array from previous usage

	// we will go through the whole gpsMessage until we find 2 commas (which is when latitude begins) and a 'N' or 'S' character
	for (int i = 0; i < gpsBufferPosition; i++){
		char currentChar = gpsMessage[i];

		if (currentChar == 'N' || currentChar == 'S'){ // latitude message is finished, return results
			gpsX[buffer] = currentChar;// finish the message
			buffer++;
			gpsX[buffer] = '\0'; // end message
			rawLat = strtod(gpsX, NULL); // convert string to number
			finalLat = NMEAToDecimal(rawLat); // convert NMEA degrees/minutes format to decimal degrees

			if (gpsX[buffer - 1] == 'S') // southern latitudes changes to negative
				{
					finalLat = -finalLat;
				}
			return finalLat;
		}
		if (commaCount == 2){ // after 2 commas, latitude begins
			gpsX[buffer] = currentChar;
			buffer++;
		}

		if (currentChar == ','){ // increment commaCount if comma is found
			commaCount++;
		}
	}
	return 1.0; // return a basic number if gpsMessage is not filled with proper data. This is here because the module broke
}
//
// see GetGPSX for more detailed comments on this function. The same rules apply. The only difference is this is longitude and looks for 4 commans, not 2
//      VVVVV VVVVV VVVVV VVVVV
double GetGPSY(void){
	int commaCount = 0;           // Parse data from string received from GPS based on commas to find latitude and longitude
	int buffer = 0;
	gpsY[0] = '\0'; // clear array
	for (int i = 0; i < gpsBufferPosition; i++){
		char currentChar = gpsMessage[i];
		if (currentChar == 'W' || currentChar == 'E'){ // longitude message is finished, return results
			gpsY[buffer] = currentChar;
			buffer++;
			gpsY[buffer] = '\0';// finish the message
			rawLon = strtod(gpsY, NULL);
			finalLon = NMEAToDecimal(rawLon);
			if (gpsY[buffer - 1] == 'W')
			{
			    finalLon = -finalLon;
			}
			return finalLon;
		}
		if (commaCount == 4){ // after 4 commas, longitude begins
			gpsY[buffer] = currentChar;
			buffer++;
		}

		if (currentChar == ','){ // increment commaCount if comma is found
			commaCount++;
		}
	}
	return 2.0;// return a basic number if gpsMessage is not filled with proper data. This is here because the module broke
}

// Convert NMEA format to decimal degrees. Example: 4325.5000 becomes 43.425 degrees
double NMEAToDecimal(double coordinate)
{
    int degrees = (int)(coordinate / 100.0); // Make first 2 digits the degrees
    double minutes = coordinate - (degrees * 100.0); // make the minutes the remaining decimal portion
    return degrees + (minutes / 60.0); // convert minutes to degrees and combine them
}

// make map boundaries for character to move in. 100 is just an arbitrary value that works well with the speed of our characters movement
void MakeMap(void){
	posXLim = homeX + 100;
	negXLim = homeX - 100;
	posYLim = homeY + 100;
	negYLim = homeY - 100;
}

// set your waypoint of which you will be shown where to return to
void SetHome(void){
	homeX = GetGPSX(); // get home coords
	homeY = GetGPSY();
	currX = homeX;     // set characters starting point at home
	currY = homeY;
	MakeMap();
	// reset buffer position and arrays after gpsMessage is compiled and read
	gpsBufferPosition = 0;
	memset(gpsX, 0, sizeof(gpsX));
	memset(gpsY, 0, sizeof(gpsY));
	memset(gpsMessage, 0, sizeof(gpsMessage));
}

void MoveCharacter(void)
{
    const double ySTEP_SIZE = 0.001; // maximum amount the character can move in one function call
    const double xSTEP_SIZE = 0.001;
    const double CENTER = 1700;      // defined center ADC value of joystick x and y. used to avoid deadzone

    HAL_ADC_Start(&hadc1);           // read the joystick's x-axis voltage using ADC1
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t xjoyStep = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Start(&hadc2);           // read the joystick's y-axis voltage using ADC2
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
    uint16_t yjoyStep = HAL_ADC_GetValue(&hadc2);

    float xMovePercent = 0; // used to find percentage of maximum movement in each direction
    float yMovePercent = 0;

// THE FOLLOWING COMMENTS ARE VALID WHEN THE JOYSTICK WIRES ARE TO THE LEFT

    if (xjoyStep < 1500 || xjoyStep > 1900){ // only move horizontally when joystick is out of deadzone
    	if (xjoyStep < 1500){
    		xMovePercent = -(CENTER - xjoyStep)/1700;// MOVE WEST
    	}
    	if (xjoyStep > 1900){
    		xMovePercent = (xjoyStep - CENTER)/1700;// MOVE EAST
		}
    }

    if (yjoyStep < 1500 || yjoyStep > 2000){ // only move vertically when joystick is out of deadzone
		if (yjoyStep > 2000){
			yMovePercent = (CENTER - yjoyStep)/1700;// MOVE SOUTH
		}
		if (yjoyStep < 1500){
			yMovePercent = -(yjoyStep - CENTER)/1700; // MOVE NORTH
		}
	}
    // calculate the next new character position
    float nextX = currX + (xMovePercent * xSTEP_SIZE);
    float nextY = currY + (yMovePercent * ySTEP_SIZE);
    // check if the next position is out of bounds or not. If not out of bounds, will update current position to new position.
    if (nextX < posXLim && nextX > negXLim)
    {
    	currX = nextX;
    }

    if (nextY < posYLim && nextY > negYLim)
	{
		currY = nextY;
	}
    //HAL_Delay(20);
}

void BlinkLEDs(void)
{
	// TIM2 counts to 1000 so 999 is about 100% of duty cycle
    const int PWM_PERIOD = 999;
    // turn off all compass LEDs before finding new brightness levels
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;

    // If character is home, turn on on-board LED
    if ((currY > homeY - 5) && (currY < homeY + 5) &&
        (currX > homeX - 5) && (currX < homeX + 5))
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        return;
    }
    // if not home, turn off on-board LED
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    // find distance from home
    double xError = currX - homeX;
    double yError = currY - homeY;

    // convert distance into a duty cycle of 1 and 0
    float dutyCycleX = fabs(xError) / 100.0f;
    float dutyCycleY = fabs(yError) / 100.0f;

    // limit them to a maximum of 1
    if (dutyCycleX > 1.0f) dutyCycleX = 1.0f;
    if (dutyCycleY > 1.0f) dutyCycleY = 1.0f;

    // find the brightness levels of the horizontal and vertical LEDs
    int resetValueX = (int)(dutyCycleX * PWM_PERIOD);
    int resetValueY = (int)(dutyCycleY * PWM_PERIOD);

    // if character is not within 5 units of home, turn on LEDs to the brightness level just found based on direction
    if (currY > homeY + 5)
    {
        TIM2->CCR3 = resetValueY; // SOUTH
    }
    if (currY < homeY - 5)
    {
        TIM2->CCR1 = resetValueY; // NORTH
    }
    if (currX > homeX + 5)
    {
        TIM2->CCR4 = resetValueX; // WEST
    }
    if (currX < homeX - 5)
    {
        TIM2->CCR2 = resetValueX; // EAST
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
