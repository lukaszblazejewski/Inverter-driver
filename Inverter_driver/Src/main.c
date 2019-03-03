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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "Functions.h"
#include <math.h>
#include "i2c-lcd.h"
#include "stdlib.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

///////////////////////////////////////
double  Rampa[120] = {0}, Rampa_U_do_F[120] = {0};
float boost_old = 99;

///////////////////////////////////////
volatile uint16_t pulse_count, positions;

float 	ma = 0, ma_old = 0, f = 0, f_old = 0, TIMEtoRAMPup = 5, TIMEtoRAMPdown = 5, deltaf = 0.1, boost = 0.25, RampL[501] = {0}, RampG[501] = {0};
int 	Usin[120] = {0}, UsinRAMPA[120] = {0}, Timer[501] = {0}, n1 = 0, fpom = 0,
		kierunekzad = 0, kierunek = 0, ksztalt = 0, fzad = 0, fzad_old = 0,
		ZP1 = 0, ZP2 = 0, ZP3 = 0, ZP4 = 0, y2 = 0, start = 0, y125 = 0, m = 0, n = 0;
uint8_t Received[8];

uint16_t Pomiary[3] = {0};
float Temp = 0, DCI = 0, DCV = 0, TABDCI[150] = {0};

char wartosc[5];

int menu[4] = {0,0,1,0};
int old_menu  =  0;
int old_menu_1 = 0;
int old_positions = 0;

float param[27];
float param_z = 0;

int przepisanie = 1;
int zapisano = 0;
int status = 0;
int czysc_ekran = 0;
int keylock = 0;

float min = 0, max = 0;
int res = 0;

float f_out = 0;
float f_zad = 0;

// wartosci parametrow
// kolejne elementy tablicy:
// nr parametru, wartosc domyslna, inkrementacja, min, max
float values[135] = {
				0, 3, 1, 1, 100,		// P101 czas rampy naras.
				1, 3, 1, 1, 100,		// P102 czas rampy opad.
				2, 0, 1, 0, 1,			// P103 typ rampy naras.
				3, 0, 1, 0, 1,			// P104 typ rampy opad.
				4, 0, 1, 0, 1,			// P105 tryb zatrzymania
				5, 100, 1, 50, 100,  	// P106 ograniczenie pradu
				6, 25, 1, 0, 100,		// P107 boost napieciowy
				7, 0, 1, 0, 1,			// P108 ksztalt U/f
				8, 1500, 50, 100, 3000,	// P109 predkosc obrotowa
				9, 0, 10, 0, 400,		// P110 napiecie
				10, 0, 1, 0, 50,   		// P111 f zadane
				11, 1, 0.1, 0.1, 1,   	// P112 inkremenctacja f zadane
				12, 0, 1, 0, 1,			// P113 kierunek obrotu
				13, 1, 0.1, 0.1, 1, 	// P201 moc silnika znam
				14, 400, 170, 230, 400,	// P202 napiecie znamionowe
				15, 3, 0.1, 0.1, 3.5,	// P203 prad znamionowy
				16, 1500, 50, 100, 3000,// P204 predkosc obrotowa znamionowa
				17, 1, 1, 0, 10,		// P205 Rs rezystancja stojana
				18, 1, 1, 0, 10,		// P206 Rr rezystancja wirnika
				19, 1, 1, 0, 10,		// P207 Ls indukcyjnosc rozproszzenia stojana
				20, 1, 1, 0, 10,		// P208 Lr indukcyjnosc rozproszenia wirnika
				21, 1, 1, 0, 10,		// P209 Lm indukcyjnosc magnesowania
				22, 2, 1, 1, 3, 		// P210 p liczba par biegunow
				23, 50, 1, 0, 50,		// P211 f znam.
				24, 0, 1, 0, 49,		// P212 f min
				25, 50, 1, 0, 50,		// P213 f max
				26, 0.80, 0.01, 0.50, 1 // P214 cosinus fi
};

/////////////////drganie stykow/////////////////////////
union {		//do obs³ugi przycisków -> w przerwaniu ISR
 struct {
  unsigned char Button:1;	//LSB
  unsigned char ENC_Button:1;
  unsigned char Button_1:1;
  unsigned char Button_2:1;
  unsigned char Button_PREV:1;
  unsigned char ENC_Button_PREV:1;
  unsigned char Button_1_PREV:1;
  unsigned char Button_2_PREV:1;
 } flags;
  char byte;
} key;

union {		//do obs³ugi przycisków -> w przerwaniu ISR
 struct {
  unsigned char Button:1;	   //LSB
  unsigned char ENC_Button:1;
  unsigned char Button_1:1;
  unsigned char Button_2:1;
  unsigned char Button_PREV:1;
  unsigned char ENC_Button_PREV:1;
  unsigned char Button_1_PREV:1;
  unsigned char Button_2_PREV:1;
  unsigned char Button_PREV1:1;
  unsigned char ENC_Button_PREV1:1;
  unsigned char Button_1_PREV1:1;
  unsigned char Button_2_PREV1:1;
  unsigned char Button_PREV2:1;
  unsigned char ENC_Button_PREV2:1;
  unsigned char Button_1_PREV2:1;
  unsigned char Button_2_PREV2:1;
  unsigned char Button_PREV3:1;
  unsigned char ENC_Button_PREV3:1;
  unsigned char Button_1_PREV3:1;
  unsigned char Button_2_PREV3:1;
  unsigned char Button_PREV4:1;
  unsigned char ENC_Button_PREV4:1;
  unsigned char Button_1_PREV4:1;
  unsigned char Button_2_PREV4:1;
  unsigned char Button_PREV5:1;
  unsigned char ENC_Button_PREV5:1;
  unsigned char Button_1_PREV5:1;
  unsigned char Button_2_PREV5:1;
  unsigned char Button_PREV6:1;
  unsigned char ENC_Button_PREV6:1;
  unsigned char Button_1_PREV6:1;
  unsigned char Button_2_PREV6:1;
  } flags;
  int word;
} falling_slope;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);
void clr_scr();
void ustaw_parametr(char param[], int resolution, char unit[]);
void pokaz_param(char nr_parametru[], char description[], int resolution);
void set_default(int il_param);
void enkoder();
void menu_LCD();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM10_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim10);

  set_default(27);
  TworzenieSIN(Usin, 120);
  SkalowanieF(Timer, 500, 68);
  Tworzenie_Rampy(Rampa, &boost_old, 120);
  RampaGladkaMA(boost, RampG);
  RampaLiniowaMA(boost, RampL);


  lcd_init();
  HAL_Delay(5);

  HAL_ADC_Start_DMA(&hadc1, Pomiary, 3);
  HAL_UART_Receive_DMA(&huart6, Received, 8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(keylock == 1 && HAL_GPIO_ReadPin(ENC_Button_GPIO_Port, ENC_Button_Pin) == GPIO_PIN_RESET)
		  keylock = 0;
	  if(keylock == 1 && HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_RESET)
		  keylock = 0;

	  pulse_count = TIM3 -> CNT;
	  positions = pulse_count / 2;

	  if((param[10] > 0) && (f == 0))
	  {
		  HAL_GPIO_WritePin(RESET_SIGNAL_GPIO_Port, RESET_SIGNAL_Pin, SET);
		  HAL_Delay(1);
		  HAL_GPIO_WritePin(RESET_SIGNAL_GPIO_Port, RESET_SIGNAL_Pin, RESET);
		  HAL_Delay(1);
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		  start = 1;
	  }

	  if((param[10] == 0) && ( f== 0))
	  {
		  start = 0;
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
	  }

	  clr_scr();
	  enkoder();
	  menu_LCD();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }

  return 0;
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 139;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 200;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 201;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 199;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RESET_SIGNAL_Pin|ENABLE_SIGNAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RESET_SIGNAL_Pin ENABLE_SIGNAL_Pin */
  GPIO_InitStruct.Pin = RESET_SIGNAL_Pin|ENABLE_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_Pin Button_2_Pin Button_1_Pin */
  GPIO_InitStruct.Pin = Button_Pin|Button_2_Pin|Button_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_Button_Pin */
  GPIO_InitStruct.Pin = ENC_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_Button_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void clr_scr()
{
	if((old_positions != positions) || czysc_ekran == 1)
	{
		// czyszczenie ekranu po ruchu enkodera
		HAL_Delay(1);
		lcd_send_cmd(0x01);
		HAL_Delay(2);
		czysc_ekran = 0;
	}
}

void pokaz_param(char nr_parametru[], char description[], int resolution)
{
	lcd_send_cmd(0x80);
	lcd_send_string(nr_parametru);
	lcd_send_cmd(0x87);
	lcd_send_string("val:");
	strcpy(wartosc, " ");
	ftoa(param[menu[2] - 1], wartosc, resolution);
	lcd_send_string(wartosc);
	lcd_send_cmd(0xc0);
	lcd_send_string(description);
}

void ustaw_parametr(char param[], int resolution, char unit[])
{
	lcd_send_cmd(0x80);
	lcd_send_string(param);
	ftoa(param_z, wartosc, resolution);
	res = resolution;
	lcd_send_string(wartosc);
	lcd_send_cmd(0xcA);
	lcd_send_string(unit);
	min=values[3 + ((menu[2] - 1) * 5)];
	max=values[4 + ((menu[2] - 1) * 5)];
}

void set_default(int il_param)
{
  //wpisanie wartosci domyslnych
	// il_param - ilosc parametrow
	for(int i = 0; i <= il_param - 1; i++){
	  param[i] = values[1 + (i * 5)];
  }
}

void enkoder()
{
	  //wykrywanie obrotu enkodera lewo/prawo

	  if(menu[0] == 0 && old_positions > positions)
	  {
 		  menu[1]--;
 		  if(old_positions == 100 && positions == 0)
 			  menu[1]--;
	  }


	  else if(menu[0] == 0 && old_positions < positions)
	  {
 		  menu[1]++;
 		  if(old_positions == 0 && positions == 100)
 			  menu[1]++;
	  }

	  if(menu[1] < 0) menu[1] = 1;
	  if(menu[1] > 1) menu[1] = 0;

	  if(menu[0] == 1 && old_positions > positions)
	  {
 		  menu[2]--;
 		  if(old_positions == 100 && positions == 0)
 			  menu[2] = menu[2] + 2;
	  }


	  else if(menu[0] == 1 && old_positions < positions)
	  {
 		  menu[2]++;
 		  if(old_positions == 0 && positions == 100)
 			  menu[2] = menu[2] - 2;
	  }


	  if(menu[2] < 1) menu[2] = 1;
	  if(menu[2] > 26) menu[2] = 26;

	  if(menu[0] == 2 && old_positions > positions)
	  {
		  	  if(menu[2] == 11)
		  		  param_z = param_z - param[11];
		  	  else
	 		  param_z = param_z - values[2 + (5 * (menu[2] - 1))];
	 		  if(old_positions == 100 && positions == 0)
	 			  param_z = param_z + 2 * values[2 + (5 * (menu[2] - 1))];
	 		  if(param_z < min) param_z = min;
	  }

	  else if(menu[0] == 2 && old_positions < positions)
	  {
	  	  	  if(menu[2] == 11)
	  	  		  param_z = param_z + param[11];
	  	  	  else
		  	  param_z = param_z + values[2 + (5 * (menu[2] - 1))];
	 		  if(old_positions == 0 && positions == 100)
	 			  param_z = param_z - 2 * values[2 + (5 * (menu[2]-1))];
	 		  if(param_z > max) param_z = max;

	  }

	  if(menu[0] == 3 && old_positions > positions)
	  {
 		  f_zad--;
 		  if(old_positions == 100 && positions == 0)
 			  f_zad = f_zad + 2;
	  }


	  else if(menu[0] == 3 && old_positions < positions)
	  {
 		  f_zad++;
 		  if(old_positions == 0 && positions == 100)
 			  f_zad = f_zad - 2;
	  }
	  if(f_zad < param[24]) f_zad = param[24];
	  if(f_zad > param[25]) f_zad = param[25];


	  old_positions = positions;


	  // zapis nastawionej wartosci parametru(nie da sie w przerwaniu tego zrobic bo przerwanie nie ogarnia)
	  if(zapisano == 1)
	  {
		  lcd_send_cmd(0x80);
		  lcd_send_string("    Zapisano!");
		  param[old_menu_1 - 1] = param_z;
		  lcd_send_cmd(0xc0);
		  ftoa(param[old_menu_1 - 1], wartosc, res);
		  lcd_send_string(" value= ");
		  lcd_send_string(wartosc);
		  HAL_Delay(1500);
		  czysc_ekran = 1;
		  menu[0] = 1;
		  HAL_Delay(20);
		  menu[2] = old_menu_1;
		  zapisano = 0;
	  }

}


void menu_LCD()
{
	  // realizacja kolejnych stopni menu
	  if(menu[0] == 0)
	  {

		  switch(menu[1])
		  {
		  	  case 0:
		  	  {
		  		lcd_send_cmd(0x80);
		  		lcd_send_string(">");
		  		break;
		  	  }
		  	  case 1:
		  	  {
		  		lcd_send_cmd(0xc0);
		  		lcd_send_string(">");
		  		break;
		  	  }
		  }
		  przepisanie = 1;
		  lcd_send_cmd(0x81);
		  lcd_send_string("Menu   Stan:");
		  if(status == 0) lcd_send_string("WYL");
		  if(status == 1) lcd_send_string("WL");
		  lcd_send_cmd(0xc1);
		  lcd_send_string("Pomiary");

	  }
	  // dzialanie lokalne(silnik ON)
	  if(menu[0] == 3)
	  {

		  if(przepisanie == 1){
			  f_zad = param[10];
			  f_out = f_zad;
			  przepisanie = 0;
		  }
		  strcpy(wartosc, " ");
		  lcd_send_cmd(0x80);
		  lcd_send_string("f_zad=");
		  ftoa(f_zad, wartosc, 0);
		  lcd_send_string(wartosc);
		  lcd_send_string(" Hz");
		  lcd_send_cmd(0x8d);
		  lcd_send_string("WL");
		  lcd_send_cmd(0xc0);
		  strcpy(wartosc, " ");
		  f_out = fpom / 10;
		  ftoa(f_out, wartosc, 0);
		  lcd_send_string("f_out=");
		  lcd_send_string(wartosc);
		  lcd_send_string(" Hz");
	  }

	  // pomiary

	  if(menu[0]==4)
	  {
		  lcd_send_cmd(0x80);
		  lcd_send_string("T:");
		  ftoa(Temp, wartosc, 0);
		  lcd_send_string(wartosc);
		  lcd_send_string("C");
		  lcd_send_cmd(0xc0);
		  lcd_send_string("DCV:");
		  ftoa(DCV, wartosc, 0);
		  lcd_send_string(wartosc);
		  lcd_send_string("V");
		  lcd_send_cmd(0x87);
		  lcd_send_string("DCI:");
		  ftoa(fabs(DCI), wartosc, 0);
		  lcd_send_string(wartosc);
		  lcd_send_string("mA");
	  }

	  if(menu[0] == 1)
	  {
		  przepisanie = 1;
		  switch(menu[2])
		  {
			  case 1:
			  {
				  pokaz_param("P101", "t rampy nar.", 0);
				  break;
			  }
			  case 2:
			  {
				  pokaz_param("P102", "t rampy opad.", 0);
				  break;
			  }
			  case 3:
			  {
				  pokaz_param("P103", "Rampa nar.", 0);
				  break;
			  }
			  case 4:
			  {
				  pokaz_param("P104","Rampa opad.",0);
				  break;
			  }
			  case 5:
			  {
				  pokaz_param("P105","Tryb zatrzym.",0);
				  break;
			  }
			  case 6:
			  {
				  pokaz_param("P106","Ogr. pradu",0);
				  break;
			  }
			  case 7:
			  {
				  pokaz_param("P107","Boost nap.",0);
				  break;
			  }
			  case 8:
			  {
				  pokaz_param("P108","Ksztalt U/f",0);
				  break;
			  }
			  case 9:
			  {
				  pokaz_param("P109","Pr. obrot. zad",0);
				  break;
			  }
			  case 10:
			  {
				  pokaz_param("P110","Napiecie zad",0);
				  break;
			  }
			  case 11:
			  {
				  pokaz_param("P111","f zadane",1);
				  break;
			  }
			  case 12:
			  {
				  pokaz_param("P112","inkr. f zad",1);
				  break;
			  }
			  case 13:
			  {
				  pokaz_param("P113","Kier. obrotu",0);
				  break;
			  }
			  case 14:
			  {
				  pokaz_param("P201","Moc znam.",1);
				  break;
			  }
			  case 15:
			  {
				  pokaz_param("P202","U znam.",0);
				  break;
			  }
			  case 16:
			  {
				  pokaz_param("P203","Prad znam.",1);
				  break;
			  }
			  case 17:
			  {
				  pokaz_param("P204","Pred. znam.",0);
				  break;
			  }
			  case 18:
			  {
				  pokaz_param("P205","Rs",0);
				  break;
			  }
			  case 19:
			  {
				  pokaz_param("P206","Rr",0);
				  break;
			  }
			  case 20:
			  {
				  pokaz_param("P207","Ls",0);
				  break;
			  }
			  case 21:
			  {
				  pokaz_param("P208","Lr",0);
				  break;
			  }
			  case 22:
			  {
				  pokaz_param("P209","Lm",0);
				  break;
			  }
			  case 23:
			  {
				  pokaz_param("P210","par biegunow",0);
				  break;
			  }
			  case 24:
			  {
				  pokaz_param("P211","f znam.",0);
				  break;
			  }
			  case 25:
			  {
				  pokaz_param("P212","f min",0);
				  break;
			  }
			  case 26:
			  {
				  pokaz_param("P213","f max",0);
				  break;
			  }
			  case 27:
			  {
				  pokaz_param("P214","cos fi",2);
				  break;
			  }
		  }
	  }

	  if(menu[0] == 2)
	  {
		  old_menu_1 = menu[2];
		  if(przepisanie == 1)
		  {
		  param_z = param[old_menu_1 - 1];
		  przepisanie = 0;
		  }
		  switch(old_menu_1)
		  {
			  case 1:
			  {
				  ustaw_parametr("Ustaw P101:",0,"[s]");
				  break;
			  }
			  case 2:
			  {
				  ustaw_parametr("Ustaw P102:",0,"[s]");
				  break;
			  }
			  case 3:
			  {
				  ustaw_parametr("Ustaw P103:",0,"");
				  break;
			  }
			  case 4:
			  {
				  ustaw_parametr("Ustaw P104:",0,"");
				  break;
			  }
			  case 5:
			  {
				  ustaw_parametr("Ustaw P105:",0,"");
				  break;
			  }
			  case 6:
			  {
				  ustaw_parametr("Ustaw P106:",0,"[%]");
				  break;
			  }
			  case 7:
			  {
				  ustaw_parametr("Ustaw P107:",0,"[%]");
				  break;
			  }
			  case 8:
			  {
				  ustaw_parametr("Ustaw P108:",0,"");
				  break;
			  }
			  case 9:
			  {
				  ustaw_parametr("Ustaw P109:",0,"[obr/m]");
				  break;
			  }
			  case 10:
			  {
				  ustaw_parametr("Ustaw P110:",0,"[V]");
				  break;
			  }
			  case 11:
			  {
				  ustaw_parametr("Ustaw P111:",1,"[Hz]");
				  break;
			  }
			  case 12:
			  {
				  ustaw_parametr("Ustaw P112:",1,"");
				  break;
			  }
			  case 13:
			  {
				  ustaw_parametr("Ustaw P113:",0,"");
				  break;
			  }
			  case 14:
			  {
				  ustaw_parametr("Ustaw P201:",1,"[kW]");
				  break;
			  }
			  case 15:
			  {
				  ustaw_parametr("Ustaw P202:",0,"[V]");
				  break;
			  }
			  case 16:
			  {
				  ustaw_parametr("Ustaw P203:",1,"[A]");
				  break;
			  }
			  case 17:
			  {
				  ustaw_parametr("Ustaw P204:",0,"[obr/m]");
				  break;
			  }
			  case 18:
			  {
				  ustaw_parametr("Ustaw P205:",0,"[ohm]");
				  break;
			  }
			  case 19:
			  {
				  ustaw_parametr("Ustaw P206:",0,"[ohm]");
				  break;
			  }
			  case 20:
			  {
				  ustaw_parametr("Ustaw P207:",0,"[ohm]");
				  break;
			  }
			  case 21:
			  {
				  ustaw_parametr("Ustaw P208:",0,"[ohm]");
				  break;
			  }
			  case 22:
			  {
				  ustaw_parametr("Ustaw P209:",0,"[ohm]");
				  break;
			  }
			  case 23:
			  {
				  ustaw_parametr("Ustaw P210:",0,"");
				  break;
			  }
			  case 24:
			  {
				  ustaw_parametr("Ustaw P211:",0,"[Hz]");
				  break;
			  }
			  case 25:
			  {
				  ustaw_parametr("Ustaw P212:",0,"[Hz]");
				  break;
			  }
			  case 26:
			  {
				  ustaw_parametr("Ustaw P213:",0,"[Hz]");
				  break;
			  }
			  case 27:
			  {
				  ustaw_parametr("Ustaw P214:",2,"");
				  break;
			  }

		  }
	  }


}
//3 KOLEJNE FUNKCJE ODPOWIADAJA ZA KONWERSJE FLOAT NA STRING!!

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{

	int powerr = 1;
	int k = 0;
	while(k < afterpoint)
	{
	powerr = powerr * 10;
	k++;
	}
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);
	if(n < 1)
	res[i] = '0';
    // check for display option after point
    if (afterpoint != 0)
    {

        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007

        fpart = fpart * powerr;

        intToStr((int)fpart, res + i + 1, afterpoint);
    	if(n < 1)
    	{
    	res[i] = '0';
    	res[i+1] = '.';
        intToStr((int)fpart, res + i +2, afterpoint);
    	}
    }
}

// KONIEC FUNKCJI KONWERTUJACYCH FLOAT NA STRING!


void HAL_SYSTICK_Callback()
{
	static int beka = 0, y = 0;
	static uint8_t Data[3];
	float x = 0;

	++beka;

	if(beka >= 80)
	{
		beka = 0;
		if(y == 0)
		{
			Data[0] = 111;
			if(fpom >= 256)
			{
				Data[1] = fpom - 255;
				Data[2] = 255;
			}
			else{
				Data[1] = fpom;
				Data[2] = 0;
			}
			++y;
		}
		else if(y == 1)
		{
			Data[0] = 51;
			Data[1] = 0;
			Data[2] = (int)Temp;
			++y;
		}
		else if(y == 2)
		{
			if((int)DCV >= 256)
			{
				Data[1] = (int)DCV - 255;
				Data[2] = 255;
			}
			else{
				Data[1] = (int)DCV;
				Data[2] = 0;
			}
			++y;
		}
		else if(y == 3)
		{
			Data[0] = 53;
			if((int)DCI >= 256)
			{
				Data[1] = (int)DCI - 255;
				Data[2] = 255;
			}
			else{
				Data[1] = (int)DCI;
				Data[2] = 0;
			}
			y = 0;
		}

		deltaf = param[11];
		x = 10 * param[10];
		fzad = (int)x;
		kierunekzad = (int)param[12];
		boost = param[6] / 100;
		ma = param[9] / 400.0;
		TIMEtoRAMPup = param[0];
		TIMEtoRAMPdown = param[1];
		ksztalt = param[7];

		if(ma != ma_old)
		{
			ZmianaMA(ksztalt, &param[10], ma, boost);
			ma_old = ma;
		}

		HAL_UART_Transmit_DMA(&huart6, Data, sizeof(Data));

	}

////////////////////////////////////////////////////////////////////////////////
	//drganie stykow
	y2++;
	if(y2 >= 60)
	{
	key.byte=(key.byte<<4);// przesuwaj¹c w lew¹ stronê bity z do³u uzupe³niane s¹ zerami
	//ustawienie jedynek na pozycjach od 0 do 3 w przypadku odczytania zer z odpowiednich wejœæ KEYx
		key.byte|=((HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin)==GPIO_PIN_SET)|((HAL_GPIO_ReadPin(ENC_Button_GPIO_Port, ENC_Button_Pin)==GPIO_PIN_SET)<<1)|((HAL_GPIO_ReadPin(Button_1_GPIO_Port, Button_1_Pin)==GPIO_PIN_SET)<<2)|((HAL_GPIO_ReadPin(Button_2_GPIO_Port, Button_2_Pin)==GPIO_PIN_SET)<<3));
		falling_slope.word=(falling_slope.word<<4);

	if(key.flags.ENC_Button)
	{  //czy wciœniêty jest przycisk Button?
		if(!key.flags.ENC_Button_PREV)
			falling_slope.flags.ENC_Button=1; //Czy wykryto zbocze opadaj¹ce
		if(falling_slope.flags.ENC_Button_PREV6)
		{
				if(keylock == 0)
					keylock = 1;
				czysc_ekran = 1;
				if(menu[0] == 0 && menu[1] == 0)
					menu[0] = 1;
				else if(menu[0] == 1)
					menu[0] = 2;
				else if(menu[0] == 2)
					zapisano = 1;

				if(menu[0] == 0 && menu[1] == 1)
					menu[0] = 4;

				if(menu[0] == 3){
					param[10] = f_zad;
				}
		}
	}

	if(key.flags.Button){  //czy wciœniêty jest przycisk Button?
		if(!key.flags.Button_PREV)
			falling_slope.flags.Button = 1; //Czy wykryto zbocze opadaj¹ce
		if(falling_slope.flags.Button_PREV6)
		{
			if(keylock == 0)
			{
				keylock = 1;
				czysc_ekran = 1;
				if(menu[0] == 1)
					menu[2] = 1;
				if(menu[0] < 3)
					menu[0] --;
				if(menu[0] < 0)
					menu[0] = 0;
				if(menu[0] == 3)
					menu[0] = 0;
				if(menu[0] == 4)
					menu[0] = 0;
			}
		}
	}
		if(key.flags.Button_1)
		{  //czy wciœniêty jest przycisk Button?
			if(!key.flags.Button_1_PREV)
				falling_slope.flags.Button_1=1; //Czy wykryto zbocze opadaj¹ce
			if(falling_slope.flags.Button_1_PREV6)
			{
				if(menu[0] == 0)
				{
					status = 1;
					czysc_ekran = 1;
					menu[0] = 3;
				}
			}
		}
			if(key.flags.Button_2)
			{  //czy wciœniêty jest przycisk Button?
				if(!key.flags.Button_2_PREV)
					falling_slope.flags.Button_2 = 1; //Czy wykryto zbocze opadaj¹ce
				if(falling_slope.flags.Button_2_PREV6)
				{

						czysc_ekran = 1;
						status = 0;
						menu[0] = 0;
						param[10] = 0;
				}
			}
		y2 = 0;
	}

	//////////////////////////////////////////////////////////////////

	++ZP2;


	if(start == 1)
	{
		if(fzad!=fzad_old)
		{
			ZP2 = 0;
			CzasRampy(fzad, &fzad_old, TIMEtoRAMPup, TIMEtoRAMPdown, &ZP3, deltaf);
		}

		if(ZP2 % ZP3 == 0)
		{
			RampaLiniowaF(fzad, fpom, &f, deltaf);
			if(boost != boost_old)
			{
				RampaGladkaMA(boost, RampG);
				RampaLiniowaMA(boost, RampL);
				boost_old=boost;
			}
			SterowanieUdoF(ksztalt, f, Usin, UsinRAMPA, 120, boost, fpom, RampL, RampG);
		}
	}
	Tworzenie_Rampy_U_do_F(Rampa, Rampa_U_do_F, param[6], &boost_old, 120);
	f_old = Rampa_Gladka(param[7], param[6], f, f_old, Usin, UsinRAMPA, Rampa, Rampa_U_do_F, 120);

	Temp = Pomiary_TEMP_DCV(Pomiary, &DCV, &y125);
	DCI = Pomiar_DCI(TABDCI, Pomiary);

	f = RampaGladkaF(&n, &m, param[2], param[3], f, param[10], &fzad_old, param[0], param[1], Rampa, 120);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	float fchwilowe = 0;

	if(htim->Instance == TIM10)
	{
		++ZP1;

		fchwilowe = 10 * f;
		fpom = (int)fchwilowe;

		ZmianaKierunku(&kierunek, kierunekzad, &param[10], f);

		if((ZP1 >= Timer[fpom]) && (f != 0))
		{
			PWM(UsinRAMPA, &n1, 120, kierunek);
			ZP1 = 0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {


	if(Received[0]=='1'){
		if(Received[1]=='0'){
			if(Received[2]=='1')	ZP4=0;			//Czas rampy rosn¹cej
			else if(Received[2]=='2')	ZP4=1;		//Czas rampy opadaj¹cej
			else if(Received[2]=='3')	ZP4=2;		//Kszta³t rampy rosn¹cej
			else if(Received[2]=='4')	ZP4=3;		//Kszta³t rampy opadaj¹cej
			else if(Received[2]=='5')	ZP4=4;		//Tryb zatrzymania
			else if(Received[2]=='6')	ZP4=5;		//Ograniczenie pr¹du
			else if(Received[2]=='7')	ZP4=6;		//Boost
			else if(Received[2]=='8')	ZP4=7;		//Kszta³t U/f
			else if(Received[2]=='9')	ZP4=8;		//Prêdkoœæ obrotowa
			}
			else if(Received[1]=='1'){
				if(Received[2]=='0')	ZP4=9;			//Napiêcie
				else if(Received[2]=='1')	ZP4=10;		//Fzad
				else if(Received[2]=='2')	ZP4=11;		//deltaf
				else if(Received[2]=='3')	ZP4=12;		//kierunek obrotu
			}
	}
	else if(Received[0]=='2'){
		if(Received[1]=='0'){
			if(Received[2]=='1')	ZP4=13;			//Moc znamionowa
			else if(Received[2]=='2')	ZP4=14;		//Napiêcie znamionowe przewodowe
			else if(Received[2]=='3')	ZP4=15;		//Pr¹d znamionowy
			else if(Received[2]=='4')	ZP4=16;		//Znamionowa prêdkoœæ
			else if(Received[2]=='5')	ZP4=17;		//Rs rezystancja stojana
			else if(Received[2]=='6')	ZP4=18;		//Rr rezystancja wirnika
			else if(Received[2]=='7')	ZP4=19;		//Ls indukcyjnosc rozproszenia stojana
			else if(Received[2]=='8')	ZP4=20;		//Lr indukcyjnosc rozproszenia wirnika
			else if(Received[2]=='9')	ZP4=21;		//Lm indukcyjnoœæ magnesowania
		}
		else if(Received[1]=='1'){
			if(Received[2]=='0')	ZP4=22;			//p liczba par biegunów
			else if(Received[2]=='1')	ZP4=23;		//czestotliwosc znamionowa
			else if(Received[2]=='2')	ZP4=24;		//czestotliwosc min
			else if(Received[2]=='3')	ZP4=25;		//czestotliwosc max
			else if(Received[2]=='4')	ZP4=26;		//cos_fi
		}
	}

	if(ZP4==0||ZP4==1||ZP4==11||ZP4==13||ZP4==15||ZP4==17||ZP4==18||ZP4==19||ZP4==20||ZP4==21||ZP4==26||ZP4==10){
		param[ZP4]=1000*(Received[3]-48)+100*(Received[4]-48)+10*(Received[5]-48)+(Received[6]-48)+0.1*(Received[7]-48);
	}
	else if(ZP4==2||ZP4==3||ZP4==4||ZP4==5||ZP4==6||ZP4==7||ZP4==8||ZP4==9||ZP4==12||ZP4==14||ZP4==16||ZP4==22||ZP4==23||ZP4==24||ZP4==25){
		param[ZP4]=10000*(Received[3]-48)+1000*(Received[4]-48)+100*(Received[5]-48)+10*(Received[6]-48)+(Received[7]-48);
	}

	HAL_UART_Receive_DMA(&huart6, Received, 8);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
