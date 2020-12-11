/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "fonts.h"
#include "tft.h"
#include "user_setting.h"
#include "functions.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG  0
#define TEST_LENGTH_SAMPLES 2048
#define LCD_W  320
#define LCD_H  240
#define TIME_MAX 5

/* FFT settings */
#define SAMPLES	512 			/* 256 real party and 256 imaginary parts */
#define FFT_SIZE SAMPLES / 2		/* FFT size is always the same size as we have samples, so 256 in our case */

//Defines para referenciar os estados da FSM
#define DC_GND		0
#define FFT			1
#define T_DIV		2
#define HOLD_DC		3
#define HOLD_FFT	4
#define HOLD_TDIV	5

#define DC_GND_SCREEN	1
#define FFT_SCREEN		2
#define FREQ_SCREEN		3
#define HOLD_SCREEN		4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* ------------------------------------------------------------------
 *	Imagens BMP convertidas para RGB565
 * -----------------------------------------------------------------*/
extern const unsigned short osciloscopio_320x240[76800];
extern const unsigned short telaosc_320x240[76800];

/*
 * Global Variables
 */
int32_t len;
int32_t size;
char sBuffer[30]; //CONVERSÃO...
uint8_t iADCValues[320] = {0};	//buffer para aguarda as amostras do sinal analogico
uint16_t iRefCH1 = 120;			//Posicao da referencia "0" no LCD
uint16_t mapTime[5][2]={{500,100},{250,50},{100,20},{50,10},{10,0}}; //1ºvalor = tempo em us
																	 //2ºvalor = tempo para delay de amostras

_Bool bModeDC = 1;		//Variavel para indicar se o modo DC esta ativo
uint8_t iTimeRef = 4;
uint8_t iModeFFT = 0;
uint8_t iVolts = 2;

/* ------------------------------------------------------------------
* Input and Output buffer Declarations for FFT
* ------------------------------------------------------------------- */
float32_t input_f32_10khz[TEST_LENGTH_SAMPLES];
static float32_t output_buffer[TEST_LENGTH_SAMPLES/2];

uint8_t fim_amostragem =0;

/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
float32_t Input[SAMPLES];
float32_t Output[FFT_SIZE];


//Maquina de estado
_Bool bHold = 0;
_Bool bAvanca = 0;
_Bool bRecua = 0;
_Bool bSelect = 0;

uint8_t iMenuSelection = 1;

struct State {
	uint16_t Screen;		//Saída para o estado
	uint16_t wait;		//Tempo de espera do estado
	uint8_t next[8];	//Vetor de proximos estados
};
typedef const struct State tipoS;

//Estrutura de dados que corresponde ao diagrama de transição de estado
tipoS Fsm[6] = {
		[DC_GND] = {DC_GND_SCREEN,0,{DC_GND,HOLD_DC,FFT,HOLD_DC,T_DIV,HOLD_DC,DC_GND,HOLD_DC}},
		[FFT] = {FFT_SCREEN,0,{FFT,HOLD_FFT,T_DIV,HOLD_FFT,DC_GND,HOLD_FFT,FFT,HOLD_FFT}},
		[T_DIV] = {FREQ_SCREEN,0,{T_DIV,HOLD_TDIV,DC_GND,HOLD_TDIV,FFT,HOLD_TDIV,T_DIV,HOLD_TDIV}},
		[HOLD_DC] = {HOLD_SCREEN,0,{DC_GND,HOLD_DC,DC_GND,HOLD_DC,DC_GND,HOLD_DC,DC_GND,HOLD_DC}},
		[HOLD_FFT] = {HOLD_SCREEN,0,{FFT,HOLD_FFT,FFT,HOLD_FFT,FFT,HOLD_FFT,FFT,HOLD_FFT}},
		[HOLD_TDIV] = {HOLD_SCREEN,0,{T_DIV,HOLD_TDIV,T_DIV,HOLD_TDIV,T_DIV,HOLD_TDIV,T_DIV,HOLD_TDIV}},
};

uint8_t cState = DC_GND ;	//Estado atual

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void USER_TFT_Init(void);
//void ModoAC(uint8_t *);
void ModoGND();
void ModoDC();
void ModoFFT();
void TelaInicial(uint8_t *);
void PlotaGrade(uint16_t, uint16_t, uint16_t,uint16_t,uint16_t,uint16_t);
long map(long, long, long, long, long);
void delayUs (uint16_t);
void USER_DrawBackground(void);
void updateMenu(uint8_t iMenuSelect);
void clickEventTreatment(uint8_t iMenuSelect);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

int x = 0;
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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  USER_TFT_Init();

  //Inicia o PWM no pino PB1
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  htim2.Instance->CCR4 = 500; //Duty cycle = 50% -> 50

  //Initialize ISR TIM6
  HAL_TIM_Base_Start_IT(&htim6);

  //Desenha o background do LCD
  USER_DrawBackground();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Verifica se o botao de click foi pressionado
	  if (bSelect == true) clickEventTreatment(iMenuSelection);

	  //Verifica se o botao hold foi pressionado
	  if (bHold == false)
	  {
		  switch(Fsm[cState].Screen)
		  {
			  case DC_GND_SCREEN:	iMenuSelection = DC_GND;
				  	  	  	  	  	updateMenu(iMenuSelection);
									if(bModeDC) ModoDC();
										else ModoGND();
									break;

			  case FFT_SCREEN:	  iMenuSelection = FFT;
				  	  	  	  	  updateMenu(iMenuSelection);
				  	  	  	  	  ModoFFT();
								  break;

			  case FREQ_SCREEN:   iMenuSelection = T_DIV;
				  	  	  	  	  updateMenu(iMenuSelection);
				  	  	  	  	  if(bModeDC) ModoDC();
				  	  	  			else ModoGND();
								  break;

			  case HOLD_SCREEN:	  updateMenu(iMenuSelection);
			  	  	  	  	  	  break;
			  default: break;
		  }
	  }
	  else
	  {
		  updateMenu(iMenuSelection);
	  }
	  //Leitura do ADC2 para verificar a escala V/DIV
	  //USER_ReadVDiv();

	 //Faz a leitura dos botao Avanca e Recua para calcular o proximo estado
	 bAvanca = HAL_GPIO_ReadPin(GPIOC, B_AVANCA_Pin);
	 bRecua = HAL_GPIO_ReadPin(GPIOC, B_RECUA_Pin);
	 cState = Fsm[cState].next[bHold | (bAvanca<<1) | (bRecua<<2)];

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
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
  sConfig.Channel = ADC_CHANNEL_9;
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 168-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim6.Init.Prescaler = 168-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 100-1;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B_HOLD_Pin */
  GPIO_InitStruct.Pin = B_HOLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B_HOLD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_AVANCA_Pin B_RECUA_Pin */
  GPIO_InitStruct.Pin = B_AVANCA_Pin|B_RECUA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B_SELECT_Pin */
  GPIO_InitStruct.Pin = B_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B_SELECT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART2 and Loop until the end of transmission */
 HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
 return ch;
}


void USER_TFT_Init(void)
{
	int16_t ID;

    //Sequência de inicialização do LCD
    tft_gpio_init(); 				//Inicializa os GPIOs do LCD (evita uso do CubeMX)
    HAL_TIM_Base_Start(&htim1); 		//Inicializa o Timer1 (base de tempo de us do LCD)
    ID = tft_readID(); 		//Lê o ID do LCD (poderia ser chamada pela inicialização do LCD)
    HAL_Delay(100);
    tft_init (ID); 			//Inicializa o LCD de acordo com seu ID
    setRotation(1); 			//Ajusta a orientação da tela
    fillScreen(BLACK); 		//Preenche a tela em uma só cor
}

void USER_DrawBackground(void)
{
	fillScreen(BLACK);
	PlotaGrade(300, 200, 50, 10, 20, GREY);	// Plota a grade

	//Plota V/DIV
	//printnewtstr (10,18, WHITE, &mono12x7bold, 1, (uint8_t *)"V/DIV");

	//Plota menu DC
	//printnewtstr (15,237, WHITE, &mono12x7bold, 1, (uint8_t *)"DC");
	//printnewtstr_bc(15,237, BLACK,WHITE, &mono12x7bold, 1, (uint8_t *)" DC ");

	//Plota menu FFT
	//printnewtstr (150,237, WHITE, &mono12x7bold, 1, (uint8_t *)"FFT");

	//Plota menu t/DIV
	//printnewtstr (260,237, WHITE, &mono12x7bold, 1, (uint8_t *)"/DIV");

}

//FUNÇÃO DO MODO DC DO OSCILOSCÓPIO, QUE ATRAVÉS DO MENU INICIAL IREMOS ACESSAR ELA//
void ModoDC()
{
    uint8_t retornar = 0;
    uint16_t iSamplePos, iADC1;

	iSamplePos = 0;
	// AQUI VAI O CODIGO PARA FUNCIONAMENTO DO MODO AC DO OSCILOSCÓPIO//
	//Inicia a leitura  do conversor AD
	for(iSamplePos = 0; iSamplePos <= LCD_W; iSamplePos++)
	{
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
		{
			//Guarda o valor do conversor AD no Array
			iADC1 = HAL_ADC_GetValue(&hadc1);
			iADCValues[iSamplePos] = map(iADC1, 0, 4095, 0, 100);
			if(DEBUG)
			{
				size = sprintf(sBuffer,"ADC1 = %d\r\n",iADC1);
				HAL_UART_Transmit(&huart2, sBuffer, size, 10);
				size = sprintf(sBuffer,"iADCValues = %d\r\n",iADCValues[iSamplePos]);
				HAL_UART_Transmit(&huart2, sBuffer, size, 10);
			}

		}
		delayUs(mapTime[iTimeRef][1]);
	}

//	//Aqui é a condição de retorno/saída da tela.
//	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))
//		retornar = 1;

	//Verifica se ja foi captada as 320 amostras
	if (iSamplePos >= LCD_W)
	{

		//Plota o sinal captado no LCD
		for(int x = 1; x <= LCD_W; x++)
		{
			drawLine(x-1, 120 - iADCValues[x-1], x, 120 - iADCValues[x], RED);
		}
		//memset(valores,0,320);
		iSamplePos = 0;
		HAL_Delay(500);

		//Limpa o sinal captado no LCD e printa a grade novamente
		for(int x = 1; x <= LCD_W; x++)
		{
			drawLine(x-1, 120 - iADCValues[x-1], x, 120 - iADCValues[x], BLACK);
		}
		PlotaGrade(300, 200, 50, 10, 20, GREY);
	}

}

/**
 * Função para desenhar a referencia GND na tela
 */
void ModoGND()
{
	drawLine ( 0,  iRefCH1,  LCD_W,  iRefCH1,  RED);
	HAL_Delay(500);
}

void ModoFFT(void)
{

	arm_cfft_radix4_instance_f32 S_CFFT;
	arm_rfft_instance_f32 S;
	uint16_t i;
    uint16_t y;
    uint16_t z;

	int32_t amp, escala, yini, yfinal, ycentro;
	char num[30];
	int32_t size;

    uint32_t fftSize = 1024;
    uint32_t ifftFlag = 0;
    uint32_t doBitReverse = 1;

	/* ----------------------------------------------------------------------
	* Max magnitude FFT Bin test
	* ------------------------------------------------------------------- */
	arm_status status;
	float32_t maxValue;

    /* Reference index at which max energy of bin ocuurs */
    uint32_t refIndex = 213;
    uint32_t  maxValueIndex = 0;

	//Verifica a amostragem do sinal terminou
    if(fim_amostragem == 1)
    {
    	fim_amostragem = 0;
    	if(DEBUG)
    	{
    		HAL_UART_Transmit(&huart2, "Fim da Amostragem\r\n", 21, 10);
    	}

		status = ARM_MATH_SUCCESS;

		//------------Teste LCD--------------
		//float32_t espelho_pre_processamento[320];
		//for(int i=0; i<320; i++)
		//  espelho_pre_processamento[i] = input_f32_10khz[i*2]; //Pula os zeros da parte imaginária

		/* Process the data through the CFFT/CIFFT module */
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, input_f32_10khz, ifftFlag, doBitReverse);

		/* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
		arm_cmplx_mag_f32(input_f32_10khz, output_buffer, fftSize);

		/* Calculates maxValue and returns corresponding BIN value */
		arm_max_f32(&output_buffer[1], fftSize-1, &maxValue, &maxValueIndex);

		/** Loop here if the signals fail the PASS check.
		 ** This denotes a test failure
		 ** ------------------------------------------------------------------- */
		if ( status != ARM_MATH_SUCCESS)
		{
			while (1);
		}

		//fs = 10.000, numero de amostras =1024, df = 10.000/1024 = 9,76
		//Detecta a frequencia principal
		maxValueIndex = maxValueIndex * 976 / 100;
		size = sprintf(sBuffer,"Frequencia principal = %d\r\n",maxValueIndex);
		HAL_UART_Transmit(&huart2, sBuffer, size, 10);

		//---------------Agora vai para o LCD----------------
		//Desenha o sinal no dominio da frequencia no LCD
		escala = 100;
		ycentro = 220;
		for(i=0; i<LCD_W; i++)
		{
			amp = (int32_t)(output_buffer[i]*escala/maxValue);
			if(DEBUG)
			{
				size = sprintf(num,"%d\r\n",amp);
				HAL_UART_Transmit(&huart2, num, size, 10);
			}

			if(amp>0) { yini = ycentro-amp; yfinal = ycentro; }
			else { yini = ycentro; yfinal = ycentro-amp; }

			fillRect(i, yini, 1, yfinal-yini+1, RED);
		}
		//Initialize ISR TIM6
		HAL_TIM_Base_Start_IT(&htim6);

		HAL_Delay(1000);

		//Limpa o sinal anterior
		for(i=0; i<LCD_W; i++)
		{
			amp = (int32_t)(output_buffer[i]*escala/maxValue);
			if(DEBUG)
			{
				size = sprintf(num,"%d\r\n",amp);
				HAL_UART_Transmit(&huart2, num, size, 10);
			}

			if(amp>0) { yini = ycentro-amp; yfinal = ycentro; }
			else { yini = ycentro; yfinal = ycentro-amp; }

			fillRect(i, yini, 1, yfinal-yini+1, BLACK);
		}

		//Plota a grade
		PlotaGrade(300, 200, 50, 10, 20, GREY);

    }


		//Geração de Sinal para teste( para utilizar, só descomentar esse e comentar o
		//laço for do  conversor AD acima)//

		/*
		for(i=0; i<512; ++i)
				{
					buffer_input[i] = (float32_t) 30*sin(2*PI*freq*i*dt);

					 ++freq;

						  if(freq > 200)
							freq = 1;
				}

		 */
		/////////////////////////////////////////////////////////////////////////////////
		// Plotagem do Sinal gerado para teste no dominio da frequencia para comparação//
		//(para utilizar, só descomentar , não esquecer de comentar o laço for do conversor AD//
		/*
		 for(i=0; i<255; ++i)
					 {
						drawLine(i + 32, (uint16_t)(buffer_input[i] + 50), i + 33, (uint16_t)(buffer_input[i+1] + 50), BLUE);

					  }
		 HAL_Delay(500);
		 */

}

//Funcao responsavel por atualizar o menu do osciloscopio
void updateMenu(uint8_t iMenuSelect)
{
	//Verifica se é modo DC ou GND
	if(bModeDC != true) sprintf(sBuffer,"GND");
		else sprintf(sBuffer,"DC ");

	if (iMenuSelect == DC_GND) printnewtstr_bc(15,237, BLACK,WHITE, &mono12x7bold, 1, sBuffer);
		else printnewtstr_bc (15,237, WHITE,BLACK, &mono12x7bold, 1, sBuffer);

	//Verifica se o Hold está pressionado
	if(bHold) printnewtstr_bc(260,15, BLACK,RED, &mono12x7bold, 1, (uint8_t *)"HOLD");
		else printnewtstr_bc(260,15, BLACK,BLACK, &mono12x7bold, 1, (uint8_t *)"    ");

	//Verifica se menu FFT está selecionado
	if (iMenuSelect == FFT)	printnewtstr_bc(150,237, BLACK,WHITE, &mono12x7bold, 1, "FFT");
		else printnewtstr_bc (150,237, WHITE,BLACK, &mono12x7bold, 1, "FFT");

	//Verifica se menu TDIV está selecionado
	if(iMenuSelect == T_DIV)
	{
		size = sprintf(sBuffer,"%dus ",mapTime[iTimeRef][0]);
		printnewtstr_bc(250,237, BLACK,WHITE, &mono12x7bold, 1, sBuffer);
	}
	else
	{
		size = sprintf(sBuffer,"%dus ",mapTime[iTimeRef][0]);
		printnewtstr_bc (250,237, WHITE,BLACK, &mono12x7bold, 1, sBuffer);
	}

	//Atualiza Volts/DIV
	size = sprintf(sBuffer,"%dV ",iVolts);
	printnewtstr (10,18, WHITE, &mono12x7bold, 1, sBuffer);

}


//Funcao para fazer o tratamento do click do botao select
void clickEventTreatment(uint8_t iMenuSelect)
{
	if(iMenuSelect == DC_GND)
	{
		//altera entre modo GND e DC
		bModeDC = !bModeDC;
	}
	else if(iMenuSelect == T_DIV)
	{
		//altera tempo de amostragem do sinal
		iTimeRef++;
		if (iTimeRef >= TIME_MAX)
		{
			iTimeRef = 0;
		}

	}
	else
	{
		//Caso existir mais tratamentos de botões
	}

	bSelect = false;
}

//Callback de tratamento de interrupcao EXTI
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Verifica se a o botao hold foi pressionado
	if(GPIO_Pin == B_HOLD_Pin)
	{
		//HAL_Delay(20);
		if((HAL_GPIO_ReadPin(GPIOC, B_HOLD_Pin)== GPIO_PIN_SET))
		{
			bHold = !bHold;
		}
	}

	//Verifica se o botao select foi precionado
	if(GPIO_Pin == B_SELECT_Pin)
	{
		if((HAL_GPIO_ReadPin(GPIOC, B_SELECT_Pin)== GPIO_PIN_SET))
		{
			bSelect = true;
		}
	}
}

//Callback das interrupcoes dos timers (TIM6) para amostragem do sinal de entrada para FFT
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int32_t amostra = 0;

	//Ler um eixo do acelerometro
	if (amostra < TEST_LENGTH_SAMPLES/2)
	{
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
		{
			input_f32_10khz[amostra*2] =  HAL_ADC_GetValue(&hadc1);
			input_f32_10khz[amostra*2 + 1] = 0;
			amostra++;
		}
	}
	else
	{
		fim_amostragem = 1;					//Sinaliza o fim do processo
		amostra = 0;
		HAL_TIM_Base_Stop_IT(&htim6);		//Interrompe a amostragem
	}
}


/**
* Funcao para plotar uma grade na tela LCD
* Por: Luiz Fernando - 22/11/2020
* Informacoes necessarias:
* 	Tamanho da tela Largura e Altura
* 	Tamanho dos quadrados simetricos
* 	Posicao X e Y do inicio da tela
*	Cor da Linha
* Necessario projeto com LCD em funcionamento e bibliotecas adjacentes
* NB.: o tamanho do quadro precisa ser um MDC entre largura e altura
*/
void PlotaGrade(uint16_t iLargura, uint16_t iAltura, uint16_t tmQuadros, uint16_t posX, uint16_t posY, uint16_t cCOLOR)
{
	/** Define quantidade de quadros e tamanho */
	uint16_t qtL = (iAltura/tmQuadros); 	//qtd linhas
	uint16_t qtC = (iLargura/tmQuadros);	//qtd colunas
	uint16_t corT = YELLOW;					//cor dos tracos das subdivisoes
	int16_t posLV = (iLargura/2)+posX;		//posicao eixo central vertical
	int16_t posLH = (iAltura/2)+posY;		//posicao eixo central horizontal
	uint16_t i = 0;
	/** desenha linhas verticais */
	for(i=0;i<=qtC;i++)
	{
		drawFastVLine((i*tmQuadros+posX), posY, iAltura, cCOLOR);
	}
	/** desenha linhas horizontais */
	for(i=0;i<=qtL;i++)
	{
		drawFastHLine(posX, (i*tmQuadros+posY), iLargura, cCOLOR);
	}
	uint16_t fDX = 4;			//fator de divisao e multiplicacao (exibir tracos)
	uint16_t tmT = 6;			//tamanho do traco
	/** desenha tracos verticais */
	for(i=0;i<=(qtC*fDX);i++)
	{
		drawFastVLine((i*(tmQuadros/fDX)+posX), posLH-(tmT/2), tmT, corT);
	}
	/** desenha tracos horizontais */
	for(i=0;i<=(qtL*fDX);i++)
	{
		drawFastHLine(posLV-(tmT/2), (i*(tmQuadros/fDX)+posY), tmT, corT);
	}
}

/**
 * FUNÇÃO MAP, PORTADO DO ARDUINO PARA O STM32
 */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Função para gerar Delay em micro segundos
 */
void delayUs (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
