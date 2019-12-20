/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "arm_math.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#include "sw_timers.h"
#include "fir_filter_taps.h"
#include "slld.h"
#include "fvt_detector.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Predefined packet, detection and transmission settings
#define CONVERSION_NUM 4
#define DATA_AMOUNT 256
#define BLOCK_SIZE 16		//FIR filter block size
#define FREQ 250.0f
#define OVERLAP 32			//window overlap in detection algorithm

#define MODE_INTERNAL 1
#define MODE_EXTERNAL 0

#define USE_MEMORY 1
#if USE_MEMORY
const short* ECG_SAMPLES = (const short*)0x08019000;
#else
extern const short ECG_SAMPLES[];
#endif

const int PRESET_LENGTH = 172800;
extern const int RpeakSamples[];
extern const int RPEAK_LENGTH;
//Predefined settings of Threshold algorithm
//#define THRESHOLD 786000
//#define THRESHOLD 566000

//Predefined markers
#define NO_MARKER 0
#define NO_MARKER_CHAR 'N'
#define	R_PEAK		'R'
#define	WINDOW_MARK	'W'
#define	SR_PEAK		'S'
#define	A_MARK		'A'
#define	B_MARK		'B'
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
typedef union {
	float update_buff_f[CONVERSION_NUM];
	uint32_t update_buff_u32[CONVERSION_NUM+1];
	uint8_t update_buff_u8[CONVERSION_NUM*4+sizeof(int)];
}update_value;
//
update_value uv; //simple conversion variable

struct fvt_detector fvt;
struct timer t_converter;
struct timer t_sender;
volatile int msg_counter	=	0;
volatile int buffer_counter = 0;
volatile int preset_buffer_counter = 0;
	
volatile int adjusted_data = 0;
volatile int preset_counter = 0;

volatile int data_from_mode = MODE_INTERNAL;
volatile bool OnDataReady = true;
volatile bool OnHaltWork = false;
volatile bool Transmission = false;
volatile int sample_index = 0;


int data_insert[DATA_AMOUNT] = {0};		//array to contain data from ADC
int data_onsend[DATA_AMOUNT] = {0};		//array to forward data on transmitter
int marker_onsend[DATA_AMOUNT] = {0};

int data_markers[DATA_AMOUNT] = {0};	//array to contain markers data
int heartbeat = 0;
int window[DATA_AMOUNT+OVERLAP] = {0};
int window_markers[DATA_AMOUNT+OVERLAP] = {0};
int data_filter[DATA_AMOUNT+OVERLAP];		//array to contain data from filter
static int firStateF32[DATA_AMOUNT + OVERLAP + NUM_TAPS - 1];
static int NUM_BLOCKS = (DATA_AMOUNT+OVERLAP)/BLOCK_SIZE;


//Transmission variables
const char* packet_begin = "ECG";
const char* packet_end = "END\r\n";
const char* packet_delimiter = "::";

char send_str[DATA_AMOUNT];
int markers[CONVERSION_NUM] = {0};
char parsed_markers[CONVERSION_NUM][CONVERSION_NUM];

uint32_t conversion_raw;
HAL_StatusTypeDef ret;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
void Thresholding(int* data, int* peak_holder, int threshold, int size);
void AppendMarker(int* dest, int marker);
void ParseMarkers(int marker_signs, char* markers,	int size);
void ClearBuffers(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		t_OnDigitCompleteInterrupt();
	}
}
//

/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion, and 
  *         you can add your own implementation.    
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	__nop();
}
//

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	__nop();
}
//

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		if(!OnHaltWork)
		{
			data_from_mode = (data_from_mode+1)%2;
			buffer_counter = 0;
			preset_buffer_counter = 0;
			preset_counter = 0;
			sample_index = 0;
			ClearBuffers();
		}
		OnHaltWork ? (OnHaltWork = false) : (OnHaltWork = true);
	}
}
//

void ClearBuffers(void)
{
	memset(window_markers, 0, sizeof(window_markers[0])*(DATA_AMOUNT+OVERLAP));
	memset(window, 0, sizeof(window[0])*(DATA_AMOUNT+OVERLAP));
	memset(data_filter, 0, sizeof(data_filter[0])*(DATA_AMOUNT+OVERLAP));
	memset(data_markers, 0, sizeof(data_markers[0])*(DATA_AMOUNT));
	memset(data_insert, 0, sizeof(data_insert[0])*(DATA_AMOUNT));
}
//

volatile int error_shift = 0;
volatile int preset_tmp = 0;
void t_Converter_callback(void)
{
	if(!OnHaltWork)
	switch(data_from_mode)
	{
		case MODE_EXTERNAL:
		{
			if((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_READY) != 0 && buffer_counter < DATA_AMOUNT)
			{
				ret = HAL_ADC_Start_DMA(&hadc1,	&conversion_raw, 1);
				if(ret != HAL_OK)
					__nop();
				//uv.update_buff_f[buffer_counter++] = (float)(conversion_raw&0x0FFF)*1.25f/4096.0f/80.53f;
				//adjusted_data = (float)(conversion_raw&0x0FFF)*1.25f/4096.0f/80.53f;
				adjusted_data = (int)(conversion_raw&0x0FFF)*1.25f/4096.0f*1000000;
				data_insert[buffer_counter++] = adjusted_data;
			}
			else if(buffer_counter >= DATA_AMOUNT)
			{
					memcpy(window, window+DATA_AMOUNT, sizeof(window[0])*OVERLAP);
					memcpy(window+OVERLAP, data_insert, sizeof(data_insert[0])*DATA_AMOUNT);
					
					memcpy(window_markers, window_markers+DATA_AMOUNT, sizeof(window_markers[0])*OVERLAP);
					//memset(window_markers+OVERLAP, 0, sizeof(data_markers[0])*(DATA_AMOUNT));
					memcpy(window_markers+OVERLAP, data_markers, sizeof(data_markers[0])*(DATA_AMOUNT));
					preset_buffer_counter = 0;
					OnDataReady = false;
			}
			break;
		}
		case MODE_INTERNAL:
		{
			if(preset_buffer_counter < DATA_AMOUNT)
			{
//				ret = HAL_ADC_Start_DMA(&hadc1,	&conversion_raw, 1);
//				if(ret != HAL_OK)
//					__nop();
				if(preset_counter >= PRESET_LENGTH)
				{
					preset_counter = 0;
					preset_buffer_counter = 0;
					sample_index = 0;
					ClearBuffers();
				}
				adjusted_data = ECG_SAMPLES[preset_counter];
				data_insert[preset_buffer_counter] = adjusted_data;
				
				// SSB 
				// Вот здесь и надо заполнять data_markers !
				while(RpeakSamples[sample_index] < preset_counter && sample_index < RPEAK_LENGTH){ 
					sample_index++;
				}

					//AppendMarker(&data_markers[preset_buffer_counter], SR_PEAK);
					if(RpeakSamples[sample_index] == preset_counter)
					{
						data_markers[preset_buffer_counter] = SR_PEAK;
					}
					else
						data_markers[preset_buffer_counter] = 0;
					
				preset_buffer_counter++;
				preset_counter++;
			}
			else if(preset_buffer_counter >= DATA_AMOUNT){
					memcpy(window, window+DATA_AMOUNT, sizeof(window[0])*OVERLAP);
					memcpy(window+OVERLAP, data_insert, sizeof(data_insert[0])*DATA_AMOUNT);
					
					memcpy(window_markers, window_markers+DATA_AMOUNT, sizeof(window_markers[0])*OVERLAP);
					//memset(window_markers+OVERLAP, 0, sizeof(data_markers[0])*(DATA_AMOUNT));
					memcpy(window_markers+OVERLAP, data_markers, sizeof(data_markers[0])*(DATA_AMOUNT));
					preset_buffer_counter = 0;
					OnDataReady = false;
			}
			
			break;
		}
		default: break;
	}
}
//

volatile int onsend_counter = 0;
void t_Sender_callback(void)
{
	if((!OnDataReady || OnHaltWork))
		return;
	
	if(onsend_counter < DATA_AMOUNT)
		{
			memcpy(uv.update_buff_u32, data_onsend+onsend_counter, CONVERSION_NUM*sizeof(data_onsend[0]));
			memcpy(markers, marker_onsend+onsend_counter, CONVERSION_NUM*sizeof(marker_onsend[0]));
			
			for(int i = 0; i < CONVERSION_NUM; i++)
				ParseMarkers(markers[i], parsed_markers[i], CONVERSION_NUM);
			sprintf(send_str, "%s %d%s%d%s %d%s %d%s %d%s %s%s", 
				packet_begin,	msg_counter,	packet_delimiter, 
											uv.update_buff_u32[0], parsed_markers[0],
											uv.update_buff_u32[1], parsed_markers[1],
											uv.update_buff_u32[2], parsed_markers[2],
											uv.update_buff_u32[3], parsed_markers[3],
																		packet_delimiter, packet_end);
											
			onsend_counter+=CONVERSION_NUM;
			msg_counter++;
			ret = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)send_str, strlen(send_str));
			if(ret == HAL_OK)
			{
				HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
			}
		}
}
//

void Thresholding(int* data, int* peak_holder, int threshold, int size)
{
	int x_m1 = 0;
	int x_p1 = 0;
	int x = 0;
	int x_m2 = 0;
	int x_p2 = 0;
	
	for(int i = OVERLAP/2; i < size+OVERLAP/2; i++)
	{
			x_m2 = data[i-2];
			x_m1 = data[i-1];
			x = data[i];
			x_p1 = data[i+1];
			x_p2 = data[i+2];
			
			if(x >= threshold)
			{
				if((x - x_m1 >= 0 && x - x_m2 > 0) && 
					(x - x_p1 >= 0 && x - x_p2 > 0))
				{
					AppendMarker(&peak_holder[i-OVERLAP/2], R_PEAK);
					heartbeat++;
				}
			}
	}
}
//

volatile int threshold = 4000;
const float alpha = 1.0;
const float gamma = 0.2;

void AdaptiveThresholding(int* data, int* peak_holder, int size)
{
	int x_m1 = 0;
	int x_m2 = 0;
	int x = 0;
	int x_p1 = 0;
	int x_p2 = 0;
	
	for(int i = OVERLAP/2; i < size+OVERLAP/2; i++)
	{
			x_m2 = data[i-2];
			x_m1 = data[i-1];
			x = data[i];
			x_p1 = data[i+1];
			x_p2 = data[i+2];
			
			if(x >= threshold)
			{
				if((x - x_m1 > 0 && x - x_m2 > 0) && 
					(x - x_p1 > 0 && x - x_p2 > 0))
				{
					AppendMarker(&peak_holder[i-OVERLAP/2], R_PEAK);
					heartbeat++;
					threshold = (int)(alpha*gamma*((float)x) - (1 - alpha)*(float)threshold);
				}
				else if((x - x_m1 > 0 && x - x_m2 > 0) && (x - x_p1 == 0 && x - x_p2 > 0))
				{
					AppendMarker(&peak_holder[i-OVERLAP/2], R_PEAK);
					heartbeat++;
					threshold = (int)(alpha*gamma*((float)x) - (1 - alpha)*(float)threshold);
				}
			}
	}
}
//


//void AppendPresetMarkers(void)
//{
	//Версия Сергея
////	while( //&&  
////				RpeakSamples[sample_index] <= preset_counter + DATA_AMOUNT && 
////							sample_index < RPEAK_LENGTH)
////	{
////			if (RpeakSamples[sample_index] >= preset_counter) { 
////					data_markers[RpeakSamples[sample_index] - preset_counter] = SR_PEAK;
////			
////				}
////    	sample_index = (sample_index+1)%RPEAK_LENGTH;
////   }
// Моя версия
//	if(preset_counter == RpeakSamples[sample_index])
//		{
//			AppendMarker(&data_markers[(RpeakSamples[sample_index]+OVERLAP)%DATA_AMOUNT], SR_PEAK);
//			sample_index = (sample_index+1)%RPEAK_LENGTH;
//		}
//}
//

void AppendMarker(int* dest, int marker)
{
	int free_pos = 0;
	uint32_t mask = 0x00FF;
	while(((*dest)&mask) != 0)
	{
		free_pos++;
		mask = mask << 8;
	}
	(*dest) = (*dest)|(marker<<(free_pos*8));
}
//

void ParseMarkers(int marker_signs, char* markers,	int size)
{
	int tmp = 0;
	int n = 0;
	memset(markers, 0, sizeof(markers[0])*size);
	markers[0] = NO_MARKER_CHAR;
	tmp = marker_signs;
	for(int i = 0; i < size; i++)
	{
		n = 0;
		while(tmp)
		{
			markers[i+n] = (char)(tmp&0x00FF);
			tmp = tmp >> 8;
			n++;
		}
	}
}
//
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	Timer_set(&t_converter, FREQ, &t_Converter_callback, true, true);
	Timer_set(&t_sender, FREQ/4.0f, &t_Sender_callback, true, true);
	fvt_InitDetector(&fvt, FREQ, 0.5f, 10, true);
	
	arm_fir_instance_q31 S;
	arm_fir_init_q31(&S, NUM_TAPS, &firCoeffs32[0], &firStateF32[0], BLOCK_SIZE);
	
	HAL_GPIO_WritePin(Enable_Indicator_GPIO_Port, Enable_Indicator_Pin, GPIO_PIN_SET);
	
	BYTE id[7];
	//slld_ReadCmd(0xAB, id, 4);
	slld_RDIDCmd(id, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int counter = 0;
	volatile int out = 0;
	int rest = 0;
  while (1)
  {
		t_OnDigitCompleteContinuous();
		
		//if(!OnDataReady && buffer_counter >= DATA_AMOUNT)
		if(!OnDataReady && onsend_counter >= DATA_AMOUNT)
		{			
			if (buffer_counter >= DATA_AMOUNT && data_from_mode == MODE_EXTERNAL)
				buffer_counter = 0;		//позволяем записывать новый массив
			if (preset_buffer_counter >= DATA_AMOUNT && data_from_mode == MODE_INTERNAL)
				preset_buffer_counter = 0;
			
				if(HAL_GPIO_ReadPin(Filter_Switch_GPIO_Port, Filter_Switch_Pin) == GPIO_PIN_RESET)
				{
					//Thresholding(window, window_markers+OVERLAP/2, 4000, DATA_AMOUNT);
					AdaptiveThresholding(window, window_markers+OVERLAP/2, DATA_AMOUNT);
					memcpy(data_onsend, window+OVERLAP/2, sizeof(data_onsend[0])*DATA_AMOUNT);
					memcpy(marker_onsend, window_markers+OVERLAP/2, sizeof(marker_onsend[0])*DATA_AMOUNT);
					fvt_FindLast(&fvt, marker_onsend, DATA_AMOUNT);
					if(fvt.window_fill == fvt.fvt_window_size)
						fvt_CheckWindow(&fvt);
				}
				else
				{
					for(counter = 0; counter < NUM_BLOCKS; counter++)
					{
						arm_fir_q31(&S, window + (counter * BLOCK_SIZE), data_filter + (counter * BLOCK_SIZE), BLOCK_SIZE);
					}
					//Thresholding(data_filter, window_markers+OVERLAP/2, 4000, DATA_AMOUNT);
					AdaptiveThresholding(data_filter, window_markers+OVERLAP/2, DATA_AMOUNT);
					memcpy(data_onsend, data_filter+OVERLAP/2, sizeof(data_onsend[0])*DATA_AMOUNT);
					memcpy(marker_onsend, window_markers+OVERLAP/2, sizeof(marker_onsend[0])*DATA_AMOUNT);
				}
				AppendMarker(&marker_onsend[0], WINDOW_MARK);
				onsend_counter = 0;
				OnDataReady = true;
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 10;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED) !=  HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 80;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_EN_Pin|LD4_Pin|Enable_Indicator_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_EN_Pin LD4_Pin Enable_Indicator_Pin */
  GPIO_InitStruct.Pin = SPI_EN_Pin|LD4_Pin|Enable_Indicator_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Filter_Switch_Pin */
  GPIO_InitStruct.Pin = Filter_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Filter_Switch_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
