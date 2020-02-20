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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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
#include "r_detector.h"
#include "markers.h"
#include "queue.h"
#include "packet.h"
#include "app_state.h"
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

#define MODE_INTERNAL 1
#define MODE_EXTERNAL 0

#define OVER_USART (1u)
#define OVER_BLE	(0u)

#define USE_PACKETS 			(1u)
#define USE_STATE_MACHINE	(0u)

#ifndef UPLOAD
const short* ECG_SAMPLES = (const short*)0x08019000;
#else
extern const short ECG_SAMPLES[];
#endif
//

#if SEND_WAY == OVER_USART

#endif
//

#if SEND_WAY == OVER_BLE
volatile uint8_t expected_message_len = MAX_STRING_LENGTH;
volatile bool exchanged = true;
#endif
//

#ifndef SEND_WAY
#error "Way to send data was not defined!"
#endif
//

#if USE_STATE_MACHINE == 1
#define INIT_RECEIVE_LEN 5
#define DEFAULT_CRC_LEN 2
volatile bool on_process = false;
uint8_t initial_receive_buff[INIT_RECEIVE_LEN] = {0};
#endif
//


const int PRESET_LENGTH = 208800;
extern const int RpeakSamples[];
extern const int RPEAK_LENGTH;
//Predefined settings of Threshold algorithm
//#define THRESHOLD 786000
//#define THRESHOLD 566000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef union {
	float update_buff_f[CONVERSION_NUM];
	uint32_t update_buff_u32[CONVERSION_NUM+1];
	uint8_t update_buff_u8[CONVERSION_NUM*4+sizeof(int)];
}update_value;
//
update_value uv; //simple conversion variable
Queue q_from_spirx;
Queue q_transmission;
uint8_t spi_receive_buff[MAX_STRING_LENGTH];
uint8_t spi_transmit_buff[MAX_STRING_LENGTH];

struct timer t_converter;
struct timer t_sender;
t_fvt_result fvt_result;

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

float fvt_container[10] = {0};

int data_insert[DATA_AMOUNT] = {0};		//array to contain data from ADC
int data_onsend[DATA_AMOUNT] = {0};		//array to forward data on transmitter
int marker_onsend[DATA_AMOUNT] = {0};

int data_markers[DATA_AMOUNT] = {0};	//array to contain markers data
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
/* USER CODE BEGIN PFP */
void Thresholding(int* data, int* peak_holder, int threshold, int size);
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

#if SEND_WAY == OVER_USART
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == huart2.Instance)
	{
		#if USE_STATE_MACHINE == 0 && SEND_WAY == OVER_BLE
		queue_push(&q_from_spirx, spi_receive_buff, MAX_STRING_LENGTH);
		switch(Check_AppState())
		{
			case STATE_REC_COMMAND:
				if(CheckIfPacket(initial_receive_buff))
				{
					expected_message_len = initial_receive_buff[4] + DEFAULT_CRC_LEN;
					//HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
					if(initial_receive_buff[5] == TYPE_READ)
						Set_AppState(STATE_READ_DATA);
					else if(initial_receive_buff[5] == TYPE_WRITE)
						Set_AppState(STATE_SEND_DATA);
				}
				break;
			case STATE_READ_DATA:
				queue_push(&q_from_spirx, spi_receive_buff, expected_message_len);
				Set_AppState(STATE_REC_COMMAND);
				break;
		}
		#endif
	}
}
//
#endif
//

#if SEND_WAY == OVER_BLE
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
		exchanged = true;
		queue_push(&q_from_spirx, spi_receive_buff, MAX_STRING_LENGTH);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}
//

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == hspi3.Instance)
	{
		exchanged = true;
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}
}
//

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == hspi3.Instance)
	{
		#if USE_STATE_MACHINE == 0
		queue_push(&q_from_spirx, spi_receive_buff, MAX_STRING_LENGTH);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
		#else
		switch(Check_AppState())
		{
			case STATE_REC_COMMAND:
				if(CheckIfPacket(initial_receive_buff))
				{
					expected_message_len = initial_receive_buff[3] + DEFAULT_CRC_LEN;
					//HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
					if(initial_receive_buff[4] == TYPE_READ)
						Set_AppState(STATE_READ_DATA);
					else if(initial_receive_buff[4] == TYPE_WRITE)
						Set_AppState(STATE_SEND_DATA);
					else
						on_process = true;
				}
				else
				{
					on_process = false;
					HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
				}
				break;
			case STATE_READ_DATA:
				queue_push(&q_from_spirx, spi_receive_buff, expected_message_len);
				HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
				Set_AppState(STATE_REC_COMMAND);
				break;
		}
		#endif
	}
}
//
#endif
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
	
	#if SEND_WAY == OVER_BLE
	if(GPIO_Pin == BLE_IRQ_Pin)
	{
		if(HAL_GPIO_ReadPin(BLE_IRQ_GPIO_Port, BLE_IRQ_Pin) == GPIO_PIN_SET)
		{
			if(queue_isempty(&q_transmission))
			{
				HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
				HAL_SPI_Receive_DMA(&hspi3, spi_receive_buff, MAX_STRING_LENGTH);
			}
			else
			{
				memset(spi_transmit_buff, 0, sizeof(spi_transmit_buff[0])*MAX_STRING_LENGTH);
				queue_get_front(&q_transmission, spi_transmit_buff, 0,  queue_get_frontl(&q_transmission));
				HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
				if(HAL_SPI_TransmitReceive_DMA(&hspi3, spi_transmit_buff, spi_receive_buff, MAX_STRING_LENGTH) != HAL_OK)
					{
						Error_Handler();
					}
				exchanged = false;
				queue_pop(&q_transmission);
			}
		}
	}
	#endif
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
				//adjusted_data = abs(ECG_SAMPLES[preset_counter%PRESET_LENGTH] - 1200);
				adjusted_data = ECG_SAMPLES[preset_counter%PRESET_LENGTH];
				data_insert[preset_buffer_counter] = adjusted_data;
				
				// SSB 
				// ��� ����� � ���� ��������� data_markers !
				while(RpeakSamples[sample_index] < preset_counter && sample_index < RPEAK_LENGTH){ 
					sample_index++;
				}

					//AppendMarker(&data_markers[preset_buffer_counter], MARK_S_PEAK);
					if(RpeakSamples[sample_index] == preset_counter)
					{
						data_markers[preset_buffer_counter] = MARK_S_PEAK;
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
volatile int window_mean = 0;
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
			
			memset(send_str, 0, DATA_AMOUNT);
			uint8_t send_length = 0;
			#if USE_PACKETS
			uint8_t data_wmarks[CONVERSION_NUM*4+CONVERSION_NUM*4] = {0};
			uint8_t p_markers[CONVERSION_NUM] = {0};
			uint8_t packet_len = 0;
			uint8_t m_len = 0;
			
			for(int i = CONVERSION_NUM, j = 0; i < CONVERSION_NUM*5; i+=CONVERSION_NUM, j++)
			{
				uint8_t len = ParseMarker_toInt(markers[j], p_markers, CONVERSION_NUM);
				for(int m = 0; m < CONVERSION_NUM; m++)
				{
					data_wmarks[m_len+i+m-4] = uv.update_buff_u8[i+m-4];
				}
				for(int y = 0; y < len; y++)
				{
					data_wmarks[m_len+i+y] = p_markers[y];
				}
				m_len += len;
			}
			packet_len = CONVERSION_NUM*4+m_len;
			
			send_length = MakePacket(IKM_ECG_TX, (uint8_t*)send_str, data_wmarks, packet_len);
			#else
			sprintf(send_str, "%s %d%s%d%s %d%s %d%s %d%s %s%d%s%s", 
				packet_begin,	msg_counter,	packet_delimiter, 
											uv.update_buff_u32[0], parsed_markers[0],
											uv.update_buff_u32[1], parsed_markers[1],
											uv.update_buff_u32[2], parsed_markers[2],
											uv.update_buff_u32[3], parsed_markers[3],
																		packet_delimiter, window_mean, 
																		packet_delimiter, packet_end);
			send_length = strlen(send_str);
			#endif
											
			onsend_counter+=CONVERSION_NUM;
			msg_counter++;
			#if SEND_WAY == OVER_BLE
			HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
			queue_push(&q_transmission, (uint8_t*)send_str, send_length);
			#elif SEND_WAY == OVER_USART
			ret = HAL_UART_Transmit_DMA(&huart2, (uint8_t*)send_str, send_length);
			if(ret == HAL_OK)
			{
				HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
			}
			#endif
		}
}
//

#if SEND_WAY == OVER_BLE
void HandleQueues(void)
{
	if(exchanged == true)
		{
			if(!queue_isempty(&q_transmission))
				{
					memset(spi_transmit_buff, 0, sizeof(spi_transmit_buff[0])*MAX_STRING_LENGTH);
					queue_get_front(&q_transmission, spi_transmit_buff, 0,  queue_get_frontl(&q_transmission));
					while(HAL_DMA_GetState(hspi3.hdmarx) != HAL_DMA_STATE_READY);
					HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
					exchanged = false;
					if(HAL_SPI_TransmitReceive_DMA(&hspi3, spi_transmit_buff, spi_receive_buff, MAX_STRING_LENGTH) == HAL_OK)
						{
							HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
						}
					queue_pop(&q_transmission);	
				}
		}
		
	if(!queue_isempty(&q_from_spirx))
	{
		queue_pop(&q_from_spirx);
	}
}
//
#endif

//void AppendPresetMarkers(void)
//{
	//������ ������
////	while( //&&  
////				RpeakSamples[sample_index] <= preset_counter + DATA_AMOUNT && 
////							sample_index < RPEAK_LENGTH)
////	{
////			if (RpeakSamples[sample_index] >= preset_counter) { 
////					data_markers[RpeakSamples[sample_index] - preset_counter] = MARK_S_PEAK;
////			
////				}
////    	sample_index = (sample_index+1)%RPEAK_LENGTH;
////   }
// ��� ������
//	if(preset_counter == RpeakSamples[sample_index])
//		{
//			AppendMarker(&data_markers[(RpeakSamples[sample_index]+OVERLAP)%DATA_AMOUNT], MARK_S_PEAK);
//			sample_index = (sample_index+1)%RPEAK_LENGTH;
//		}
//}
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
	Timer_set(&t_converter, FREQ, &t_Converter_callback, true, true, true);
	Timer_set(&t_sender, FREQ/4.0f, &t_Sender_callback, true, true, true);
	fvt_InitDetector(fvt_container, 10, marker_onsend, DATA_AMOUNT, FREQ, 0.9f, 4);
	
	arm_fir_instance_q31 S;
	arm_fir_init_q31(&S, NUM_TAPS, &firCoeffs32[0], &firStateF32[0], BLOCK_SIZE);
	
	HAL_GPIO_WritePin(Enable_Indicator_GPIO_Port, Enable_Indicator_Pin, GPIO_PIN_SET);
	
	queue_init(&q_from_spirx);
	queue_init(&q_transmission);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int counter = 0;
	volatile int out = 0;
	int rest = 0;
  while (1)
  {
		t_OnDigitCompleteContinuous();
		#if SEND_WAY == OVER_BLE
		if(!OnHaltWork)
			HandleQueues();
		#endif
		
		#if USE_STATE_MACHINE
		if(OnHaltWork && on_process == false)
		{
			switch(Check_AppState())
			{
				case STATE_REC_COMMAND:
					HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
					#if SEND_WAY == OVER_BLE
					while(HAL_DMA_GetState(hspi3.hdmarx) == HAL_DMA_STATE_BUSY);
					HAL_SPI_Receive_DMA(&hspi3, initial_receive_buff, INIT_RECEIVE_LEN);
					#elif	SEND_WAY == OVER_USART
					HAL_UART_Receive_DMA(&hspi3, initial_receive_buff, INIT_RECEIVE_LEN);
					#endif
					on_process = true;
					break;
				case STATE_READ_DATA:
					HAL_SPI_Receive_DMA(&hspi3, spi_receive_buff, expected_message_len);
					on_process = true;
					break;
				default:
					break;
			}
		}
		#endif
		//
		 
		if(!OnDataReady && onsend_counter >= DATA_AMOUNT)
		{			
			if (buffer_counter >= DATA_AMOUNT && data_from_mode == MODE_EXTERNAL)
				buffer_counter = 0;		//��������� ���������� ����� ������
			if (preset_buffer_counter >= DATA_AMOUNT && data_from_mode == MODE_INTERNAL)
				preset_buffer_counter = 0;
			
			if(HAL_GPIO_ReadPin(Filter_Switch_GPIO_Port, Filter_Switch_Pin) == GPIO_PIN_RESET)
			{
				//AdaptiveThresholding(window, window_markers+OVERLAP/2, DATA_AMOUNT);
				AdaptiveThresholding_high(window, window_markers+OVERLAP/2, DATA_AMOUNT,
					FREQ, 0.2, 0.3, 0.2);
				memcpy(data_onsend, window+OVERLAP/2, sizeof(data_onsend[0])*DATA_AMOUNT);
				memcpy(marker_onsend, window_markers+OVERLAP/2, sizeof(marker_onsend[0])*DATA_AMOUNT);
				int sum_tmp = 0;
				for(int i = 0; i < DATA_AMOUNT; i++)
				{
					sum_tmp += data_onsend[i];
				}
				window_mean = sum_tmp/DATA_AMOUNT;
				fvt_result = fvt_SeekForFVT();
				if(fvt_result.found == FVT_BEGIN)
					AppendMarker(&marker_onsend[fvt_result.index], MARK_FVT_START);
				else if(fvt_result.found == FVT_FINISH)
					AppendMarker(&marker_onsend[fvt_result.index], MARK_FVT_FINISH);
				else if(fvt_result.found == FVT_ONGOING)
					AppendMarker(&marker_onsend[fvt_result.index], MARK_FVT_ONGOING);
			}
			else
			{
				for(counter = 0; counter < NUM_BLOCKS; counter++)
				{
					arm_fir_q31(&S, window + (counter * BLOCK_SIZE), data_filter + (counter * BLOCK_SIZE), BLOCK_SIZE);
				}
				AdaptiveThresholding(data_filter, window_markers+OVERLAP/2, DATA_AMOUNT);
				memcpy(data_onsend, data_filter+OVERLAP/2, sizeof(data_onsend[0])*DATA_AMOUNT);
				memcpy(marker_onsend, window_markers+OVERLAP/2, sizeof(marker_onsend[0])*DATA_AMOUNT);
			}
			AppendMarker(&marker_onsend[0], MARK_WINDOW);
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
