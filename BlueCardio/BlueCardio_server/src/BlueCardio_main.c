
/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name          : BLE_Throughput_main.c
* Author             : RF Application Team
* Version            : 1.1.0
* Date               : 09-February-2016
* Description        : Code demostrating the BLE Throughput applications
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file BLE_Throughput_main.c
 * @brief This is a Throughput demo that shows how to implement a simple throughput test  between two BlueNRG-1,2 devices.
 * Two throughput test options are available: 
 * unidirectional (server notification to client side); 
 * bidirectional (server notification to client side and write without response from client to server side).
 * 
**/
    
/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-1 throughput demo \see BLE_Throughput_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "gp_timer.h"
#include "sleep.h"
#include "app_state.h"
#include "throughput.h"
#include "SDK_EVAL_Config.h"
#include "Throughput_config.h"
#include "adc.h"
#include "gpio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_THROUGHPUT_VERSION_STRING "1.0.0" 


/* Private macro -------------------------------------------------------------*/
extern uint16_t connection_handle;
/* Private variables ---------------------------------------------------------*/


BOOL send_flag;
volatile _Bool on_sleep = FALSE;

typedef union {
	float update_buff_f[CONVERSION_NUM];
	uint32_t update_buff_u32[CONVERSION_NUM+1];
	uint8_t update_buff_u8[CONVERSION_NUM*4+sizeof(int)];
}update_value;


uint32_t Timer_Elapsed(struct timer *t)
{
	uint32_t ct = Clock_Time() ;
	if ( ct > t->start )
		return Clock_Time() - t->start;
	else 
		return 0xFFFFFFFF - t->start + ct;
}

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void) 
{
	uint8_t ret;
	update_value uv;
	uint32_t msg_counter	=	0;
	int elapsed_time_conv = 0;
	int elapsed_time_send = 0;
	
	uint8_t wakeup_source = 0;
	uint8_t wakeup_level = 0;
	
	int buf_c = 0;
	_Bool processed	= FALSE;
	
	struct timer t_elapsed_conv; // bad name
	struct timer t_elapsed_send; 
	
	struct timer t_converter;
	
	Timer_Restart(&t_elapsed_conv);
	Timer_Restart(&t_elapsed_send);
  /* System Init */
  SystemInit();
  
  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();
  
  /* Init Clock */
  Clock_Init();

  /* Init the UART peripheral */
  SdkEvalComUartInit(UART_BAUDRATE); 
	USR_Initialize();

  /* BlueNRG-1 stack init */
  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    while(1);
  }
  
  printf("BlueNRG-1 BLE Throughput Server Application (version: %s, %d,%d,%d)\r\n", BLE_THROUGHPUT_VERSION_STRING,MAX_ATT_MTU,PREPARE_WRITE_LIST_SIZE,MBLOCKS_COUNT);

  /* Init Throughput test */
  ret = THROUGHPUT_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("THROUGHPUT_DeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }
  
  printf("BLE Stack Initialized \n");
	
	ADC_Initialize();
	
	printf("ADC Initialized \n");
	
	Timer_Set(&t_elapsed_conv, 0);
	Timer_Set(&t_elapsed_send, 0);
	Timer_Set(&t_converter, CLOCK_SECOND/250);
	
  while(1) {
    /* BlueNRG-1 stack tick */
    BTLE_StackTick();
		
		/* Application tick */
    APP_Tick(NULL);
		
		if(Timer_Expired(&t_converter))
		{
			ADC_Start();
			processed = FALSE;
			Timer_Restart(&t_converter);
		}
		
		if (ADC_Ready() && processed == FALSE)
			{
				elapsed_time_conv = Timer_Elapsed(&t_elapsed_conv);
				Timer_Restart(&t_elapsed_conv);
				
				ADC_GetData(&uv.update_buff_f[buf_c++], 1);
				processed = TRUE;
				
				if(buf_c >= 4)
				{
					buf_c = 0;	
					uv.update_buff_u32[CONVERSION_NUM] = msg_counter;
					msg_counter+=1;
//					Commented to check real ADC rate without sending noise
					if ( APP_UpdateTX(uv.update_buff_u8, CONVERSION_NUM*4+sizeof(msg_counter)) )
					{
//					измер€ем период исполнени€ команды изменени€ атрибута
//					elapsed_time_send = Timer_Elapsed(&t_elapsed_send);
//					Timer_Restart(&t_elapsed_send);
					
					}
//				printf("%d :: ", conv_counter);
//				for(int i = 0; i < CONVERSION_NUM; i++)
//						printf("%f ", uv.update_buffer_f[i]);
//				printf(":: ETc: %d(ms) ", elapsed_time_conv);	//врем€ с последнего измерени€ ј÷ѕ
//				printf("ETs: %d(ms) ", elapsed_time_send);	//врем€ с последней команды изменени€ атрибута
//				printf("\r\n");
				}
			}
//			if(on_sleep)
//			{
//				wakeup_source = WAKEUP_IO11;
//				wakeup_level = (WAKEUP_IOx_HIGH << WAKEUP_IO11_SHIFT_MASK);
//				BlueNRG_Sleep(SLEEPMODE_NOTIMER, wakeup_source, wakeup_level);
//				if (ret != BLE_STATUS_SUCCESS) {
//					printf("BlueNRG_Sleep() error 0x%02x\r\n", ret);
//					while(1);
//				}
//				APP_FLAG_SET(SET_CONNECTABLE);
//				on_sleep = FALSE;
//			}
  }
  
} /* end main() */

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
