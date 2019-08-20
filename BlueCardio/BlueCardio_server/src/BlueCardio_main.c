
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

* \subsection Project_configurations Project configurations
- \c Client_bidirectional - Client role configuration for bidirectional throughput test
- \c Client_unidirectional - Client role configuration for unidirectional throughput test
- \c Server_bidirectional - Server role configuration for bidirectional throughput test
- \c Server_unidirectional - Server role configuration for unidirectional throughput test


* \section Board_supported Boards supported
- \c STEVAL-IDB007V1
- \c STEVAL-IDB007V2
- \c STEVAL-IDB008V1
- \c STEVAL-IDB008V2
- \c STEVAL-IDB009V1


* \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB00XV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name |            |  Description                                                                |
| JP1         |   JP2      |                                                                             |
----------------------------------------------------------------------------------------------------------
| ON 1-2      | ON 2-3     | USB supply power                                                            |
| ON 2-3      | ON 1-2     | The supply voltage must be provided through battery pins.                   |
| ON 1-2      |            | USB supply power to STM32L1, JP2 pin 2 external power to BlueNRG1           |


@endtable 

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB00XV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
| JP1         | 1-2: to provide power from USB (JP2:2-3). 2-3: to provide power from battery holder (JP2:1-2)                                                                          |          
| JP2         | 1-2: to provide power from battery holder (JP1:2-3). 2-3: to provide power from USB (JP1:1-2). Pin2 to VDD  to provide external power supply to BlueNRG-1 (JP1: 1-2)   |
| JP3         | pin 1 and 2 UART RX and TX of MCU. pin 3 GND.                                                                                                                          |          
| JP4         | Fitted: to provide VBLUE to BlueNRG1. It can be used also for current measurement.                                                                                     |
| JP5         | Fitted : TEST pin to VBLUE. Not fitted:  TEST pin to GND                                                                                                               |


@endtable
                        
* \section Pin_settings Pin settings
@table
|            |                                                    Client_bidirectional                                                     |||||                                                         Server_unidirectional                                                         |||||                                                    Server_bidirectional                                                     |||||                                                         Client_unidirectional                                                         |||||
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  PIN name  |     STEVAL-IDB007V1    |     STEVAL-IDB007V2    |     STEVAL-IDB008V1    |     STEVAL-IDB008V2    |     STEVAL-IDB009V1    |      STEVAL-IDB007V1     |      STEVAL-IDB007V2     |      STEVAL-IDB008V1     |      STEVAL-IDB008V2     |      STEVAL-IDB009V1     |     STEVAL-IDB007V1    |     STEVAL-IDB007V2    |     STEVAL-IDB008V1    |     STEVAL-IDB008V2    |     STEVAL-IDB009V1    |      STEVAL-IDB007V1     |      STEVAL-IDB007V2     |      STEVAL-IDB008V1     |      STEVAL-IDB008V2     |      STEVAL-IDB009V1     |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|    ADC1    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    ADC2    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO0    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO1    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO11    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO12    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO13    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO14    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO15    |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO16    |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO17    |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO18    |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO19    |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|     IO2    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    IO20    |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO21    |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO22    |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO23    |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO24    |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|    IO25    |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |          N.A.          |          N.A.          |          N.A.          |          N.A.          |        Not Used        |           N.A.           |           N.A.           |           N.A.           |           N.A.           |         Not Used         |
|     IO3    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO4    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO5    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO6    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO7    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     IO8    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|    TEST1   |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |

@endtable 

* \section Serial_IO Serial I/O
 
@table
| Parameter name  | Value               | Unit      |
------------------------------------------------------
| Baudrate        | 921600              |  bit/sec  |
| Data bits       | 8                   | bit       |
| Parity          | None                | bit       |
| Stop bits       | 1                   | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|            |                                                    Client_bidirectional                                                     |||||                                                         Server_unidirectional                                                         |||||                                                    Server_bidirectional                                                     |||||                                                         Client_unidirectional                                                         |||||
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |     STEVAL-IDB007V1    |     STEVAL-IDB007V2    |     STEVAL-IDB008V1    |     STEVAL-IDB008V2    |     STEVAL-IDB009V1    |      STEVAL-IDB007V1     |      STEVAL-IDB007V2     |      STEVAL-IDB008V1     |      STEVAL-IDB008V2     |      STEVAL-IDB009V1     |     STEVAL-IDB007V1    |     STEVAL-IDB007V2    |     STEVAL-IDB008V1    |     STEVAL-IDB008V2    |     STEVAL-IDB009V1    |      STEVAL-IDB007V1     |      STEVAL-IDB007V2     |      STEVAL-IDB008V1     |      STEVAL-IDB008V2     |      STEVAL-IDB009V1     |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     DL2    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     DL3    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|     DL4    |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |

@endtable


* \section Buttons_description Buttons description
@table
|                |                                                    Client_bidirectional                                                     |||||                                                         Server_unidirectional                                                         |||||                                                    Server_bidirectional                                                     |||||                                                         Client_unidirectional                                                         |||||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |     STEVAL-IDB007V1    |     STEVAL-IDB007V2    |     STEVAL-IDB008V1    |     STEVAL-IDB008V2    |     STEVAL-IDB009V1    |      STEVAL-IDB007V1     |      STEVAL-IDB007V2     |      STEVAL-IDB008V1     |      STEVAL-IDB008V2     |      STEVAL-IDB009V1     |     STEVAL-IDB007V1    |     STEVAL-IDB007V2    |     STEVAL-IDB008V1    |     STEVAL-IDB008V2    |     STEVAL-IDB009V1    |      STEVAL-IDB007V1     |      STEVAL-IDB007V2     |      STEVAL-IDB008V1     |      STEVAL-IDB008V2     |      STEVAL-IDB009V1     |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|      PUSH2     |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |        Not Used        |        Not Used        |        Not Used        |        Not Used        |        Not Used        |         Not Used         |         Not Used         |         Not Used         |         Not Used         |         Not Used         |
|      RESET     |     Reset BlueNRG1     |     Reset BlueNRG1     |     Reset BlueNRG2     |     Reset BlueNRG2     |     Reset BlueNRG2     |      Reset BlueNRG1      |      Reset BlueNRG1      |      Reset BlueNRG2      |      Reset BlueNRG2      |      Reset BlueNRG2      |     Reset BlueNRG1     |     Reset BlueNRG1     |     Reset BlueNRG2     |     Reset BlueNRG2     |     Reset BlueNRG2     |      Reset BlueNRG1      |      Reset BlueNRG1      |      Reset BlueNRG2      |      Reset BlueNRG2      |      Reset BlueNRG2      |

@endtable

* \section Usage Usage

This Throughput demo has 2 roles:
 - The server that expose the Throughput service. It is the slave.
 - The client that uses the Throughput service. It is the master.

The Throughput Service contains 2 Characteristics:
 -# The TX Characteristic: the client can enable notifications on this characteristic. When the server has data to be sent, it sends notifications which contains the value of the TX Characteristic
 -# The RX Characteristic: it is a writable caracteristic. When the client has data to be sent to the server, it writes a value into this characteristic.

The maximum length of the characteristic value is 20 bytes. 

NOTES:
 - The <b>Client_unidirectional</b> and <b>Server_unidirectional </b> workspaces allow to target a unidirectional throughput test: server device sends characteristic notifications to the client device(THROUGHPUT_TEST_SERVER define enabled on on both workspaces, on the preprocessor options). The required serial port baudrate is 921600.
 - Program the client side on one BlueNRG-1 platform and reset it. The platform is seen on the PC as a virtual COM port. Open the port in a serial terminal emulator. 
 - Program the server side on a second BlueNRG-1 platform and reset it. The two platforms try to establish a connection. As soon as they get connected, the slave continuously 
   sends notification of TX characteristic (20 bytes) to the client. 
 - After every 500 packets, the measured application unidirectional throughput is displayed.
 
 - The <b>Client_bidirectional</b> and <b>Server_bidirectional</b> workspaces allow to target a bidirectional throughput test: server device sends characteristic notifications to the client device  and client device performs sends characteristic write without responses to the server device (THROUGHPUT_TEST_SERVER and THROUGHPUT_TEST_CLIENT define enabled on both workspaces, on the preprocessor options). The required serial port baudrate is 921600.
 - Program the client side on one BlueNRG-1 platform and reset it. The platform is seen on the PC as a virtual COM port. Open the port in a serial terminal emulator. 
 - Program the server side on a second BlueNRG-1 platform and reset it. The two platforms try to establish a connection. As soon as they get connected, the slave continuously 
   sends notifications of TX characteristic (20 bytes) to the client and the client continuously sends  write without response of RX characteristic (20 bytes) to the server.
 - After every 500 packets, the measured application bidirectional throughput is displayed.

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
#include "app_state.h"
#include "throughput.h"
#include "SDK_EVAL_Config.h"
#include "Throughput_config.h"
#include "adc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_THROUGHPUT_VERSION_STRING "1.0.0" 


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


BOOL send_flag;

typedef union {
	float update_buffer_f[CONVERSION_NUM];
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
	//uint8_t send_buffer[32+2];
	uint32_t conv_counter=0;
	int elapsed_time_conv = 0;
	int elapsed_time_send = 0;
	struct timer t_elapsed_time; // bad name
	Timer_Restart(&t_elapsed_time);
	struct timer t_elapsed_send; 
	Timer_Restart(&t_elapsed_send);
	

  /* System Init */
  SystemInit();
  
  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();
  
  /* Init Clock */
  Clock_Init();

  /* Init the UART peripheral */
  SdkEvalComUartInit(UART_BAUDRATE); 

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
	
	Timer_Set(&t_elapsed_time, 0);
	Timer_Set(&t_elapsed_send, 0);
  
  while(1) {
    /* BlueNRG-1 stack tick */
    BTLE_StackTick();
		
		/* Application tick */
    APP_Tick(NULL);
		
		if (ADC_Ready())
			{
				conv_counter+=1;	
				ADC_GetData(uv.update_buffer_f, CONVERSION_NUM);
				ADC_Start();
				//memcpy(send_buffer, uv.update_buff_u8, CONVERSION_NUM*4);
				//memcpy(send_buffer+CONVERSION_NUM*4, (void*)&conv_counter, 2);		
				uv.update_buff_u32[CONVERSION_NUM]=conv_counter;
				
	
				//измер€ем период конверсии
				elapsed_time_conv = Timer_Elapsed(&t_elapsed_time);
				Timer_Restart(&t_elapsed_time);
				
				
				/*	Commented for check real ADC rate without sending noise
				
				if ( APP_UpdateTX(uv.update_buff_u8, CONVERSION_NUM*4+sizeof(conv_counter)) )
				{
					//измер€ем период исполнени€ команды изменени€ атрибута
					elapsed_time_send = Timer_Elapsed(&t_elapsed_send);
					Timer_Restart(&t_elapsed_send);
					
				}
				*/
				
				printf("%d :: ", conv_counter);
				for(int i = 0; i < CONVERSION_NUM; i++)
						printf("%f ", uv.update_buffer_f[i]);
				printf(":: ETc: %d(ms) ", elapsed_time_conv);	//врем€ с последнего измерени€ ј÷ѕ
				printf("ETs: %d(ms) ", elapsed_time_send);	//врем€ с последней команды изменени€ атрибута
				printf("\r\n");
			}
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
