/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
* File Name          : throughput.c
* Author             : AMS - VMA RF  Application team
* Version            : V1.0.0
* Date               : 08-February-2016
* Description        : This file handles bytes received from USB and the init
*                      function. 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "gp_timer.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "app_state.h"
#include "throughput.h"
#include "osal.h"
#include "gatt_db.h"
#include "SDK_EVAL_Config.h"
#include "gpio.h"

/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

#define ADV_INTERVAL_MIN        200     // ms
#define ADV_INTERVAL_MAX        300     // ms

#define SERVER_ADDRESS 0xBB, 0x00, 0x03, 0xE1, 0x80, 0x92

#define CMD_BUFF_SIZE 512

/* Private macros ------------------------------------------------------------*/ 

/* Private variables ---------------------------------------------------------*/

uint8_t connInfo[20];
volatile int app_flags = SET_CONNECTABLE;
volatile uint16_t connection_handle = 0;

/* UUIDs */
UUID_t UUID_Tx;
UUID_t UUID_Rx;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : THROUGHPUT_DeviceInit.
* Description    : Init the throughput test.
* Input          : none.
* Return         : Status.
*******************************************************************************/
uint8_t THROUGHPUT_DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t name[] = {'B', 'l', 'u', 'e', 'N', 'R', 'G', '1'};
  
  uint8_t role = GAP_PERIPHERAL_ROLE;
  uint8_t bdaddr[] = {SERVER_ADDRESS};
  
  /* Configure Public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != BLE_STATUS_SUCCESS){
    printf("Setting BD_ADDR failed: 0x%02x\r\n", ret);
    return ret;
  }

  /* Set the TX power to +8 dBm */
  aci_hal_set_tx_power_level(1, 7);

  /* GATT Init */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS) {
    printf ("Error in aci_gatt_init(): 0x%02x\r\n", ret);
    return ret;
  } else {
    printf ("aci_gatt_init() --> SUCCESS\r\n");
  }
  
  /* GAP Init */
  ret = aci_gap_init(role, 0x00, 0x08, &service_handle, 
                     &dev_name_char_handle, &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS) {
    printf ("Error in aci_gap_init() 0x%02x\r\n", ret);
    return ret;
  } else {
    printf ("aci_gap_init() --> SUCCESS\r\n");
  }

  /* Set the device name */
  ret = aci_gatt_update_char_value_ext(0, service_handle, dev_name_char_handle, 0,sizeof(name), 0, sizeof(name), name);
  if (ret != BLE_STATUS_SUCCESS) {
    printf ("Error in Gatt Update characteristic value 0x%02x\r\n", ret);
    return ret;
  } else {
    printf ("aci_gatt_update_char_value_ext() --> SUCCESS\r\n");
  }

  ret = Add_Throughput_Service();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in Add_Throughput_Service 0x%02x\r\n", ret);
    return ret;
  } else {
    printf("Add_Throughput_Service() --> SUCCESS\r\n");
  }  
  return BLE_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name  : Make_Connection.
* Description    : If the device is a Client create the connection. Otherwise puts
*                  the device in discoverable mode.
* Input          : none.
* Return         : none.
*******************************************************************************/
void Make_Connection(void)
{  
  tBleStatus ret;
  
  uint8_t local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','C','a','r','d','i','o'};
  
  /* disable scan response */
  hci_le_set_scan_response_data(0,NULL);
  
  ret = aci_gap_set_discoverable(ADV_IND, ADV_INTERVAL_MIN, ADV_INTERVAL_MAX, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 0, 0);
  if (ret != BLE_STATUS_SUCCESS)
    printf ("Error in aci_gap_set_discoverable(): 0x%02x\r\n", ret);
  else
    printf ("aci_gap_set_discoverable() --> SUCCESS\r\n");
}

/*******************************************************************************
* Function Name  : APP_Tick.
* Description    : Tick to run the application state machine.
* Input          : none.
* Return         : none.
*******************************************************************************/
void APP_Tick( void (*fptr_while_connected)(void))
{
  //tBleStatus ret;
  
  if(APP_FLAG(SET_CONNECTABLE))
  {
    Make_Connection();
		USRB_EXTI_Set();
    APP_FLAG_CLEAR(SET_CONNECTABLE);
  }
	if(APP_FLAG(TX_BUFFER_FULL)){
		printf("tx buffer is full!\n");
		return;
	}
			
	if(APP_FLAG(CONNECTED))
	{
			if (fptr_while_connected)
				fptr_while_connected();
	
	}
		
#if REQUEST_CONN_PARAM_UPDATE    
  if(APP_FLAG(CONNECTED) && !APP_FLAG(L2CAP_PARAM_UPD_SENT) && Timer_Expired(&l2cap_req_timer))
  {
    aci_l2cap_connection_parameter_update_req(connection_handle, 8, 16, 0, 600);
    APP_FLAG_SET(L2CAP_PARAM_UPD_SENT);
  }
#endif
  
}/* end APP_Tick() */



_Bool APP_UpdateTX(uint8_t *sendbuf, uint8_t size)
{
	if(APP_FLAG(CONNECTED))	{
		tBleStatus ret = aci_gatt_update_char_value_ext( 
													connection_handle, 
													ServHandle, 
													TXCharHandle, 
													1,		// 0x01: Notification
													size, // Char_Length Total length
													0,		//Value_Offset
													size, //length 
													sendbuf);
	
		if(ret != BLE_STATUS_SUCCESS){
			printf("Updating characteristic value failed! 0x%02x\r\n", ret);
			return FALSE;
		}
		return TRUE;
	}
	return FALSE; // Yes, If we have not sent data - we says that false! it's logical
}







/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
 * Function Name  : hci_le_connection_complete_event.
 * Description    : This event indicates that a new connection has been created.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)

{ 
  connection_handle = Connection_Handle;
  
  APP_FLAG_SET(CONNECTED);
  
#if REQUEST_CONN_PARAM_UPDATE
  APP_FLAG_CLEAR(L2CAP_PARAM_UPD_SENT);
  Timer_Set(&l2cap_req_timer, CLOCK_SECOND*2);
#endif
}/* end hci_le_connection_complete_event() */

/*******************************************************************************
 * Function Name  : hci_disconnection_complete_event.
 * Description    : This event occurs when a connection is terminated.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
   APP_FLAG_CLEAR(CONNECTED);
  /* Make the device connectable again. */
  APP_FLAG_SET(SET_CONNECTABLE);
  APP_FLAG_CLEAR(NOTIFICATIONS_ENABLED);
  APP_FLAG_CLEAR(TX_BUFFER_FULL);

  APP_FLAG_CLEAR(START_READ_TX_CHAR_HANDLE);
  APP_FLAG_CLEAR(END_READ_TX_CHAR_HANDLE);
  APP_FLAG_CLEAR(START_READ_RX_CHAR_HANDLE); 
  APP_FLAG_CLEAR(END_READ_RX_CHAR_HANDLE);

}/* end hci_disconnection_complete_event() */


/*******************************************************************************
 * Function Name  : aci_gatt_attribute_modified_event.
 * Description    : This event occurs when an attribute is modified.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  Attribute_Modified_CB(Attr_Handle, Attr_Data_Length, Attr_Data);  
}

/*******************************************************************************
 * Function Name  : aci_gatt_tx_pool_available_event.
 * Description    : This event occurs when a TX pool available is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{       
  /* It allows to notify when at least 2 GATT TX buffers are available */
  APP_FLAG_CLEAR(TX_BUFFER_FULL);
} 
