#include <stdio.h>
#include <string.h>
#include "gp_timer.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "app_state.h"
#include "osal.h"
#include "gatt_db.h"
#include "template.h"
#include "SDK_EVAL_Config.h"
#include "OTA_btl.h"

extern int time;
extern uint32_t updateFreq;
extern BOOL update_freq;

#define CMD_BUFF_SIZE 512

/* Private variables ---------------------------------------------------------*/

uint8_t connInfo[20];
volatile int app_flags = SET_CONNECTABLE;
volatile uint16_t connection_handle = 0;
uint8_t send_time[4] = {0};


/* UUIDs */
UUID_t UUID_Tx;
UUID_t UUID_Rx;

static char cmd[CMD_BUFF_SIZE];
static uint16_t cmd_buff_end = 0, cmd_buff_start = 0;




/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Template_DeviceInit.
* Description    : Init the Template device.
* Input          : none.
* Return         : Status.
*******************************************************************************/
uint8_t Template_DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t name[] = {'C', 'a', 'r', 'd', 'i', 'o', '_', 'A', 'D', 'C'};
	int name_size =  sizeof(name)/sizeof(name[0]);
  uint8_t role = GAP_PERIPHERAL_ROLE;
  uint8_t bdaddr[] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};

  
  /* Configure Public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != BLE_STATUS_SUCCESS){
    printf("Setting BD_ADDR failed: 0x%02x\r\n", ret);
    return ret;
  }

  /* Set the TX power to 4 dBm */
  aci_hal_set_tx_power_level(1, 6);

  /* GATT Init */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS) {
    printf ("Error in aci_gatt_init(): 0x%02x\r\n", ret);
    return ret;
  } else {
    printf ("aci_gatt_init() --> SUCCESS\r\n");
  }
  
  /* GAP Init */
  ret = aci_gap_init(role, 
										 0, // privacy disabled
										 name_size, // length of device name characteristic
										 &service_handle,&dev_name_char_handle, &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS) {
    printf ("Error in aci_gap_init() 0x%02x\r\n", ret);
    return ret;
  } else {
    printf ("aci_gap_init() --> SUCCESS\r\n");
  }

  /* Set the device name */
  ret = aci_gatt_update_char_value_ext(
		0, //Conn_Handle_To_Notify
		service_handle, 
		dev_name_char_handle,
		0, // No notification or indication (local characteristic value update) 
		name_size, // Total length of the characteristic value
		0,  // The offset from which the attribute value has to be updated
		name_size, // Value_Length Length of the Value parameter in octets
	  name);
	
  if (ret != BLE_STATUS_SUCCESS) {
    printf ("Error in Gatt Update characteristic value 0x%02x\r\n", ret);
    return ret;
  } else {
    printf ("aci_gatt_update_char_value_ext() --> SUCCESS\r\n");
  }

  ret = Add_Template_Service();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in Add_Template_Service 0x%02x\r\n", ret);
    return ret;
  } else {
    printf("Add_Chat_Service() --> SUCCESS\r\n");
  }
  return BLE_STATUS_SUCCESS;
}


/*******************************************************************************
* Function Name  : Process_InputData.
* Description    : Process a command. It should be called when data are received.
* Input          : data_buffer: data address.
*	           Nb_bytes: number of received bytes.
* Return         : none.
*******************************************************************************/
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
  uint8_t i;
  
  for (i = 0; i < Nb_bytes; i++)
  {
    if(cmd_buff_end >= CMD_BUFF_SIZE-1){
      cmd_buff_end = 0;
    }
    
    cmd[cmd_buff_end] = data_buffer[i];
    SdkEvalComIOSendData(data_buffer[i]);
    cmd_buff_end++;
    
    if(cmd[cmd_buff_end-1] == '\n'){
      if(cmd_buff_end != 1){
        
        cmd[cmd_buff_end] = '\0'; // Only a termination character. Not strictly needed.
        
        // Set flag to send data. Disable UART IRQ to avoid overwriting buffer with new incoming data
        APP_FLAG_SET(SEND_DATA);
        NVIC_DisableIRQ(UART_IRQn);
        
        cmd_buff_start = 0;        
        
      }
      else {
        cmd_buff_end = 0; // Discard
      }
    }
  }
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
   
  uint8_t local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'C','a','r','d','i','o','_','A','D','C'};
	
  hci_le_set_scan_response_data(0,NULL);
  
  ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
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
void APP_Tick(void)
{
  tBleStatus ret;
  
  if(APP_FLAG(SET_CONNECTABLE))
  {
    Make_Connection();
    APP_FLAG_CLEAR(SET_CONNECTABLE);
  }

	  if(APP_FLAG(TX_BUFFER_FULL))
			return;
		
		send_time[0] = (uint8_t)(0x000000FF&time);
		send_time[1] = (uint8_t)((0x0000FF00&time)>>8);
		send_time[2] = (uint8_t)((0x00FF0000&time)>>16);
		send_time[3] = (uint8_t)((0xFF000000&time)>>24);
		
		if(APP_FLAG(CONNECTED))
		{
			ret = aci_gatt_update_char_value_ext(connection_handle, templateServHandle, TXCharHandle, 1, 4, 0, 4, send_time);
			if(ret ==	BLE_STATUS_INSUFFICIENT_RESOURCES)
			{
				APP_FLAG_SET(TX_BUFFER_FULL);
				return;
			}
			if(ret != BLE_STATUS_SUCCESS)
			{
				printf("Updating characteristic value failed! 0x%02x\r\n", ret);
			}
			
			if(update_freq)
			{
				uint8_t send_freq[4];
				send_freq[0] = (uint8_t)(0x000000FF&updateFreq);
				send_freq[1] = (uint8_t)((0x0000FF00&updateFreq)>>8);
				send_freq[2] = (uint8_t)((0x00FF0000&updateFreq)>>16);
				send_freq[3] = (uint8_t)((0xFF000000&updateFreq)>>24);
				update_freq = FALSE;
				ret = aci_gatt_update_char_value_ext(connection_handle, templateServHandle, timeCharHandle, 1, 4, 0, 4, send_freq);
				if(ret ==	BLE_STATUS_INSUFFICIENT_RESOURCES)
				{
					APP_FLAG_SET(TX_BUFFER_FULL);
					return;
				}
				if(ret != BLE_STATUS_SUCCESS)
				{
					printf("Updating characteristic value failed! 0x%02x\r\n", ret);
				}
			}
		}


#if REQUEST_CONN_PARAM_UPDATE    
  if(APP_FLAG(CONNECTED) && !APP_FLAG(L2CAP_PARAM_UPD_SENT) && Timer_Expired(&l2cap_req_timer))
  {
    aci_l2cap_connection_parameter_update_req(connection_handle, 8, 16, 0, 600);
    APP_FLAG_SET(L2CAP_PARAM_UPD_SENT);
  }
#endif
  

  
}/* end APP_Tick() */


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

/*******************************************************************************
 * Function Name  : aci_gatt_read_permit_req_event.
 * Description    : This event is given when a read request is received
 *                  by the server from the client.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
}

void aci_hal_end_of_radio_activity_event(uint8_t Last_State,
                                         uint8_t Next_State,
                                         uint32_t Next_State_SysTime)
{
}
