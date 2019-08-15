#include "BlueNRG_x_device.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include "adc.h"
#include <stdio.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC_OUT_ADDRESS         (ADC_BASE + 0x16)
#define ADC_DMA_CH0             (DMA_CH0)


/* Private variables ---------------------------------------------------------*/ 
ADC_InitType xADC_InitType;
uint16_t buffer_adc[ADC_DMA_BUFFER_LEN];

/**
* @brief  ADC_Configuration.
* @param  None
* @retval None
*/
void ADC_Configuration(void)
{ 
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, ENABLE);
  
  /* Configure ADC */
  /* ADC_Input_AdcPin1 == ADC1 */
  /* ADC_Input_AdcPin2 == ADC2 */
  /* ADC_Input_AdcPin12 == ADC1 - ADC2 */
  xADC_InitType.ADC_OSR = ADC_OSR_200;
    //ADC_Input_BattSensor; //ADC_Input_TempSensor;// ADC_Input_AdcPin1 // ADC_Input_AdcPin12 // ADC_Input_AdcPin2
  xADC_InitType.ADC_Input = ADC_Input_AdcPin1; //ADC_Input_AdcPin12;
  xADC_InitType.ADC_ConversionMode = ADC_ConversionMode_Continuous;
  xADC_InitType.ADC_ReferenceVoltage = ADC_ReferenceVoltage_0V6; //ADC_ReferenceVoltage_0V6;
  xADC_InitType.ADC_Attenuation = ADC_Attenuation_0dB;
    
  ADC_Init(&xADC_InitType);
  
  /* Enable auto offset correction */
  ADC_AutoOffsetUpdate(ENABLE);
  ADC_Calibration(ENABLE);
}

/**
* @brief  DMA_Configuration.
* @param  None
* @retval None
*/
void DMA_Configuration(void)
{ 
  DMA_InitType DMA_InitStructure;
  
  SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_DMA, ENABLE);
  
  /* DMA_CH_UART_TX Initialization */
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC_OUT_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buffer_adc;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = (uint32_t)ADC_DMA_BUFFER_LEN;  
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
  DMA_Init(ADC_DMA_CH0, &DMA_InitStructure);
  
  /* Enable DMA ADC CHANNEL 0 Transfer Complete interrupt */
  DMA_FlagConfig(ADC_DMA_CH0, DMA_FLAG_TC, ENABLE);
  
  /* Select DMA ADC CHANNEL 0 */
  DMA_SelectAdcChannel(DMA_ADC_CHANNEL0, ENABLE);
    
  /* Enable DMA ADC CHANNEL 0 */
  DMA_Cmd(ADC_DMA_CH0, ENABLE);
}


void ADC_Initialize(void){
	
	/* DMA Initialization */
  DMA_Configuration();
  
  /* ADC Initialization */
  ADC_Configuration();
  
  /* ADC Initialization */
  ADC_DmaCmd(ENABLE);
  
  /* Start new conversion */
  ADC_Cmd(ENABLE);
	
}

int ADC_Ready(void){
	return DMA_GetFlagStatus(DMA_FLAG_TC0);
}

void ADC_Start(void){
  /* Reload DMA channel 0 */
  ADC_DMA_CH0->CMAR = (uint32_t)buffer_adc;        
  ADC_DMA_CH0->CNDTR = ADC_DMA_BUFFER_LEN;
      
  /* Enable DMA channel 0 */
  DMA_Cmd(ADC_DMA_CH0, ENABLE);
}

/* Return size of data pushed to buf */
int	ADC_GetData(float *buf, int max_size){
	uint16_t i=0;
	if(DMA_GetFlagStatus(DMA_FLAG_TC0)) {     
      DMA_ClearFlag(DMA_FLAG_TC0);
      
      /* ADC_DMA_CH0 disable */
      DMA_Cmd(ADC_DMA_CH0, DISABLE);
      
      
      for(i=0; i< ADC_DMA_BUFFER_LEN && i<max_size; i++) {
				buf[i] = (ADC_ConvertSingleEndedVoltage(buffer_adc[i], ADC_Input_AdcPin1, xADC_InitType.ADC_ReferenceVoltage, xADC_InitType.ADC_Attenuation))*1000;
      }
		 return i;
    } 
		return 0;
}

