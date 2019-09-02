#include "BlueNRG_x_device.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"

#define USR_BUTT GPIO_Pin_11

GPIO_InitType GPIO_InitStructure;
NVIC_InitType NVIC_InitStructure;
GPIO_EXTIConfigType GPIO_EXTIStructure;

void USR_Initialize(void)
{
	/* Configure USER_BUTTON as interrupt sources */
  GPIO_InitStructure.GPIO_Pin = USR_BUTT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Input;
  GPIO_InitStructure.GPIO_Pull = DISABLE;
  GPIO_InitStructure.GPIO_HighPwr = DISABLE;
  GPIO_Init(&GPIO_InitStructure);
  
  /* Set the GPIO interrupt priority and enable it */
  NVIC_InitType NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Configures EXTI line */
  GPIO_EXTIStructure.GPIO_Pin = USR_BUTT;
  GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
  GPIO_EXTIStructure.GPIO_Event = IRQ_ON_BOTH_EDGE;
  GPIO_EXTIConfig(&GPIO_EXTIStructure);
	
  GPIO_ClearITPendingBit(USR_BUTT);
  
  /* Enable the interrupt */
  GPIO_EXTICmd(USR_BUTT, ENABLE);
}
//

void USRB_EXTI_Reset(void)
{
	GPIO_EXTICmd(USR_BUTT, DISABLE);
}
//

void USRB_EXTI_Set(void)
{
	GPIO_EXTICmd(USR_BUTT, ENABLE);
}
//
