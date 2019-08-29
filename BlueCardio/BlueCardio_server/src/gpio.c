#include "BlueNRG_x_device.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"

void WUP_Initialize(void)
{
	/* Configure USER_BUTTON as interrupt sources */
	GPIO_InitType GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
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
  GPIO_EXTIConfigType GPIO_EXTIStructure;
  GPIO_EXTIStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_EXTIStructure.GPIO_IrqSense = GPIO_IrqSense_Edge;
  GPIO_EXTIStructure.GPIO_Event = IRQ_ON_BOTH_EDGE;
  GPIO_EXTIConfig(&GPIO_EXTIStructure);
	
  GPIO_ClearITPendingBit(GPIO_Pin_11);
  
  /* Enable the interrupt */
  GPIO_EXTICmd(GPIO_Pin_11, ENABLE);
}
