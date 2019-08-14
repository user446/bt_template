#ifndef __ADC_H__ 
#define __ADC_H__

#define ADC_DMA_BUFFER_LEN      (2)

void ADC_Initialize(void);
int ADC_Ready(void);
void ADC_Start(void);
int	ADC_GetData(float *buf, int max_size);

#endif 
