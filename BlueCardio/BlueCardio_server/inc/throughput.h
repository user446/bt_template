
#ifndef _THROUGHPUT_H_
#define _THROUGHPUT_H_

#define NUM_PACKETS 500 
#define CONVERSION_NUM 4

typedef union {
	float update_buffer_f[CONVERSION_NUM];
	uint32_t update_buff_u32[CONVERSION_NUM];
	uint8_t update_buff_u8[CONVERSION_NUM*4];
}update_value;

uint8_t THROUGHPUT_DeviceInit(void);
void APP_Tick(void);

#endif // _THROUGHPUT_H_
