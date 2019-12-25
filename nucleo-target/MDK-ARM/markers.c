#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "markers.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

void AppendMarker(int* dest, int marker)
{
	int free_pos = 0;
	uint32_t mask = 0x00FF;
	while(((*dest)&mask) != 0)
	{
		free_pos++;
		mask = mask << 8;
	}
	(*dest) = (*dest)|(marker<<(free_pos*8));
}
//

void ParseMarkers(int marker_signs, char* markers,	int size)
{
	int tmp = 0;
	int n = 0;
	memset(markers, 0, sizeof(markers[0])*size);
	markers[0] = NO_MARKER_CHAR;
	tmp = marker_signs;
	for(int i = 0; i < size; i++)
	{
		n = 0;
		while(tmp)
		{
			markers[i+n] = (char)(tmp&0x00FF);
			tmp = tmp >> 8;
			n++;
		}
	}
}
//