#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "fvt_detector.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#define	R_PEAK		0x52
#define S_PEAK		0x53
#define SR_PEAK		0x5253


void fvt_InitDetector(struct fvt_detector* fvt, float dfreq, float critical_period, int window_size, bool enabled)
{
	fvt->d_freq = dfreq;
	fvt->average_fvt_period = 0;
	fvt->fvt_count = 0;
	fvt->fvt_window_size = window_size;
	fvt->last_r_peak = 0;
	fvt->fvt_window = (float*)malloc((window_size)*sizeof(float));
	//memset(fvt->fvt_window, 0, window_size*sizeof(float));
	fvt->fvt_critical_period = critical_period;
	fvt->window_fvt_counter = 0;
	fvt->end_counter = 0;
	fvt->window_fill = 0;
	fvt->detected = false;
	fvt->IsOn = enabled;
}
//

bool IsDetected(struct fvt_detector* fvt)
{
	return fvt->detected;
}
//

void InsertInWindow(struct fvt_detector* fvt, float data)
{
	for(int i = 0; i < fvt->fvt_window_size; i++) 
		fvt->fvt_window[i] = fvt->fvt_window[i+1];
	fvt->fvt_window[fvt->fvt_window_size-1] = data;
}
//

void fvt_FindLast(struct fvt_detector* fvt, int* markers, int size)
{
	int i_delta = 0;
	for(int i = 0; i < size; i++)
	{
		if(markers[i] == R_PEAK || markers[i] == SR_PEAK)
		{
			fvt->last_r_peak = ((float)(fvt->end_counter + i - i_delta)*(1/fvt->d_freq));
			i_delta = i;
			InsertInWindow(fvt, fvt->last_r_peak);
			if(fvt->window_fill < fvt->fvt_window_size)
				fvt->window_fill++;
		}
	}
	fvt->end_counter = size - i_delta;
}
//

bool fvt_CheckWindow(struct fvt_detector* fvt)
{
	bool ret = false;
	fvt->window_fvt_counter = 0;
	for(int i = fvt->fvt_window_size; i >= 1; i--)
	{
		if(fvt->fvt_window[i] - fvt->fvt_window[i-1] <= fvt->fvt_critical_period)
			fvt->window_fvt_counter++;
	}
	if(fvt->window_fvt_counter >= fvt->fvt_count)
		ret = true;
	return ret;
}
//

void ResetDetector(struct fvt_detector* fvt)
{
	
}
//
