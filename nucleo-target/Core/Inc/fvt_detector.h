#ifndef __FVT_DETECT__
#define __FVT_DETECT__

#include "stdbool.h"

#define STATE_NOCHANGE -1

struct fvt_detector
{
	float d_freq;
	float last_r_peak;
	float average_fvt_period;
	float fvt_critical_period;
	int fvt_count;
	int fvt_window_size;
	int end_counter;
	int window_fvt_counter;
	int window_fill;
	float* fvt_window;
	bool detected;
	bool IsOn;
};
//

void fvt_InitDetector(struct fvt_detector* fvt, float dfreq, float critical_period, int window_size, bool enabled);
bool IsDetected(struct fvt_detector* fvt);
void fvt_FindLast(struct fvt_detector* fvt, int* markers, int size);
void ResetDetector(struct fvt_detector* fvt);
bool fvt_CheckWindow(struct fvt_detector* fvt);

#endif
