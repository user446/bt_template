#ifndef __FVT_DETECT__
#define __FVT_DETECT__

#include "stdbool.h"

#define FVT_NO 			0
#define FVT_BEGIN 	-1
#define FVT_START		1

typedef struct fvt_result
{
	int 	index;
	int		found;
}t_fvt_result;

void fvt_InitDetector(float* fvt_window, int window_size, int* markers, int mk_size, float dfreq, float interval, int fvt_count);
t_fvt_result fvt_SeekForFVT(void);
bool fvt_IsDetected(void);

#endif
