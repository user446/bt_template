#ifndef __R_DETECT__
#define __R_DETECT__

#define OVERLAP 32			//window overlap in detection algorithm

void Thresholding(int* data, int* peak_holder, int threshold, int size);
void AdaptiveThresholding(int* data, int* peak_holder, int size);

#endif
