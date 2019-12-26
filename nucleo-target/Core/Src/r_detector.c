#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "r_detector.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "markers.h"

volatile int heartbeat = 0;
volatile int windows_counter = 0;


static float r_peak = 0;
static float max_r_peak = 0;
static float time = 0;
static float time_compare = 0;
static int current_threshold = 1800;
static float percent65 = 0.65f;
static float percent30 = 0.30f;
static float percent20 = 0.20f;
static int last_mean_sum = 0;;
static int last_mean = 1800;

volatile int adaptive_threshold_high = 2000;
volatile int adaptive_threshold_low = 600;

volatile bool detected = false;

static int state = 0;
//суммарно blind_period + low_det_period не может быть больше 0.5с

//0 - слепой период time + blind_period - x*0.65
//1 - период пониженного порога детекции time + blind_period + low_det_period - x*0.65
//2 - период снижени€ time + blind_period + low_det_period + 1с - x*0.30
//3 - time + blind_period + low_det_period + 1с + 0.5 - x*0.2
//возврат к 0 на любой стадии


void AdaptiveThresholding_high(int* data, int* peak_holder, int size, float d_freq, float blind_period, float low_det_period, float sensitivity)
{
	int x_m1 = 0;
	int x_p1 = 0;
	int x = 0;
	int x_m2 = 0;
	int x_p2 = 0;
	last_mean_sum = 0;
	for(int i = OVERLAP/2; i < size+OVERLAP/2; i++)
	{
			x_m2 = data[i-2];
			x_m1 = data[i-1];
			x = data[i];
			x_p1 = data[i+1];
			x_p2 = data[i+2];
			time += 1/d_freq;
			last_mean_sum += x;
		if(x >= current_threshold)
			{
				if((x - x_m1 > 0 && x - x_m2 > 0) &&
					(x - x_p1 > 0 && x - x_p2 > 0) && detected == false)
				{
					AppendMarker(&peak_holder[i-OVERLAP/2], MARK_R_PEAK);
					heartbeat++;
					r_peak = x;
					current_threshold = (int)(r_peak - last_mean)*0.80 + last_mean;
					time_compare = time + blind_period;
					detected = true;
					state = 0;
				}
					switch(state)
					{
						case 0:
							if(time >= time_compare && detected == true)
							{
								current_threshold = (int)(r_peak - last_mean)*0.65 + last_mean;
								time_compare += low_det_period;
								detected = false;
								state = 1;
							}
							break;
						case 1:
							if(time >= time_compare && detected == false)
							{
								time_compare += 1.0f;
								current_threshold = (int)(r_peak - last_mean)*0.30 + last_mean;
								state = 2;
							}
							break;
						case 2:
							if(time >= time_compare && detected == false)
							{
								time_compare += 0.5f;
								current_threshold = (int)(r_peak - last_mean)*0.20 + last_mean;
							}
							break;
					}
				}
			}
	last_mean = last_mean_sum/size;
}
//

//==================================================================================
//===========================Ёкспериментальные функции==============================
//==================================================================================

void Thresholding(int* data, int* peak_holder, int threshold, int size)
{
	int x_m1 = 0;
	int x_p1 = 0;
	int x = 0;
	int x_m2 = 0;
	int x_p2 = 0;
	
	for(int i = OVERLAP/2; i < size+OVERLAP/2; i++)
	{
			x_m2 = data[i-2];
			x_m1 = data[i-1];
			x = data[i];
			x_p1 = data[i+1];
			x_p2 = data[i+2];
			
			if(x >= threshold)
			{
				if((x - x_m1 >= 0 && x - x_m2 > 0) && 
					(x - x_p1 >= 0 && x - x_p2 > 0))
				{
					AppendMarker(&peak_holder[i-OVERLAP/2], MARK_R_PEAK);
					heartbeat++;
				}
			}
	}
}
//

volatile int tmp_heartbeat = 0;
const float alpha = 0.2f;
const float gamma = 0.2f;

void AdaptiveThresholding(int* data, int* peak_holder, int size)
{
	int x_m1 = 0;
	int x_m2 = 0;
	int x = 0;
	int x_p1 = 0;
	int x_p2 = 0;
	int mean_sum = 0;
	
	for(int i = OVERLAP/2; i < size+OVERLAP/2; i++)
	{
			x_m2 = data[i-2];
			x_m1 = data[i-1];
			x = data[i];
			mean_sum += data[i];
			x_p1 = data[i+1];
			x_p2 = data[i+2];
			
			if(x >= adaptive_threshold_high)
			{
				if((x - x_m1 > 0 && x - x_m2 > 0) && 
					(x - x_p1 > 0 && x - x_p2 > 0))
				{
					AppendMarker(&peak_holder[i-OVERLAP/2], MARK_R_PEAK);
					heartbeat++;
					adaptive_threshold_high = (int)(alpha*gamma*((float)x) - (1 - alpha)*(float)adaptive_threshold_high);
				}
			}
			else if(x <= adaptive_threshold_low)
			{
				if((x_m1 - x > 0 && x_m2 - x > 0) && 
					(x_p1 - x > 0 && x_p2 - x > 0))
				{
					AppendMarker(&peak_holder[i-OVERLAP/2], MARK_R_PEAK);
					heartbeat++;
					adaptive_threshold_low = (int)(alpha*gamma*((float)x) + (1 - alpha)*(float)adaptive_threshold_low);
				}
			}
	}
	tmp_heartbeat = heartbeat;
}
//
