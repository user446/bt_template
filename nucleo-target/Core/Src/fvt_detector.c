#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "fvt_detector.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "markers.h"
#include "sw_timers.h"
#include "math.h"

static float	fvt_disc_frequency = 0;	///	������� �������������, ��������� ������� ���������� ������������ �������� R ����

static int 		fvt_window_size = 0;						///������ ���� ��� �������� ������ � �����
static float*	fvt_window_ptr;									///��������� �� ����
static int 		fvt_window_fullness = 0;				///������������� ����, ����� ��� ������� ��������
static int		fvt_window_fail_counter	=	0;		///������� ������������� �������� � ����
static int 		fvt_window_last_part_count = 0;	///������ ���������� ������� � ���������� R ���� �� ����� ���� � ���� ��������

static int 		fvt_detected = FVT_UNDEFINED;	///���� ������������ ��, ��� ��� ���� ���������������

static float 	fvt_last_r_peak = 0;				///������ ����� ���������� � ��������� R �����
static float 	fvt_interval = 0;						///����������� ��������, � ������� ������������ ������ � ���� ���
static float	fvt_average_period = 0;			///������� �������� ����� ������, ��������� ��������� ������� ������������
static float 	fvt_peak_time_sum = 0;			///������������ ��� �������� ����� ����� R �����
static int		fvt_detect_after_count = 0;	///����� ������������� ������������ ���������, ����� �������� ��� ��������� �����������������
static int*		fvt_markers_ptr;						///��������� �� ������ ��������, �� �������� ���������� ��������
static int		fvt_markers_size	= 0;			///������ ������� ��������

static float 	fvt_20_second_r_peak_sum = 0;
static int		fvt_20_second_r_peak_count = 0;			///������� ������ �������� ��� ������� ������� ��������� (����� ���������������� R �����)
static float		fvt_timer = 0;							///������� ����� �� ����������� �������� � ��������
static float		fvt_10_second_mark_time = 0;
static float		fvt_20_second_mark_time = 0;

static int fvt_CheckWindow(void);
static void InsertInWindow(float data);

static const float t20_seconds_normal = 20.0f; //�������
static const float t10_seconds_no_r = 10.0f;

void fvt_InitDetector(float* fvt_window, int window_size, int* markers, int mk_size, float dfreq, float interval, int fvt_count)
{
	fvt_disc_frequency = dfreq;
	fvt_window_ptr = fvt_window;
	fvt_markers_ptr = markers;
	fvt_markers_size = mk_size;
	fvt_window_size = window_size;
	fvt_interval = interval;
	fvt_detect_after_count = fvt_count;
}
//

bool fvt_IsDetected(void)
{
	return fvt_detected;
}
//

static void InsertInWindow(float data)
{
	for(int i = 0; i < fvt_window_size; i++) 
		fvt_window_ptr[i] = fvt_window_ptr[i+1];
	fvt_window_ptr[fvt_window_size-1] = data;
}
//

t_fvt_result ret;
t_fvt_result fvt_SeekForFVT(void)
{
	int i_delta = 0;
	
	if(fvt_detected == FVT_FINISH)
		fvt_detected = FVT_UNDEFINED;
		
	for(int i = 0; i < fvt_markers_size; i++)
	{
		fvt_timer += 1/fvt_disc_frequency;
		if(SearchFor(MARK_R_PEAK, fvt_markers_ptr[i]))
		{
			fvt_last_r_peak = ((float)(fvt_window_last_part_count + i - i_delta)*(1/fvt_disc_frequency));
			i_delta = i;
			InsertInWindow(fvt_last_r_peak);
			fvt_10_second_mark_time = fvt_timer + t10_seconds_no_r;
			if(fvt_window_fullness < fvt_window_size)	//�������, ���� ���� �� ���������� �������
			{
				fvt_window_fullness++;
			}
			else if(fvt_CheckWindow() >= fvt_detect_after_count)
			{
				if(fvt_detected == FVT_UNDEFINED)
					fvt_detected = FVT_BEGIN;
				else fvt_detected = FVT_ONGOING;
				ret.index = i;
				fvt_20_second_r_peak_count = 0;
				fvt_20_second_r_peak_sum = 0;
				fvt_20_second_mark_time = fvt_timer + t20_seconds_normal;
			}
			if(fvt_detected && fvt_20_second_mark_time >= fvt_timer)
			{
				fvt_20_second_r_peak_count++;
				fvt_20_second_r_peak_sum += fvt_last_r_peak;
			}
			else if(fvt_detected && fvt_20_second_mark_time <= fvt_timer)
			{
				if(fvt_20_second_r_peak_sum/fvt_20_second_r_peak_count >= fvt_interval && fvt_detected == FVT_ONGOING)
				{
					fvt_detected = FVT_FINISH;
					ret.index = i;
				}
			}
		}
		if(fvt_timer >= fvt_10_second_mark_time && fvt_detected == FVT_ONGOING)
		{
			fvt_detected = FVT_FINISH;
			ret.index = i;
		}
	}
	fvt_window_last_part_count = fvt_markers_size - i_delta;
	
	ret.found = fvt_detected;
	return ret;
}
//

int fvt_CheckWindow(void)
{
	int peak_time_sum = 0;
	int interval_too_small = 0;
	for(int i = 0; i < fvt_window_size; i++)
	{
		fvt_peak_time_sum += fvt_window_ptr[i];
		if(fvt_window_ptr[i] <= fvt_interval)
			interval_too_small++;
	}
	fvt_average_period = peak_time_sum/fvt_window_size;
	fvt_window_fail_counter = interval_too_small;
	return interval_too_small;
}
//

