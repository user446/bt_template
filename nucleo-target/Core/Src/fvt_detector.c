#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "fvt_detector.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "markers.h"
#include "sw_timers.h"

static float	fvt_disc_frequency = 0;	///	������� �������������, ��������� ������� ���������� ������������ �������� R ����

static int 		fvt_window_size = 0;						///������ ���� ��� �������� ������ � �����
static float*	fvt_window_ptr;									///��������� �� ����
static int 		fvt_window_fullness = 0;				///������������� ����, ����� ��� ������� ��������
static int		fvt_window_fail_counter	=	0;		///������� ������������� �������� � ����
static int 		fvt_window_last_part_count = 0;	///������ ���������� ������� � ���������� R ���� �� ����� ���� � ���� ��������

static bool 	fvt_detected = false;	///���� ������������ ��, ��� ��� ���� ���������������
static bool		fvt_onfinish = false;	///����, ���������� ����� ���������� ���������� ��������� ���������� ���

static float 	fvt_last_r_peak = 0;				///������ ����� ���������� � ��������� R �����
static float 	fvt_interval = 0;						///����������� ��������, � ������� ������������ ������ � ���� ���
static float	fvt_average_period = 0;			///������� �������� ����� ������, ��������� ��������� ������� ������������
static float 	fvt_peak_time_sum = 0;			///������������ ��� �������� ����� ����� R �����
static float	fvt_onfinish_time_sum = 0;	///����� ����� R ����� ��� ������� ������� ��������� (�� ������� �� 20 ������)
static int		fvt_detect_after_count = 0;	///����� ������������� ������������ ���������, ����� �������� ��� ��������� �����������������
static int*		fvt_markers_ptr;						///��������� �� ������ ��������, �� �������� ���������� ��������
static int		fvt_markers_size	= 0;			///������ ������� ��������
static int		fvt_onfinish_count = 0;			///������� ������ �������� ��� ������� ������� ��������� (����� ���������������� R �����)

static void fvt_CheckWindow(void);
static void InsertInWindow(float data);
static void CheckFinishFVT(void);
static void OnNoRpeaks(void);

struct timer t_finish_fvt;				///������ ��� ������� ������� ���������
struct timer t_tensec_period;			///������ ��� �������� ������� ���������
static const float timer_freq = 1/20;
static const float onfinish_time = 20.0f; //�������

void fvt_InitDetector(float* fvt_window, int window_size, int* markers, int mk_size, float dfreq, float interval, int fvt_count)
{
	fvt_disc_frequency = dfreq;
	fvt_window_ptr = fvt_window;
	fvt_markers_ptr = markers;
	fvt_markers_size = mk_size;
	fvt_window_size = window_size;
	fvt_interval = interval;
	fvt_detect_after_count = fvt_count;
	
	//������������� �������, ������������ ��� � 20 ������, ��� ����������������, ���������� ��������
	Timer_set(&t_finish_fvt, timer_freq, &CheckFinishFVT, true, false, false);
	//������������� �������, ������������ ��� � 10 ������
	Timer_set(&t_tensec_period, 1/10, &OnNoRpeaks, true, true, true);
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

//���������� �������, ����������� ����� timer_freq ����� ������� �������
static void CheckFinishFVT(void)
{
	if(fvt_onfinish)
	{
		if(fvt_interval <= fvt_onfinish_time_sum/onfinish_time)
			fvt_detected = false;		//������ ������� ������
		else
			fvt_detected = true;
		fvt_onfinish_time_sum = 0;
		fvt_onfinish = false;
	}
}
//

//���������� �������, ����������� ������ 10 ������, ���� �� ��� �������
static void OnNoRpeaks(void)
{
	fvt_detected = false;
	fvt_onfinish = false;
}
//

t_fvt_result fvt_SeekForFVT(void)
{
	int i_delta = 0;
	t_fvt_result ret;
	
	for(int i = 0; i < fvt_markers_size; i++)
	{
		if(SearchFor(MARK_R_PEAK, fvt_markers_ptr[i]))
		{
			fvt_last_r_peak = ((float)(fvt_window_last_part_count + i - i_delta)*(1/fvt_disc_frequency));
			i_delta = i;
			InsertInWindow(fvt_last_r_peak);
			Timer_restart(&t_tensec_period);
			if(fvt_window_fullness < fvt_window_size)	//�������, ���� ���� �� ���������� �������
			{
				fvt_window_fullness++;
			}
			else			//���� ���� ���������, �� �������� ��������� ���� � ������ ��������� r �����
			{
				fvt_CheckWindow();
			}
		}
	}
	fvt_window_last_part_count = fvt_markers_size - i_delta;
}
//

void fvt_CheckWindow(void)
{
	fvt_window_fail_counter = 0;
	fvt_onfinish_count = 0;
	fvt_peak_time_sum = 0;
	for(int i = 0; i < fvt_window_size; i++)
	{
		fvt_peak_time_sum += fvt_window_ptr[i];
		if(fvt_onfinish)
			fvt_onfinish_time_sum += fvt_window_ptr[i];
		if(fvt_window_ptr[i] <= fvt_interval)
			fvt_window_fail_counter++;
		else
		{
			fvt_onfinish_count++;
		}
	}
	
	if(fvt_window_fail_counter >= fvt_detect_after_count)	//���� � ���� ������������� ������ ���� ������ ����������� �������
	{
		fvt_detected = true;		//������� ������ ��������
		fvt_onfinish = true;
		Timer_enable(&t_finish_fvt);
	}
	if(fvt_onfinish_count >= fvt_detect_after_count) //���� � ���� ������������� ������ ���� � ���������� ��������
		fvt_detected = false;	//������ ������� ������
	
	fvt_peak_time_sum += fvt_window_ptr[0];
	fvt_average_period = fvt_peak_time_sum/fvt_window_size;
}
//

