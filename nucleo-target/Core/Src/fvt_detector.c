#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "fvt_detector.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "markers.h"
#include "sw_timers.h"

static float	fvt_disc_frequency = 0;	///	частота дискретизации, позволяет простым умножением подсчитывать интервал R пика

static int 		fvt_window_size = 0;						///размер окна для хранения данных о пиках
static float*	fvt_window_ptr;									///указатель на окно
static int 		fvt_window_fullness = 0;				///заполненность окна, нужна для запуска проверки
static int		fvt_window_fail_counter	=	0;		///счетчик отрицательных проверок в окне
static int 		fvt_window_last_part_count = 0;	///хранит количество позиций с последнего R пика до конца окна в окне маркеров

static bool 	fvt_detected = false;	///Флаг обозначающий то, что БЖТ было задетектировано
static bool		fvt_onfinish = false;	///Флаг, включаемый когда необходимо обработать возможное завершение БЖТ

static float 	fvt_last_r_peak = 0;				///Дельта между предыдущим и последним R пиком
static float 	fvt_interval = 0;						///критический интервал, с которым сравниваются дельты в окне БЖТ
static float	fvt_average_period = 0;			///средний интервал между пиками, позволяет расчитать частоту сердцебиения
static float 	fvt_peak_time_sum = 0;			///используется для хранения суммы дельт R пиков
static float	fvt_onfinish_time_sum = 0;	///сумма дельт R пиков для второго условия остановки (по таймеру за 20 секунд)
static int		fvt_detect_after_count = 0;	///число положительных срабатываний детектора, после которого БЖТ считается задетектированным
static int*		fvt_markers_ptr;						///указатель на массив маркеров, по которому проводится проверка
static int		fvt_markers_size	= 0;			///размер массива маркеров
static int		fvt_onfinish_count = 0;			///счетчик сброса детекции для первого условия остановки (число последовательных R пиков)

static void fvt_CheckWindow(void);
static void InsertInWindow(float data);
static void CheckFinishFVT(void);
static void OnNoRpeaks(void);

struct timer t_finish_fvt;				///таймер для второго условия остановки
struct timer t_tensec_period;			///таймер для третьего условия остановки
static const float timer_freq = 1/20;
static const float onfinish_time = 20.0f; //секунды

void fvt_InitDetector(float* fvt_window, int window_size, int* markers, int mk_size, float dfreq, float interval, int fvt_count)
{
	fvt_disc_frequency = dfreq;
	fvt_window_ptr = fvt_window;
	fvt_markers_ptr = markers;
	fvt_markers_size = mk_size;
	fvt_window_size = window_size;
	fvt_interval = interval;
	fvt_detect_after_count = fvt_count;
	
	//инициализация таймера, срабатывание раз в 20 секунд, без автоперезагрузки, изначально выключен
	Timer_set(&t_finish_fvt, timer_freq, &CheckFinishFVT, true, false, false);
	//инициализация таймера, срабатывание раз в 10 секунд
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

//обработчик таймера, срабатывает через timer_freq после запуска таймера
static void CheckFinishFVT(void)
{
	if(fvt_onfinish)
	{
		if(fvt_interval <= fvt_onfinish_time_sum/onfinish_time)
			fvt_detected = false;		//второе условие сброса
		else
			fvt_detected = true;
		fvt_onfinish_time_sum = 0;
		fvt_onfinish = false;
	}
}
//

//обработчик таймера, срабатывает каждые 10 секунд, если не был сброшен
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
			if(fvt_window_fullness < fvt_window_size)	//считаем, пока окно не заполнится целиком
			{
				fvt_window_fullness++;
			}
			else			//если окно заполнено, то начинаем проверять окна с каждым найденным r пиком
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
	
	if(fvt_window_fail_counter >= fvt_detect_after_count)	//если в окне зафиксированы только пики меньше нормального периода
	{
		fvt_detected = true;		//условие начала подсчета
		fvt_onfinish = true;
		Timer_enable(&t_finish_fvt);
	}
	if(fvt_onfinish_count >= fvt_detect_after_count) //если в окне зафиксированы только пики с нормальным периодом
		fvt_detected = false;	//первое условие сброса
	
	fvt_peak_time_sum += fvt_window_ptr[0];
	fvt_average_period = fvt_peak_time_sum/fvt_window_size;
}
//

