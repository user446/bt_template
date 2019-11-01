#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "sw_timers.h"

static volatile uint64_t sys_tick_count;

static volatile uint8_t t_counter_interrupt = 0;
static volatile uint8_t t_counter_continuous = 0;
static struct timer* timer_stack[128];

/**
	*	@brief	�������� ������� �� ��������� ��������, ����� � ���������� �������
	*	@note		
	*	@param	t ������ �� ������
	*	@retval	���
**/
//
static void CheckTimer(struct timer* t) 
{
	if(Timer_expired(t) == true && t->set == true) 
	{ 
		if(t->autorestart)
			Timer_restart(t);
		t->callback();
	}
}
//


/**
	*	@brief	�������� ���� ������������ �������� �� ��������� ������� ������
	*	@note		���������� �������� � callback �� ��������� ���������� ��������� �������
	*	@note		�������� ������������ ������ ��� �������� ������� ������� in_interrupt = true
	*	@param	���
	*	@retval	���
**/
//
volatile uint8_t timer_counter_int = 0;
void t_OnDigitCompleteInterrupt(void)
{
	sys_tick_count++;
	timer_counter_int = 0;
 	while(timer_counter_int < t_counter_interrupt)
	{
		if(timer_stack[timer_counter_int]->on_interrupt == true)
			CheckTimer(timer_stack[timer_counter_int]);
		timer_counter_int++;
	}
}
//

/**
	*	@brief	�������� ���� ������������ �������� �� ��������� ������� ������
	*	@note		���������� �������� � �������� ���� ���������
	*	@note		�������� ������������ ������ ��� �������� ������� ������� in_interrupt = false
	*	@param	���
	*	@retval	���
**/
//
volatile uint8_t timer_counter_cont = 0;
void t_OnDigitCompleteContinuous(void)
{
	timer_counter_cont = 0;
 	while(timer_counter_cont < t_counter_continuous)
	{
		if(timer_stack[timer_counter_cont]->on_interrupt == false)
			CheckTimer(timer_stack[timer_counter_cont]);
			timer_counter_cont++;
	}
}
//

/**
	*	@brief	��������� �������
	*	@note		
	*	@param	*t ��������� �� ��������� �������
	*	@param	interval ������ ������ �������
	*	@param	time_base ��������� ������� ���� ��� �������, � �������� ��� � ������������
	*	@param	callback ��������� �� �������, ������� ������� ��������� �� ��������� ������� �������
	* @param	on_interrupt �������� callback ������� ������ �� ����������
	*	@retval	���
**/
//
void Timer_set(struct timer* t, uint32_t interval, uint32_t time_base, void (*callback)(void), bool on_interrupt, bool autorestart)
{
	t->set = true;
	t->interval = interval * time_base;
	t->start = sys_tick_count;
	t->callback = callback;
	timer_stack[t_counter_interrupt + t_counter_continuous] = t;
	t->on_interrupt = on_interrupt;
	t->autorestart = autorestart;
	if(on_interrupt)
		t_counter_interrupt++;
	else
		t_counter_continuous++;
}
//

/**
	*	@brief	����� �������
	*	@note		
	*	@param	*t ��������� �� ��������� �������
	*	@retval	���
**/
//
void Timer_reset(struct timer* t)
{
	t->start += t->interval;
}
//

/**
	*	@brief	������� �������
	*	@note		
	*	@param	*t ��������� �� ��������� �������
	*	@retval	���
**/
//
void Timer_restart(struct timer* t)
{
	t->start = sys_tick_count;
}
//

/**
	*	@brief	�������� �� ��������� ������� ������
	*	@note		
	*	@param	*t ��������� �� ��������� �������
	*	@retval	����� �� �������� ������� �������
**/
//
bool Timer_expired(struct timer* t)
{
	volatile uint32_t diff = (sys_tick_count - t->start) + 1;
	return t->interval < diff;
}
//

/**
	*	@brief	�������� ���������� ���������� ����� �������
	*	@note		
	*	@param	*t ��������� �� ��������� �������
	*	@retval	���������� ���������� ����� ������� �� ������������
**/
//
uint32_t Timer_remaining(struct timer* t)
{
	return t->start + t->interval - sys_tick_count;
}
//

