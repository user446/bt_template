#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "sw_timers.h"

static volatile uint64_t sys_tick_count;

static volatile uint8_t t_counter_interrupt = 0;
static volatile uint8_t t_counter_continuous = 0;
static struct timer* timer_stack[128];

/**
	*	@brief	Проверка таймера на окончание счетчика, сброс и выполнение колбека
	*	@note		
	*	@param	t ссылка на таймер
	*	@retval	нет
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
	*	@brief	Проверка всех существующих таймеров на истечение периода работы
	*	@note		Необходимо вставить в callback по окончанию прерывания основного таймера
	*	@note		Проверка производится только для таймеров имеющих атрибут in_interrupt = true
	*	@param	нет
	*	@retval	нет
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
	*	@brief	Проверка всех существующих таймеров на истечение периода работы
	*	@note		Необходимо вставить в основной цикл программы
	*	@note		Проверка производится только для таймеров имеющих атрибут in_interrupt = false
	*	@param	нет
	*	@retval	нет
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
	*	@brief	Установка таймера
	*	@note		
	*	@param	*t указатель на структуру таймера
	*	@param	interval период работы таймера
	*	@param	time_base позволяет выбрать базу для отсчета, в секундах или в милисекундах
	*	@param	callback указатель на функцию, которую следует исполнить по окончанию периода таймера
	* @param	on_interrupt вызывать callback таймера только по прерыванию
	*	@retval	нет
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
	*	@brief	Сброс таймера
	*	@note		
	*	@param	*t указатель на структуру таймера
	*	@retval	нет
**/
//
void Timer_reset(struct timer* t)
{
	t->start += t->interval;
}
//

/**
	*	@brief	Рестарт таймера
	*	@note		
	*	@param	*t указатель на структуру таймера
	*	@retval	нет
**/
//
void Timer_restart(struct timer* t)
{
	t->start = sys_tick_count;
}
//

/**
	*	@brief	Проверка на окончание периода работы
	*	@note		
	*	@param	*t указатель на структуру таймера
	*	@retval	истек ли интервал периода таймера
**/
//
bool Timer_expired(struct timer* t)
{
	volatile uint32_t diff = (sys_tick_count - t->start) + 1;
	return t->interval < diff;
}
//

/**
	*	@brief	Получить количество отсавшихся тиков таймера
	*	@note		
	*	@param	*t указатель на структуру таймера
	*	@retval	оставшееся количество тиков таймера до перезагрузки
**/
//
uint32_t Timer_remaining(struct timer* t)
{
	return t->start + t->interval - sys_tick_count;
}
//

