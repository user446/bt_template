#ifndef __SW_TIMER_H
#define __SW_TIMER_H

#include <stdbool.h>

#define sw_timer_base_100us 1
#define sw_timer_base_ms 10
#define sw_timer_base_s 10000

#define sw_timer_100us_insec 10000
#define sw_timer_ms_insec 1000
#define sw_timer_s_insec 1

/**
	*	@brief	структура таймера
	*	@note		
**/
//
struct timer 
{
	volatile bool set;
	volatile uint32_t start;
	volatile uint32_t interval;
	void (*callback)(void);
	volatile bool on_interrupt;
	volatile bool autorestart;
};
//

void Timer_set(struct timer* t, uint32_t interval, uint32_t time_base, void (*callback)(void), bool on_interrupt, bool autorestart);
void Timer_reset(struct timer* t);
void Timer_restart(struct timer* t);
bool Timer_expired(struct timer* t);
uint32_t Timer_remaining(struct timer* t);

void t_OnDigitCompleteInterrupt(void);
void t_OnDigitCompleteContinuous(void);

#endif
