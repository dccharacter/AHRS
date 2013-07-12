/* timing.h */

#ifndef TIMING_H
#define TIMING_H

#include "stm32f30x.h"

void TimingDelay_Decrement(void);
void Delay_1ms(volatile uint32_t nTime);

void timeBaseUpgradeCounters (void);

void timeBase1ms (void (*callBack)(void));
void timeBase10ms (void (*callBack)(void));
void timeBase20ms (void (*callBack)(void));
void timeBase50ms (void (*callBack)(void));
void timeBase100ms (void (*callBack)(void));
void timeBase1s (void (*callBack)(void));
void timeBase5s (void (*callBack)(void));

extern volatile uint32_t timeCounter100us;

extern volatile uint32_t TimingDelay;

#endif //#ifndef TIMING_H
