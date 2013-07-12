/* timing.c
* timing functions
* declarations are in timing.h */

#include "timing.h"

volatile uint32_t timeCounter100us = 0;
volatile uint32_t timeCounter1ms = 0;
volatile uint32_t timeCounter10ms = 0;
volatile uint32_t timeCounter20ms = 0;
volatile uint32_t timeCounter50ms = 0;
volatile uint32_t timeCounter100ms = 0;
volatile uint32_t timeCounter1s = 0;
volatile uint32_t timeCounter5s = 0;

volatile uint32_t TimingDelay = 0;

void timeBase100us (void);
void timeBaseUnifier (volatile uint32_t *mytimeCounter, uint32_t myTimeBasesThreshold, void (*callBack)(void));


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay_1ms(__IO uint32_t nTime)
{
  TimingDelay = nTime * 10;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

/**
  * @brief Periodic 10 us call
  * @param None
  * @retval None
  */

void timeBaseUpgradeCounters (void) {
   timeCounter1ms++;
   timeCounter10ms++;
   timeCounter20ms++;
   timeCounter50ms++;
   timeCounter100ms++;
   timeCounter1s++;
   timeCounter5s++;
}

void timeBaseUnifier (volatile uint32_t *mytimeCounter, uint32_t myTimeBasesThreshold, void (*callBack)(void)) {
  if (*mytimeCounter < myTimeBasesThreshold) return;
  *mytimeCounter = 0;

  callBack();
}


void timeBase1ms (void (*callBack)(void)) {
  timeBaseUnifier (&timeCounter1ms, 10, callBack);
}

void timeBase10ms (void (*callBack)(void)) {
  timeBaseUnifier (&timeCounter10ms, 100, callBack);
}

void timeBase20ms (void (*callBack)(void)) {
  timeBaseUnifier (&timeCounter20ms, 200, callBack);
}

void timeBase50ms (void (*callBack)(void)) {
  timeBaseUnifier (&timeCounter50ms, 500, callBack);
}

void timeBase100ms (void (*callBack)(void)) {
  timeBaseUnifier (&timeCounter100ms, 1000, callBack);
}

void timeBase1s (void (*callBack)(void)) {
  timeBaseUnifier (&timeCounter1s, 10000, callBack);
}

void timeBase5s (void (*callBack)(void)) {
  timeBaseUnifier (&timeCounter5s, 50000, callBack);
}
