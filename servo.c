#include "main.h"
#include "servo.h"

uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0;
uint16_t angleToUs(int16_t angle);
uint16_t angleToHz(int16_t angle);

uint16_t TimerPrescaler = 0, TimerClockDivision = 0;

void configureServo(void) {
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  uint16_t TimerPeriod = 0;



  /* GPIOA clocks enable */
  //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); //already enabled

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* GPIOA Configuration: Channel 1, 2 and 3 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // Servos need at least +4,5V to control. Using OD + pull-ups to +5V
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect TIM pins to AF6 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_6);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_6);

    /* Compute the value to be set in ARR register to generate signal frequency at 50 hz */
  TimerPrescaler = SystemCoreClock / 60000; //65.535 is the max, so 60.000 to be on the safe side
  TimerPeriod = (SystemCoreClock / (TimerPrescaler * 50)) - 1;

  /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
  Channel1Pulse = (uint16_t) (SystemCoreClock / (TimerPrescaler * angleToHz(0))) - 1;

  /* Compute CCR2 value to generate a duty cycle at 25%  for channel 2 */
  Channel2Pulse = (uint16_t) (SystemCoreClock / (TimerPrescaler * angleToHz(-60))) - 1;

  /* Compute CCR3 value to generate a duty cycle at 12.5%  for channel 3 */
  Channel3Pulse = (uint16_t) (SystemCoreClock / (TimerPrescaler * angleToHz(60))) - 1;

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = TimerPrescaler;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = TimerClockDivision;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2 and 3 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void setServoPositions(float *euler) {
  uint16_t scaler = SystemCoreClock / TimerPrescaler;
  TIM1->CCR1 = (uint16_t) (scaler/ angleToHz((int16_t)euler[0]));
  TIM1->CCR2 = (uint16_t) (scaler/ angleToHz((int16_t)euler[1]));
  TIM1->CCR3 = (uint16_t) (scaler/ angleToHz((int16_t)euler[2]));
}

uint16_t angleToUs(int16_t angle) {
  if (angle > 90) angle = 90;
  if (angle < -90) angle = -90;
  return (uint16_t)(1500 + (500 * angle)/90);
}

uint16_t angleToHz(int16_t angle) {
  return (uint16_t)(1000000l/angleToUs(angle));
}