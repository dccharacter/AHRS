/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "main.h"


/** @addtogroup STM32F3-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define ABS(x)         (x < 0) ? (-x) : x

/* Private variables ---------------------------------------------------------*/
  RCC_ClocksTypeDef RCC_Clocks;

__IO uint32_t USBConnectTimeOut = 100;
__IO uint8_t UserButtonPressed = 0;

__IO uint8_t PrevXferComplete = 1;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int putchar(int ch);
void TimeBase_Do1ms (void);
void TimeBase_Do10ms (void);
void TimeBase_Do20ms (void);
void TimeBase_Do50ms (void);
void TimeBase_Do100ms (void);
void TimeBase_Do1s (void);
void TimeBase_Do5s (void);
void USB_Config (void);

#define SERVO

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /* SysTick end of count event each 10ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  uint32_t sysTickFrequency = 10000; //10KHz
  SysTick_Config(RCC_Clocks.HCLK_Frequency / sysTickFrequency);

  /* Initialize LEDs and User Button available on STM32F3-Discovery board */
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);
  STM_EVAL_LEDInit(LED7);
  STM_EVAL_LEDInit(LED8);
  STM_EVAL_LEDInit(LED9);
  STM_EVAL_LEDInit(LED10);

  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

  Set_System();
#ifdef USART_USE_USB
  /* Configure the USB */
  USB_Config();
#endif //#ifdef USART_USE_USB

  Demo_GyroConfig();
  UpdateGyroBias();
  Demo_CompassConfig();

#ifdef SERVO
  configureServo();
#endif //#ifdef SERVO

  myPrintf("\f\f");

  GyroExtiConfig();
  readAllSensors(GyroTempBuffer, AccTempBuffer, MagTempBuffer);
  GyroDRDFlag = 1;


  /* Infinite loop */
  while (1)
  {

    timeBase1ms (TimeBase_Do1ms);
    timeBase10ms (TimeBase_Do10ms);
    timeBase20ms (TimeBase_Do20ms);
    timeBase50ms (TimeBase_Do50ms);
    timeBase100ms (TimeBase_Do100ms);
    timeBase1s (TimeBase_Do1s);
    timeBase5s (TimeBase_Do5s);
    if (GyroDRDFlag) {
      GyroDRDFlag = 0;
      updateQuaternions(QuaternionsBuffer);
    }
  }
}


void TimeBase_Do1ms (void) {
}

void TimeBase_Do10ms (void) {

}

void TimeBase_Do20ms (void) {
#ifdef USB_JOYSTICK
  static uint8_t Mouse_Buffer[7] = {0};
  getEulerAsArray(Mouse_Buffer);
  if (STM_EVAL_PBGetState(BUTTON_USER) == RESET) {
    Mouse_Buffer[6] = 0; } else {
      Mouse_Buffer[6] = 1; }

  /* Reset the control token to inform upper layer that a transfer is ongoing */
  PrevXferComplete = 0;

  /* Copy mouse position info in ENDP1 Tx Packet Memory Area*/
  USB_SIL_Write(EP1_IN, Mouse_Buffer, 7);

  /* Enable endpoint for transmission */
  SetEPTxValid(ENDP1);
#endif

#ifdef SERVO
  getEulerAngles(euler);
  setServoPositions(euler);
#endif //#ifdef SERVO
}

void TimeBase_Do50ms (void) {

#ifdef OUT_QUATERNION
  STM_EVAL_LEDOn(LED10);
  myPrintf("%f,%f,%f,%f,\n", QuaternionsBuffer[0], QuaternionsBuffer[1], QuaternionsBuffer[2], QuaternionsBuffer[3]);
  STM_EVAL_LEDOff(LED10);

#endif //#ifdef OUT_QUATERNION

#ifdef OUT_EULER
  myPrintf("%f,%f,%f,\r\n", euler[0], euler[1], euler[2]);
#endif //#ifdef OUT_EULER
}

void TimeBase_Do100ms (void) {
  STM_EVAL_LEDToggle(LED5);
}

void TimeBase_Do1s (void) {
  STM_EVAL_LEDToggle(LED7);
}

void TimeBase_Do5s (void) {
  STM_EVAL_LEDToggle(LED9);
}

#ifdef USART_USE_USART
int putchar(int ch) {
  while (!USART_GetFlagStatus(USART2, USART_FLAG_TC));
  USART_SendData(USART2, ch);
  return ch;
}
#endif //#ifdef USART_USE_USART

#ifdef USART_USE_USB
/**
  * @brief  Configure the USB.
  * @param  None
  * @retval None
  */
void USB_Config(void)
{
  Set_USBClock();
  USB_Interrupts_Config();

  USB_Init();

//!!!зависнет, пока не подкл. usb
  //while (bDeviceState != CONFIGURED)
  //{}
}
#endif //#ifdef USART_USE_USB

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
