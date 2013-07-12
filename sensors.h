#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"

#define OUT_QUATERNION
//#define OUT_EULER
//#define OUT_ACCEL
//#define OUT_GYRO
//#define OUT_MAG

#define PI                         (float)     3.14159265f

void UpdateGyroBias(void);
void Demo_GyroConfig(void);
void Demo_CompassConfig(void);
void updateQuaternions(float * quatBuf);
void getEulerAngles(float *euler);
void getEulerAsArray(uint8_t *eulerArr);
void readAllSensors(uint8_t *GyroTempBuf, uint8_t *AccTempBuf, uint8_t *MagTempBuf);

extern float QuaternionsBuffer[4];
extern uint8_t MagTempBuffer[6], AccTempBuffer[6], GyroTempBuffer[6];
extern float euler[3];
extern uint8_t eulerArr[6];
extern uint8_t GyroDRDFlag;

#endif //#ifndef SENSORS_H