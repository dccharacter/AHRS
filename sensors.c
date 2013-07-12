#include "sensors.h"
#include <math.h>

#define USE_MAG

#define USE_MADGWICK_AHRS
//#define USE_MAHONY_AHRS

#ifdef USE_MAHONY_AHRS
#include "MahonyAHRS.h"
#else
#include "MadgwickAHRS.h"
#endif

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */


#define QW q[0]
#define QX q[1]
#define QY q[2]
#define QZ q[3]

void AHRS_GetValues(float * val);
void readAllSensors(uint8_t *GyroTempBuf, uint8_t *AccTempBuf, uint8_t *MagTempBuf);
void readRawGyro(uint8_t *GyroTempBuf);
void processGyroData(float *GyroBuf, uint8_t *GyroTempBuf);
void processAccelData(float *AccBuf, uint8_t *AccTempBuf);
void processMagnetoData(float *MagBuf, uint8_t *MagTempBuf);
uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
float GyroSensitivity = 0;
uint8_t Gyro_LBEFlag = 0;
uint8_t GyroDRDFlag = 0;
uint8_t Accel_LBEorFIFOFlag = 0;
float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
uint8_t Accel_cDivider;

void myFusion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

//float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;
//float fTiltedX,fTiltedY = 0.0f;

float MagBuffer[3] = {0.0f}, AccBuffer[3] = {0.0f}, GyroBuffer[3] = {0.0f};
uint8_t MagTempBuffer[6] = {0.0f}, AccTempBuffer[6] = {0.0f}, GyroTempBuffer[6] = {0.0f};
float QuaternionsBuffer[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float euler[3] = {0.0f};
uint8_t eulerArr[6] = {0};
float GyroCorrectionCoeffs[3] = {0.0f};

void myFusion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float recipNorm;
  q0 = 0;
  q1 = gx;
  q2 = gy;
  q3 = gz;

  // Normalise quaternion
  recipNorm = 1/sqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}


void updateQuaternions(float * quatBuf) {
float val[9] = {0.0f};
AHRS_GetValues(val);

#ifdef USE_MADGWICK_AHRS
#ifdef USE_MAG
  MadgwickAHRSupdate(val[0]*PI/180.0, val[1]*PI/180.0, val[2]*PI/180.0, val[3], val[4], val[5], val[6], val[7], val[8]);
#else
  MadgwickAHRSupdate(val[0]*PI/180.0, val[1]*PI/180.0, val[2]*PI/180.0, val[3], val[4], val[5], 0.0f, 0.0f, 0.0f);
  //MadgwickAHRSupdate(0.0f, 0.0f, 0.0f, 100, 100, 1100, 0.0f, 0.0f, 0.0f);
#endif //#ifdef USE_MAG

#elif defined USE_MAHONY_AHRS

#ifdef USE_MAG
  MahonyAHRSupdate(val[0]*PI/180.0, val[1]*PI/180.0, val[2]*PI/180.0, val[3], val[4], val[5], val[6], val[7], val[8]);
#else
  MahonyAHRSupdate(val[0]*PI/180.0, val[1]*PI/180.0, val[2]*PI/180.0, val[3], val[4], val[5], 0.0f, 0.0f, 0.0f);
#endif //#ifdef USE_MAG

#else
  myFusion(val[0]*PI/180.0, val[1]*PI/180.0, val[2]*PI/180.0, val[3], val[4], val[5], val[6], val[7], val[8]);
#endif


  quatBuf[0] = q0;
  quatBuf[1] = q1;
  quatBuf[2] = q2;
  quatBuf[3] = q3;
}

void readAllSensors(uint8_t *GyroTempBuf, uint8_t *AccTempBuf, uint8_t *MagTempBuf) {
  L3GD20_Read(GyroTempBuf, L3GD20_OUT_X_L_ADDR, 6);
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, AccTempBuf, 6);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, MagTempBuf, 6);
}

void readRawGyro(uint8_t *GyroTempBuf) {
  L3GD20_Read(GyroTempBuf, L3GD20_OUT_X_L_ADDR, 6);
}

void AHRS_GetValues(float * val) {
  STM_EVAL_LEDOn(LED8);
#if defined OUT_GYRO || defined OUT_ACCEL || defined OUT_MAG
  uint8_t USART_TempBuf[100];
  uint8_t byteCounter = 0;
#endif //#if defined OUT_GYRO || defined OUT_ACCEL || defined OUT_MAG


  processGyroData(GyroBuffer, GyroTempBuffer);
  processAccelData(AccBuffer, AccTempBuffer);
  processMagnetoData(MagBuffer, MagTempBuffer);



  val[0] = - (GyroBuffer[1] - GyroCorrectionCoeffs[1]);
  val[1] = GyroBuffer[0] - GyroCorrectionCoeffs[0];
  val[2] = GyroBuffer[2] - GyroCorrectionCoeffs[2];
#ifdef OUT_GYRO
  byteCounter += sprintf((char*)(USART_TempBuf + byteCounter), "%f,%f,%f,", val[0], val[1], val[2]);
  //USART_printfWithDMA("%f,%f,%f,", val[0], val[1], val[2]);
#endif //#ifdef OUT_GYRO

  val[3] = AccBuffer[0];
  val[4] = AccBuffer[1];
  val[5] = AccBuffer[2];
#ifdef OUT_ACCEL
  byteCounter += sprintf((char*)(USART_TempBuf + byteCounter), "%f,%f,%f,", val[3], val[4], val[5]);
  //USART_printfWithDMA("%f,%f,%f,", val[3], val[4], val[5]);
#endif //#ifdef OUT_ACCEL

  val[6] = MagBuffer[0];
  val[7] = MagBuffer[1];
  val[8] = MagBuffer[2];
#ifdef OUT_MAG
  byteCounter += sprintf((char*)(USART_TempBuf + byteCounter), "%f,%f,%f,", val[6], val[7], val[8]);
  //USART_printfWithDMA("%f,%f,%f,", val[6], val[7], val[8]);
#endif //#ifdef OUT_MAG

#if defined OUT_GYRO || defined OUT_ACCEL || defined OUT_MAG
  sprintf((char*)(USART_TempBuf + byteCounter), "\r\n");
  USART_printfWithDMA("%s", USART_TempBuf);
  //USART_printfWithDMA("\r\n");
#endif
  STM_EVAL_LEDOff(LED8);

}

void UpdateGyroBias() {
  Delay_1ms(1000);
  int i = 0;
  for (i = 0; i < 100; i++) {
      readRawGyro(GyroTempBuffer);
      processGyroData(GyroBuffer, GyroTempBuffer);
      GyroCorrectionCoeffs[0] += GyroBuffer[0];
      GyroCorrectionCoeffs[1] += GyroBuffer[1];
      GyroCorrectionCoeffs[2] += GyroBuffer[2];
      Delay_1ms(10);
  }
  GyroCorrectionCoeffs[0] /= 100.0f;
  GyroCorrectionCoeffs[1] /= 100.0f;
  GyroCorrectionCoeffs[2] /= 100.0f;
}


/**
  * @brief  Configure the Mems to gyroscope application.
  * @param  None
  * @retval None
  */
void Demo_GyroConfig(void)
{
  L3GD20_InitTypeDef L3GD20_InitStructure;
  L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;

  /* Configure Mems L3GD20 */
  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_3;
  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Single;
  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_250;
  L3GD20_Init(&L3GD20_InitStructure);

  L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_REF_SIGNAL;
  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_5;
  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;

  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_DISABLE);

  L3GD20_INT2InterruptCmd(L3GD20_INT2INTERRUPT_ENABLE);

  uint8_t tmpreg = 0;
  L3GD20_Read(&tmpreg, L3GD20_CTRL_REG4_ADDR, 1);
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    GyroSensitivity=L3G_Sensitivity_250dps;
    break;

  case 0x10:
    GyroSensitivity=L3G_Sensitivity_500dps;
    break;

  case 0x20:
    GyroSensitivity=L3G_Sensitivity_2000dps;
    break;
  }
  Gyro_LBEFlag = tmpreg & 0x40;
}

/**
  * @brief  Configure the Mems to compass application.
  * @param  None
  * @retval None
  */
void Demo_CompassConfig(void)
{
  LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
  LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
  LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;

  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
  LSM303DLHC_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_220_HZ ;
  LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_1_3_GA;
  LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION; //DO NOT CHANGE - IT DOES ONLY ONE CONVERSIO AND STOPS
  LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);

   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE; // normal or low power mode
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_1344_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE; // all axes are enabled
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G; //2G, 4G, 8G, 16G
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous; // Single or Continous
  LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE; // High resolution enable
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);

  /* Fill the accelerometer LPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection = LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_64; // 8, 16, 32, 64
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);

  LSM303DLHC_AccFilterCmd(LSM303DLHC_HIGHPASSFILTER_DISABLE);

  uint8_t CTRLB = 0;
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);

  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }

  uint8_t ctrlx[2];
  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx, 2);

  if(ctrlx[1]&0x40)
  {
    Accel_cDivider=64;
    LSM_Acc_Sensitivity = 0.25;
  }
  else
  {
    Accel_cDivider=16;
    /* normal mode */
    /* switch the sensitivity value set in the CRTL4*/
    switch(ctrlx[0] & 0x30)
    {
    case LSM303DLHC_FULLSCALE_2G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
      break;
    case LSM303DLHC_FULLSCALE_4G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_4g;
      break;
    case LSM303DLHC_FULLSCALE_8G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_8g;
      break;
    case LSM303DLHC_FULLSCALE_16G:
      LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_16g;
      break;
    }
  }
  Accel_LBEorFIFOFlag = (ctrlx[0] & 0x40) || (ctrlx[1] & 0x40);
}

void processGyroData(float *GyroBuf, uint8_t *GyroTempBuf) {
  int16_t RawData[3] = {0};
  int i = 0;

  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(Gyro_LBEFlag))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)GyroTempBuf[2*i+1] << 8) + GyroTempBuf[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)GyroTempBuf[2*i] << 8) + GyroTempBuf[2*i+1]);
    }
  }


  /* divide by sensitivity */
  for(i=0; i<3; i++)
  {
    GyroBuf[i]=(float)RawData[i]/GyroSensitivity;
  }
}

void processAccelData(float *AccBuf, uint8_t *AccelTempBuf) {
  int16_t pnRawData[3];
  uint8_t i;


  /* check in the control register4 the data alignment*/
  if(!Accel_LBEorFIFOFlag) /* Little Endian Mode or FIFO mode */
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)AccelTempBuf[2*i+1] << 8) + AccelTempBuf[2*i])/Accel_cDivider;
    }
  }
  else /* Big Endian Mode */
  {
    for(i=0; i<3; i++)
      pnRawData[i]=((int16_t)((uint16_t)AccelTempBuf[2*i] << 8) + AccelTempBuf[2*i+1])/Accel_cDivider;
  }

  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    AccBuf[i]=(float)pnRawData[i]/LSM_Acc_Sensitivity;
  }
}

void processMagnetoData(float *MagBuf, uint8_t *MagnTempBuf) {
  MagBuf[0]=(float)((int16_t)(((uint16_t)MagnTempBuf[0] << 8) + MagnTempBuf[1]))/Magn_Sensitivity_XY;
  MagBuf[2]=(float)((int16_t)(((uint16_t)MagnTempBuf[2] << 8) + MagnTempBuf[3]))/Magn_Sensitivity_Z; // THIS IS Z! Check datasheet!
  MagBuf[1]=(float)((int16_t)(((uint16_t)MagnTempBuf[4] << 8) + MagnTempBuf[5]))/Magn_Sensitivity_XY; // THIS IS Y! Check datasheet!
}

void getEulerAngles(float *euler) {
  float *q = QuaternionsBuffer;

  /*euler[0] = atan2(2*q[1]*q[2]-2*q[0]*q[3], 2*q[0]*q[0]+2*q[1]*q[1]-1)*180/PI; // heading, yaw, phi
  euler[1] = -asin(2*q[1]*q[3]+2*q[0]*q[2])*180/PI; // attitude, elevation, pitch, theta
  euler[2] = atan2(2*q[2]*q[3]-2*q[0]*q[1], 2*q[0]*q[0]+2*q[3]*q[3]-1)*180/PI; // bank, roll, psi*/
  float test = QX*QY+QZ*QW;
  if (test > 0.499) {
    euler[0] = 2*atan2(QX, QW)*180/PI;
    euler[1] = PI*180/(2*PI);
    euler[2] = 0;
  } else if (test< -0.499) {
    euler[0] = -2*atan2(QX, QW)*180/PI;
    euler[1] = -PI*180/(2*PI);
    euler[2] = 0;
  } else {
    euler[0] = atan2(2*QY*QW - 2*QX*QZ, 1 - 2*QY*QY - 2*QZ*QZ)*180/PI;
    euler[1] = asin(2*QX*QY + 2*QZ*QW)*180/PI;
    euler[2] = atan2(2*QX*QW - 2*QY*QZ, 1 - 2*QX*QX - 2*QZ*QZ)*180/PI;
  }
}


void getEulerAsArray(uint8_t *eulerArr) {
  int i = 0;
  int16_t elrs = 0;
  // Axis X - third Euler
  elrs = (int16_t)(euler[0]);
  eulerArr[0] = (uint8_t)elrs;
  eulerArr[1] = (uint8_t)(elrs>>8);
  // Axis Y - second Euler
  elrs = (int16_t)(euler[1]);
  eulerArr[2] = (uint8_t)elrs;
  eulerArr[3] = (uint8_t)(elrs>>8);
  // Axis Z - first Euler
  elrs = (int16_t)(euler[2]);
  eulerArr[4] = (uint8_t)elrs;
  eulerArr[5] = (uint8_t)(elrs>>8);
}

/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
{
  return 0;
}

/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  return 0;
}