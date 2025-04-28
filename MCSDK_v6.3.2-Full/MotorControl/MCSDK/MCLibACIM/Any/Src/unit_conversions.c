/**
  ******************************************************************************
  * @file    unit_conversions.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of units conversion of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "unit_conversions.h"
#include "arm_math.h"

#define RAD3DIV3        0.577350269f

#if (defined (CCMRAM) || defined(CCMRAM_ENABLED))
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#elif defined (__GNUC__)
__attribute__((section ("ccmram")))
#endif
#endif
int16_t Convert_Rad_to16bit(float fAngle_rad)
{
  int16_t hElAngle;
       
  hElAngle =  (int16_t) ((fAngle_rad*32767.0f)/(float)PI);
  
  return (hElAngle);
}


#if (defined (CCMRAM) || defined(CCMRAM_ENABLED))
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#elif defined (__GNUC__)
__attribute__((section ("ccmram")))
#endif
#endif
Signal_Components Convert_s16_to_A(qd_t hIqd)
{
  float fConvfactor = (float)ADC_REFERENCE_VOLTAGE/(65536.0f*(float)RSHUNT*(float)AMPLIFICATION_GAIN);
  
  Signal_Components  fIqdA;
 
  fIqdA.fS_Component1 = (float)hIqd.q*fConvfactor;
  fIqdA.fS_Component2 = (float)hIqd.d*fConvfactor;
  
  return fIqdA;
}


#if (defined (CCMRAM) || defined(CCMRAM_ENABLED))
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#elif defined (__GNUC__)
__attribute__((section ("ccmram")))
#endif
#endif
qd_t Convert_A_to_s16(Signal_Components fIqdA)
{
  float fConvfactor = (float)ADC_REFERENCE_VOLTAGE/(65536.0f*(float)RSHUNT*(float)AMPLIFICATION_GAIN);
  
  qd_t hIqd;
 
  hIqd.q  = (int16_t)(fIqdA.fS_Component1/fConvfactor);
  hIqd.d  = (int16_t)(fIqdA.fS_Component2/fConvfactor);
  
  return hIqd;
}

#if (defined (CCMRAM) || defined(CCMRAM_ENABLED))
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#elif defined (__GNUC__)
__attribute__((section ("ccmram")))
#endif
#endif
Signal_Components Convert_s16_to_V(BusVoltageSensor_Handle_t *pBVS, qd_t hVqd)
{
  float fConvfactor = ((float)RAD3DIV3* (float)VBS_GetAvBusVoltage_V(pBVS))/32767.0f;
  
  Signal_Components  fVqd_V;
 
  fVqd_V.fS_Component1 = (float)hVqd.q*fConvfactor;
  fVqd_V.fS_Component2 = (float)hVqd.d*fConvfactor;
  
  return fVqd_V;
}


#if (defined (CCMRAM) || defined(CCMRAM_ENABLED))
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM)
__attribute__((section ("ccmram")))
#elif defined (__GNUC__)
__attribute__((section ("ccmram")))
#endif
#endif
qd_t Convert_V_to_s16(BusVoltageSensor_Handle_t *pBVS, Signal_Components fVqd_V)
{
  float fConvfactor = 32767.0f/((float)RAD3DIV3* (float)VBS_GetAvBusVoltage_V(pBVS));
  
  qd_t hVqd;
  
  float ftempVq = (fVqd_V.fS_Component1*fConvfactor);
  float ftempVd = (fVqd_V.fS_Component2*fConvfactor);
  
  if(ftempVq >32767.0f)
  {
    ftempVq = 32767.0f;
  }
  
  if(ftempVd >32767.0f)
  {
    ftempVd = 32767.0f;
  }
    
  hVqd.q = (int16_t)ftempVq;    
  hVqd.d = (int16_t)ftempVd;  

  return hVqd;
}

















/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
