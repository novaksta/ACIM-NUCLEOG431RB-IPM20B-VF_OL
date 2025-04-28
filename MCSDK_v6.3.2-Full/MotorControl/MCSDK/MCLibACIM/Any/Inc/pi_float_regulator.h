/**
  ******************************************************************************
  * @file    pi_float_regulator.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          floating point version of the PI regulator component of the Motor 
  *          Control SDK.
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
  * @ingroup PIFloatRegulator
  */
#ifndef __PI_FLOAT_REGULATOR_H
#define __PI_FLOAT_REGULATOR_H


#include "mc_type.h"


typedef struct
{
  
  float   fDefKpGain;           /**< Default @f$K_{pg}@f$ gain */
  float   fDefKiGain;           /**< Default @f$K_{ig}@f$ gain */
  
  float fKpGain;
  float fKiGain;
  float fIntegralTerm;

  
  float fUpperIntegralLimit;
  float fLowerIntegralLimit;
  
  float fLowerLimit;
  float fUpperLimit; 
  
  float fKs;
  float fAntiWindTerm;
  
  FunctionalState bAntiWindUpActivation;
  float fExecFrequencyHz;
  
} PI_Float_Handle_t;

//typedef struct
//{
//  float fKp;
//  float fKi;
//  float fKs;
//  FunctionalState AntiWindupEnable; 
//  float fLowerLimit;
//  float fUpperLimit;
//  float fExecFrequencyHz;
//}FloatPI_StructInit_t;


void PI_Float_HandleInit(PI_Float_Handle_t *pHandle);
float PI_Float_Calc(PI_Float_Handle_t *pHandle, float fProcessVarError);

#endif /* __PI_FLOAT_REGULATOR_H */

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
