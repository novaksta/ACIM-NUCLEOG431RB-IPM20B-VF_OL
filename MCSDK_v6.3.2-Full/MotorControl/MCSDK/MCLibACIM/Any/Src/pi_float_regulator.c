/**
  ******************************************************************************
  * @file    pi_float_regulator.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides the implementation of the floating point version 
  *          of the PI regulator component of the Motor Control SDK.
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

/* Includes ------------------------------------------------------------------*/

#include "pi_float_regulator.h"



/** @addtogroup MCSDK
 * @{
 */

/**
 * @defgroup PIFloatRegulator PI Float Regulator
 * @brief PI Float regulator component of the Motor Control SDK
 *
 * The PI float regulator component implements the following control function:
 *
 * @f[
 * u(t) = K_{p} e(t) + K_{i} \int_0^t e(\tau) \,d\tau
 * @f]
 *
 *
 * @{
 */

/**
 * @brief  It initializes the handle
 * @param  pHandle: handler of the current instance of the PID component
 * @retval None
 */



void PI_Float_HandleInit(PI_Float_Handle_t *pHandle)
{
  pHandle->fKpGain = pHandle->fDefKpGain;
  pHandle->fKiGain = pHandle->fDefKiGain;
  
  pHandle->fIntegralTerm = 0.0f;
  pHandle->fAntiWindTerm = 0.0f;  
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
/**
  * @brief  This function computes the output of a PI regulator sum of its 
  *         proportional and integral terms
  * @param  Float_PIReg PI regulator object
  * @param  float Actual process variable error, intended as the reference 
  *         value minus the actual process variable value
  * @retval float PI output
  */
float PI_Float_Calc(PI_Float_Handle_t *pHandle, float fProcessVarError)
{
  float fProportional_Term, fOutput,fOutputSat;
  
    /* Proportional term computation*/
  fProportional_Term = pHandle->fKpGain * fProcessVarError;
  fOutput =   fProportional_Term;


  /* Integral term computation */
  if (pHandle->fKiGain == 0.0f)
  {
    pHandle->fIntegralTerm = 0.0f;
  }
  else
  { 
//    pHandle->fIntegralTerm+= ((pHandle->fKiGain *(fProcessVarError)) - (pHandle->fKiGain*pHandle->fAntiWindTerm));
    pHandle->fIntegralTerm+= ((pHandle->fKiGain *(fProcessVarError))/pHandle->fExecFrequencyHz);
    
    if(pHandle->fIntegralTerm > pHandle->fUpperIntegralLimit)
    {
      pHandle->fIntegralTerm =  pHandle->fUpperIntegralLimit;
    }
    else if(pHandle->fIntegralTerm < pHandle->fLowerIntegralLimit)
    {
      pHandle->fIntegralTerm =  pHandle->fLowerIntegralLimit;
    }
    
    fOutput += pHandle->fIntegralTerm;
  }
   
  fOutputSat = fOutput;
  
  /* Saturation */
  if (fOutput > pHandle->fUpperLimit)
  {
    fOutputSat = pHandle->fUpperLimit;   
  } 
  
  if (fOutput < pHandle->fLowerLimit)
  {
    fOutputSat = pHandle->fLowerLimit;
  }
  
  if(pHandle->bAntiWindUpActivation == ((FunctionalState)ENABLE))
  {
    pHandle->fAntiWindTerm = (fOutput - fOutputSat) * pHandle->fKs;
  }
  else
  {
    pHandle->fAntiWindTerm =0.0f;
  }
  
  fOutput = fOutputSat;
  
  return(fOutput); 	
}

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
