/**
  ******************************************************************************
  * @file    acim_ifoc.c
  * @author  STMicroelectronics - SRA - System Development Unit - MC Team
  * @brief   This file provides firmware functions that implement the  features
  *          of the Rotor Flux Position Feedback component of the Motor Control SDK.
  *           - estimates the rotor flux electrical angle
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
#include "acim_ifoc.h"
#include "math.h"
#include "arm_math.h"

/**
  * @brief  It initializes the variables required for the rotor flux angle estimation.
  * @param  pHandle: handler of the current instance of the ACIM_IFOC component
  * @retval none
  */
  
void ACIM_IFOC_Init(ACIM_IFOC_Handle_t *pHandle , SpeednPosFdbk_Handle_t * SPD_Handle)
{
 pHandle->SPD = SPD_Handle;
  
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
  * @brief  It performs the rotor flux angle estimation.
  * @param  pHandle: handler of the current instance of the ACIM_IFOC component
  * @retval none
  */
void ACIM_IFOC_CalcAngle(ACIM_IFOC_Handle_t *pHandle)
{
  float fSlipFreq_rads = 0.0f;
  float fRotorFluxFreq_rads = 0.0f;
  float fRotorElSpeed_meas_rads = 0.0f;
  
  /* Conversion factor for conversion 01Hz to rad/s */
  float SpeedConvFactor = (2.0f*(float)PI)/10.0f;
  
  /* Conversion of measured current from s16A to Ampere unit */
  Signal_Components  fIqdA;
  
  fIqdA  = Convert_s16_to_A(pHandle->pFOCVars[M1].Iqdref);
  
  float fids_A_tmp = fIqdA.fS_Component2;
  
  /* It avoids division by zero*/
  if(fids_A_tmp == 0.0f)
  {
    fids_A_tmp = pHandle->pACIM_MotorParams->fImagn_A;
  }
  
  /* Slip Frequency estimation (rad/s) - Iq/(Id*taur) */
  fSlipFreq_rads = fIqdA.fS_Component1/(fids_A_tmp*pHandle->pACIM_MotorParams->ftaur);
  
  /* Get and convert the measured meach. rotor speed from the speed/position sensor */  
  fRotorElSpeed_meas_rads = (float)SPD_GetAvrgMecSpeedUnit(pHandle->SPD)*SpeedConvFactor*(float)pHandle->pACIM_MotorParams->bPP*(10.0f/SPEED_UNIT);
    
  /* Rotor Flux Frequency calculation (rad/s) */
  fRotorFluxFreq_rads = fSlipFreq_rads + fRotorElSpeed_meas_rads;
  
  pHandle->fRotorFluxFreq_rads = fRotorFluxFreq_rads;
  
  /* Rotor Flux angle discrete integration */
  pHandle->fRotorFlux_Angle_rad += fRotorFluxFreq_rads * pHandle->fCalcAngleExecTime_s;
  
  /* Rotor Flux angle between [0, 2pi] */
  pHandle->fRotorFlux_Angle_rad = (float) fmod((double) pHandle->fRotorFlux_Angle_rad, (double)((float)(2.0f*(float)PI)));
   
  pHandle->hElAngle = Convert_Rad_to16bit(pHandle->fRotorFlux_Angle_rad);
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

/**/
/**
  * @brief  It returns the flux electrical angle in digital format
  * @param  pHandle: handler of the current instance of the ACIM_IFOC component
  * @retval none
  */
int16_t ACIM_IFOC_GetElAngle(ACIM_IFOC_Handle_t *pHandle)
{
 return pHandle->hElAngle;
}


/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
