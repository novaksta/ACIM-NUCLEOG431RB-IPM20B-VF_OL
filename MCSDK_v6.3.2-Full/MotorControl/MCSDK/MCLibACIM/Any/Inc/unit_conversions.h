/**
  ******************************************************************************
  * @file    unit_conversions.h
  * @author  STMicroelectronics - SRA - System Development Unit - MC Team
  * @brief   This file contains definitions and functions prototypes 
  *          related to unit conversion in the Motor Control SDK.
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UNIT_CONVERSIONS_H
#define UNIT_CONVERSIONS_H


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


/* Includes --------------------------------------------------*/   
#include "mc_type.h"  
#include "parameters_conversion.h"  
#include "bus_voltage_sensor.h"   
/* Exported type definition --------------------------------------------------*/
   
   
/* Functions prototype -------------------------------------------------------*/   
Signal_Components Convert_s16_to_A(qd_t hIqd);
Signal_Components Convert_s16_to_V(BusVoltageSensor_Handle_t *pBVS, qd_t hVqd);

qd_t Convert_V_to_s16(BusVoltageSensor_Handle_t *pBVS, Signal_Components fVqdV);
qd_t Convert_A_to_s16(Signal_Components fIqdA);

int16_t Convert_Rad_to16bit(float fAngle_rad);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif


/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
