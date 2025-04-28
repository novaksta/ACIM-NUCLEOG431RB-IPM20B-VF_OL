
/**
  ******************************************************************************
  * @file    mc_math.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control.
  *
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
  * @ingroup MC_Math
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MC_MATH_H
#define MC_MATH_H

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MC_Math
  * @{
  */
#define SQRT_2  1.4142
#define SQRT_3  1.732

/* CORDIC coprocessor configuration register settings */

/* CORDIC FUNCTION: PHASE q1.31 (Electrical Angle computation) */
#define CORDIC_CONFIG_PHASE     (LL_CORDIC_FUNCTION_PHASE | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 |\
				 LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_1 |\
				 LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

/* CORDIC FUNCTION: SQUAREROOT q1.31 */
#define CORDIC_CONFIG_SQRT      (LL_CORDIC_FUNCTION_SQUAREROOT | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_1 |\
				 LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 |\
				 LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

/* CORDIC FUNCTION: COSINE q1.15 */
#define CORDIC_CONFIG_COSINE    (LL_CORDIC_FUNCTION_COSINE | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 |\
         LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 |\
         LL_CORDIC_INSIZE_16BITS | LL_CORDIC_OUTSIZE_16BITS)
/* CORDIC FUNCTION: MODULUS q1.15 */
#define CORDIC_CONFIG_MODULUS   (LL_CORDIC_FUNCTION_MODULUS | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 |\
				 LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 |\
				 LL_CORDIC_INSIZE_16BITS | LL_CORDIC_OUTSIZE_16BITS)

/**
  * @brief  Macro to compute logarithm of two
  */
#define LOG2(x) \
  ((x) == 65535 ? 16 : \
   ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 15 : \
    ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2*2 ? 14 : \
     ((x) == 2*2*2*2*2*2*2*2*2*2*2*2*2 ? 13 : \
      ((x) == 2*2*2*2*2*2*2*2*2*2*2*2 ? 12 : \
       ((x) == 2*2*2*2*2*2*2*2*2*2*2 ? 11 : \
        ((x) == 2*2*2*2*2*2*2*2*2*2 ? 10 : \
         ((x) == 2*2*2*2*2*2*2*2*2 ? 9 : \
          ((x) == 2*2*2*2*2*2*2*2 ? 8 : \
           ((x) == 2*2*2*2*2*2*2 ? 7 : \
            ((x) == 2*2*2*2*2*2 ? 6 : \
             ((x) == 2*2*2*2*2 ? 5 : \
              ((x) == 2*2*2*2 ? 4 : \
               ((x) == 2*2*2 ? 3 : \
                ((x) == 2*2 ? 2 : \
                 ((x) == 2 ? 1 : \
                  ((x) == 1 ? 0 : -1)))))))))))))))))

/**
  * @brief  Trigonometrical functions type definition
  */
typedef struct
{
  int16_t hCos;
  int16_t hSin;
} Trig_Components;

/**
* @brief  Trigonometrical functions type definition
*/
typedef struct
{
  float fSin;
  float fCos;
} fTrig_Components;

/**
  * @brief  This function transforms stator currents Ia and qIb (which are
  *         directed along axes each displaced by 120 degrees) into currents
  *         Ialpha and Ibeta in a stationary qd reference frame.
  *                               Ialpha = Ia
  *                       Ibeta = -(2*Ib+Ia)/sqrt(3)
  * @param  Curr_Input: stator current Ia and Ib in ab_t format
  * @retval Stator current Ialpha and Ibeta in alphabeta_t format
  */
alphabeta_t MCM_Clarke( ab_t Input );

/**
  * @brief  This function transforms stator values alpha and beta, which
  *         belong to a stationary qd reference frame, to a rotor flux
  *         synchronous reference frame (properly oriented), so as Iq and Id.
  *                   Id= Ialpha *sin(theta)+qIbeta *cos(Theta)
  *                   Iq=qIalpha *cos(Theta)-qIbeta *sin(Theta)
  * @param  Curr_Input: stator values alpha and beta in alphabeta_t format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator current q and d in qd_t format
  */
qd_t MCM_Park( alphabeta_t Input, int16_t Theta );

/**
  * @brief  This function transforms stator voltage qVq and qVd, that belong to
  *         a rotor flux synchronous rotating frame, to a stationary reference
  *         frame, so as to obtain qValpha and qVbeta:
  *                  Valfa= Vq*Cos(theta)+ Vd*Sin(theta)
  *                  Vbeta=-Vq*Sin(theta)+ Vd*Cos(theta)
  * @param  Curr_Input: stator voltage Vq and Vd in qd_t format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator values alpha and beta in alphabeta_t format
  */
alphabeta_t MCM_Rev_Park( qd_t Input, int16_t Theta );

/**
  * @brief  This function returns cosine and sine functions of the angle fed in
  *         input
  * @param  hAngle: angle in q1.15 format
  * @retval Trig_Components Cos(angle) and Sin(angle) in Trig_Components format
  */
Trig_Components MCM_Trig_Functions( int16_t hAngle );

/**
  * @brief  It calculates the square root of a non-negative s32. It returns 0
  *         for negative s32.
  * @param  Input int32_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
int32_t MCM_Sqrt( int32_t wInput );

int16_t MCM_PhaseComputation( int32_t wBemf_alfa_est, int32_t wBemf_beta_est );

/**
  * @brief  This function codify a floting point number into the relative
  *         32bit integer.
  * @param  float Floting point number to be coded.
  * @retval uint32_t Coded 32bit integer.
  */
uint32_t MCM_floatToIntBit( float x );

/**
  * @brief  This function returns sine and cosine functions of the angle fed in
  *         input
  * @param  fAngle: angle in float format [0�, 360�]
  * @retval Sin(angle) and Cos(angle) in fTrig_Components format
  */
fTrig_Components MCM_floatTrig_Functions(float fAngleDeg);

/**
 * @brief  This function transforms an Alpha Beta vector, which
  *         belong to a stationary qd reference frame , to a rotor flux
  *         synchronous reference frame (properly oriented), so as Vector_q and Vector_d.
  *                  Vector_q =  Vector_alpha *Cos(theta) - Vector_beta*Sin(theta)
  *                  Vector_d =  Vector_alpha *Sin(theta) + Vector_beta*Cos(theta)
  * @param  Vector_Input: Alpha Beta vector in Vector_s16_Components format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval qd Vector expressed in Vector_s16_Components format
  */
Vector_s16_Components MCM_Park_Generic(Vector_s16_Components Vector_Input, int16_t Theta);

/**
  * @brief  This function transforms a qd_Vector, that belong to
  *         a rotor flux synchronous rotating frame, to a stationary reference
  *         frame, so as to obtain an Alpha Beta Vector:
  *                  Valpha=  Vq*Cos(theta)+ Vd*Sin(theta)
  *                  Vbeta = -Vq*Sin(theta)+ Vd*Cos(theta)
  * @param  Vector_Input: qd_Vecor expressed in Vector_s16_Components format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Alpha Beta Vector expressed in Vector_s16_Components format
  */
Vector_s16_Components MCM_Rev_Park_Generic(Vector_s16_Components Vector_Input, int16_t Theta);

/**
  * @brief  It executes Modulus algorithm
  * @param  alpha component
  *         beta component
  * @retval int16_t Modulus
  */
static inline int16_t MCM_Modulus( int16_t alpha, int16_t beta )
{
  int16_t Val;
   __disable_irq();
   /* Configure and call to CORDIC- */
   WRITE_REG(CORDIC->CSR,CORDIC_CONFIG_MODULUS);
   LL_CORDIC_WriteData(CORDIC, (int32_t) (beta)<<16 | alpha);
   /* Wait for result */
   while(!LL_CORDIC_IsActiveFlag_RRDY( CORDIC ))
   {
   }
  /* Read computed modulus */
  Val = (int16_t)(LL_CORDIC_ReadData(CORDIC)&0xFFFF);
  __enable_irq();
  return Val;

}

/**
  * @}
  */

/**
  * @}
  */
#endif /* MC_MATH_H*/
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
