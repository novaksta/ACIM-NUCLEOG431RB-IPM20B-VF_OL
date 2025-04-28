/**
  ******************************************************************************
  * @file    mc_math.c
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
  */
/* Includes ------------------------------------------------------------------*/
#include "mc_math.h"
#include "mc_type.h"
#include "math.h"
/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MC_Math Motor Control Math functions
  * @brief Motor Control Mathematic functions of the Motor Control SDK
  *
  * @todo Document the Motor Control Math "module".
  *
  * @{
  */

/* Private macro -------------------------------------------------------------*/
/* 256 elelemnts Sin wave look-up table [0-90 degrees] in float format*/
#define SIN_FLOAT_0_90_TABLE {\
0.000000,0.006133,0.012265,0.018397,0.024529,0.030659,0.036789,0.042917,\
0.049043,0.055167,0.061290,0.067410,0.073527,0.079642,0.085754,0.091862,\
0.097968,0.104069,0.110167,0.116260,0.122349,0.128433,0.134513,0.140587,\
0.146657,0.152720,0.158778,0.164830,0.170876,0.176915,0.182948,0.188974,\
0.194993,0.201004,0.207008,0.213004,0.218992,0.224972,0.230943,0.236906,\
0.242859,0.248804,0.254739,0.260665,0.266581,0.272487,0.278382,0.284267,\
0.290142,0.296005,0.301858,0.307699,0.313528,0.319346,0.325151,0.330945,\
0.336726,0.342494,0.34825,0.353992,0.3597210,0.365436,0.371138,0.376826,\
0.382499,0.388159,0.393803,0.399433,0.405048,0.410647,0.416232,0.421800,\
0.427353,0.432889,0.438409,0.443913,0.449400,0.454870,0.460323,0.465759,\
0.471177,0.476578,0.481960,0.487325,0.492671,0.497998,0.503307,0.508597,\
0.513868,0.519119,0.524351,0.529564,0.534756,0.539928,0.545080,0.550211,\
0.555322,0.560412,0.565480,0.570528,0.575554,0.580558,0.585541,0.590501,\
0.595439,0.600355,0.605249,0.610119,0.614967,0.619791,0.624592,0.629370,\
0.634124,0.638854,0.643560,0.648242,0.652900,0.657533,0.662141,0.666724,\
0.671282,0.675815,0.680323,0.684805,0.689261,0.693691,0.698096,0.702474,\
0.706825,0.711150,0.715448,0.719720,0.723964,0.728181,0.732371,0.736533,\
0.740667,0.744773,0.748852,0.752902,0.756924,0.760918,0.764883,0.768819,\
0.772726,0.776604,0.780453,0.784273,0.788063,0.791824,0.795554,0.799255,\
0.802926,0.806566,0.810176,0.813756,0.817305,0.820824,0.824311,0.827768,\
0.831193,0.834587,0.837950,0.841281,0.844581,0.847848,0.851084,0.854288,\
0.857460,0.860599,0.863706,0.866781,0.869823,0.872832,0.875809,0.878753,\
0.881663,0.884540,0.887384,0.890195,0.892972,0.895716,0.898426,0.901102,\
0.903744,0.906353,0.908927,0.911467,0.913973,0.916444,0.918881,0.921283,\
0.923651,0.925984,0.928282,0.930545,0.932773,0.934966,0.937124,0.939247,\
0.941334,0.943386,0.945403,0.947384,0.949329,0.951239,0.953112,0.954950,\
0.956752,0.958518,0.960248,0.961942,0.963600,0.965221,0.966807,0.968355,\
0.969868,0.971344,0.972783,0.974186,0.975552,0.976881,0.978174,0.979430,\
0.980649,0.981831,0.982977,0.984085,0.985156,0.986190,0.987188,0.988148,\
0.989070,0.989956,0.990804,0.991616,0.992389,0.993126,0.993825,0.994487,\
0.995111,0.995698,0.996248,0.996760,0.997234,0.997671,0.998071,0.998433,\
0.998757,0.999044,0.999293,0.999505,0.999679,0.999816,0.999915,0.999976}

#define divSQRT_3 (int32_t)0x49E6    /* 1/sqrt(3) in q1.15 format=0.5773315*/

/* Private variables ---------------------------------------------------------*/
const float fSin_Table[256] = SIN_FLOAT_0_90_TABLE;

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function transforms stator values a and b (which are
  *         directed along axes each displaced by 120 degrees) into values
  *         alpha and beta in a stationary qd reference frame.
  *                               alpha = a
  *                       beta = -(2*b+a)/sqrt(3)
  * @param  Input: stator values a and b in ab_t format
  * @retval Stator values alpha and beta in alphabeta_t format
  */
__weak alphabeta_t MCM_Clarke( ab_t Input  )
{
  alphabeta_t Output;

  int32_t a_divSQRT3_tmp, b_divSQRT3_tmp ;
  int32_t wbeta_tmp;
  int16_t hbeta_tmp;

  /* qIalpha = qIas*/
  Output.alpha = Input.a;

  a_divSQRT3_tmp = divSQRT_3 * ( int32_t )Input.a;

  b_divSQRT3_tmp = divSQRT_3 * ( int32_t )Input.b;

  /*qIbeta = -(2*qIbs+qIas)/sqrt(3)*/
#ifdef FULL_MISRA_C_COMPLIANCY
  wbeta_tmp = ( -( a_divSQRT3_tmp ) - ( b_divSQRT3_tmp ) -
                 ( b_divSQRT3_tmp ) ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */

  wbeta_tmp = ( -( a_divSQRT3_tmp ) - ( b_divSQRT3_tmp ) -
                 ( b_divSQRT3_tmp ) ) >> 15;
#endif

  /* Check saturation of Ibeta */
  if ( wbeta_tmp > INT16_MAX )
  {
    hbeta_tmp = INT16_MAX;
  }
  else if ( wbeta_tmp < ( -32768 ) )
  {
    hbeta_tmp = ( -32768 );
  }
  else
  {
    hbeta_tmp = ( int16_t )( wbeta_tmp );
  }

  Output.beta = hbeta_tmp;

  if ( Output.beta == ( int16_t )( -32768 ) )
  {
    Output.beta = -32767;
  }

  return ( Output );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function transforms stator values alpha and beta, which
  *         belong to a stationary qd reference frame, to a rotor flux
  *         synchronous reference frame (properly oriented), so as q and d.
  *                   d= alpha *sin(theta)+ beta *cos(Theta)
  *                   q= alpha *cos(Theta)- beta *sin(Theta)
  * @param  Input: stator values alpha and beta in alphabeta_t format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator values q and d in qd_t format
  */
__weak qd_t MCM_Park( alphabeta_t Input, int16_t Theta )
{
  qd_t Output;
  int32_t d_tmp_1, d_tmp_2, q_tmp_1, q_tmp_2;
  Trig_Components Local_Vector_Components;
  int32_t wqd_tmp;
  int16_t hqd_tmp;

  Local_Vector_Components = MCM_Trig_Functions( Theta );

  /*No overflow guaranteed*/
  q_tmp_1 = Input.alpha * ( int32_t )Local_Vector_Components.hCos;

  /*No overflow guaranteed*/
  q_tmp_2 = Input.beta * ( int32_t )Local_Vector_Components.hSin;

  /*Iq component in Q1.15 Format */
#ifdef FULL_MISRA_C_COMPLIANCY
  wqd_tmp = ( q_tmp_1 - q_tmp_2 ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  wqd_tmp = ( q_tmp_1 - q_tmp_2 ) >> 15;
#endif

  /* Check saturation of Iq */
  if ( wqd_tmp > INT16_MAX )
  {
    hqd_tmp = INT16_MAX;
  }
  else if ( wqd_tmp < ( -32768 ) )
  {
    hqd_tmp = ( -32768 );
  }
  else
  {
    hqd_tmp = ( int16_t )( wqd_tmp );
  }

  Output.q = hqd_tmp;

  if ( Output.q == ( int16_t )( -32768 ) )
  {
    Output.q = -32767;
  }

  /*No overflow guaranteed*/
  d_tmp_1 = Input.alpha * ( int32_t )Local_Vector_Components.hSin;

  /*No overflow guaranteed*/
  d_tmp_2 = Input.beta * ( int32_t )Local_Vector_Components.hCos;

  /*Id component in Q1.15 Format */
#ifdef FULL_MISRA_C_COMPLIANCY
  wqd_tmp = ( d_tmp_1 + d_tmp_2 ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  wqd_tmp = ( d_tmp_1 + d_tmp_2 ) >> 15;
#endif

  /* Check saturation of Id */
  if ( wqd_tmp > INT16_MAX )
  {
    hqd_tmp = INT16_MAX;
  }
  else if ( wqd_tmp < ( -32768 ) )
  {
    hqd_tmp = ( -32768 );
  }
  else
  {
    hqd_tmp = ( int16_t )( wqd_tmp );
  }

  Output.d = hqd_tmp;

  if ( Output.d == ( int16_t )( -32768 ) )
  {
    Output.d = -32767;
  }

  return ( Output );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function transforms stator voltage qVq and qVd, that belong to
  *         a rotor flux synchronous rotating frame, to a stationary reference
  *         frame, so as to obtain qValpha and qVbeta:
  *                  Valfa= Vq*Cos(theta)+ Vd*Sin(theta)
  *                  Vbeta=-Vq*Sin(theta)+ Vd*Cos(theta)
  * @param  Input: stator voltage Vq and Vd in qd_t format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator voltage Valpha and Vbeta in qd_t format
  */
__weak alphabeta_t MCM_Rev_Park( qd_t Input, int16_t Theta )
{
  int32_t alpha_tmp1, alpha_tmp2, beta_tmp1, beta_tmp2;
  Trig_Components Local_Vector_Components;
  alphabeta_t Output;

  Local_Vector_Components = MCM_Trig_Functions( Theta );

  /*No overflow guaranteed*/
  alpha_tmp1 = Input.q * ( int32_t )Local_Vector_Components.hCos;
  alpha_tmp2 = Input.d * ( int32_t )Local_Vector_Components.hSin;

#ifdef FULL_MISRA_C_COMPLIANCY
  Output.alpha = ( int16_t )( ( ( alpha_tmp1 ) + ( alpha_tmp2 ) ) / 32768 );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  Output.alpha = ( int16_t )( ( ( alpha_tmp1 ) + ( alpha_tmp2 ) ) >> 15 );
#endif

  beta_tmp1 = Input.q * ( int32_t )Local_Vector_Components.hSin;
  beta_tmp2 = Input.d * ( int32_t )Local_Vector_Components.hCos;

#ifdef FULL_MISRA_C_COMPLIANCY
  Output.beta = ( int16_t )( ( beta_tmp2 - beta_tmp1 ) / 32768 );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
  the compiler to perform the shift (instead of LSR logical shift right) */
  Output.beta = ( int16_t )( ( beta_tmp2 - beta_tmp1 ) >> 15 );
#endif

  return ( Output );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function returns cosine and sine functions of the angle fed in
  *         input
  * @param  hAngle: angle in q1.15 format
  * @retval Sin(angle) and Cos(angle) in Trig_Components format
  */

__weak Trig_Components MCM_Trig_Functions( int16_t hAngle )
{

 union u32toi16x2 {
    uint32_t CordicRdata;
    Trig_Components Components;
  } CosSin;

  /* Configure CORDIC */
  WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_COSINE);
  LL_CORDIC_WriteData(CORDIC, 0x7FFF0000 + (uint32_t) hAngle);
  /* Read angle */
  CosSin.CordicRdata = LL_CORDIC_ReadData(CORDIC);
  return (CosSin.Components);

}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  It calculates the square root of a non-negative int32_t. It returns 0
  *         for negative int32_t.
  * @param  Input int32_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
__weak int32_t MCM_Sqrt( int32_t wInput )
{
  int32_t wtemprootnew;

  if ( wInput > 0 )
  {

    /* disable Irq as sqrt is used in MF and HF task */
    __disable_irq();
    /* Configure CORDIC */
    WRITE_REG(CORDIC->CSR, CORDIC_CONFIG_SQRT);
    LL_CORDIC_WriteData(CORDIC, (uint32_t) (wInput));
    /* Read sqrt and return */
    wtemprootnew = ((int32_t) (LL_CORDIC_ReadData(CORDIC))>>15);
	__enable_irq();

  }
  else
  {
    wtemprootnew = ( int32_t )0;
  }

  return ( wtemprootnew );
}

/**
  * @brief  It executes CORDIC algorithm for rotor position extraction from B-emf
  *         alpha and beta
  * @param  wBemf_alfa_est estimated Bemf alpha on the stator reference frame
  *         wBemf_beta_est estimated Bemf beta on the stator reference frame
  * @retval int16_t rotor electrical angle (s16degrees)
  */
inline int16_t MCM_PhaseComputation( int32_t wBemf_alfa_est, int32_t wBemf_beta_est )
{

  /* Configure and call to CORDIC */
  WRITE_REG(CORDIC->CSR,CORDIC_CONFIG_PHASE);
  LL_CORDIC_WriteData(CORDIC, (uint32_t) wBemf_alfa_est);
  LL_CORDIC_WriteData(CORDIC, (uint32_t) wBemf_beta_est);

  /* Read computed angle */
  return (int16_t)(LL_CORDIC_ReadData(CORDIC)>>16);

}

/**
  * @brief  This function codify a floating point number into the relative
  *         32bit integer.
  * @param  float Floating point number to be coded.
  * @retval uint32_t Coded 32bit integer.
  */
__weak uint32_t MCM_floatToIntBit( float x )
{
  uint32_t * pInt;
  pInt = ( uint32_t * )( &x );
  return *pInt;
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
  * @brief  This function returns sine and cosine functions of the angle fed in
  *         input
  * @param  fAngle: angle in float format [0�, 360�]
  * @retval Sin(angle) and Cos(angle) in fTrig_Components format
  */
fTrig_Components MCM_floatTrig_Functions(float fAngleDeg)
{
  fTrig_Components fSinCos;
  volatile uint32_t SinIndex,CosIndex;
  float fAngleTmp;
  uint8_t bSector=0;

  /* Correction for erroneous angle convention */
  if(fAngleDeg>=360.0f)
  {
   fAngleDeg -= 360.0f;
  }
  else if(fAngleDeg<=0.0f)
  {
    fAngleDeg +=360.0f;
  }

  fAngleTmp =  fAngleDeg;

  if((fAngleTmp>= 0.0f) && (fAngleTmp<=(90.0f)))
  { /* Between 0-90 degrees */
    bSector = 1;
  }
  else if((fAngleTmp> (90.0f)) && (fAngleTmp<=180.0f))
  {/* Between 90-180 degrees */
    bSector = 2;
  }
  else if((fAngleTmp> 180.0f) && (fAngleTmp<=(270.0f)))
  {/* Between 180-270 degrees */
    bSector = 3;
  }
    else if((fAngleTmp> (270.0f)) && (fAngleTmp<=(360.0f)))
  {/* Between 270-360 degrees */
    bSector = 4;
  }

  /*NOTE.
      Sin(Theta) = Cos (90�-Theta)
      Cos(Theta) = Sin (90�-Theta)
  */

  /* Index computation */
  switch(bSector)
  {
  case 1:
    {
      SinIndex =  (uint32_t)((fAngleTmp*1024.0f)/360.0f);
      if(SinIndex>255)
      {
       SinIndex = 255;
      }
      fSinCos.fSin = fSin_Table[(uint8_t)SinIndex];

      CosIndex =  (uint32_t)(((90.0f-fAngleTmp)*1024.0f)/360.0f);

      if(CosIndex>255)
      {
        CosIndex = 255;
      }
      fSinCos.fCos = +fSin_Table[(uint8_t)CosIndex];

    }
    break;
  case 2:
    {
      SinIndex =  (uint32_t)(((180.0f - fAngleTmp)*1024.0f)/360.0f);
      if(SinIndex>255)
      {
        SinIndex = 255;
      }
      fSinCos.fSin = fSin_Table[(uint8_t)SinIndex];

      CosIndex =  (uint32_t)(((90.0f- (180.0f - fAngleTmp))*1024.0f)/360.0f);

      if(CosIndex>255)
      {
        CosIndex = 255;
      }
      fSinCos.fCos = -fSin_Table[(uint8_t)CosIndex];

    }
    break;
  case 3:
    {
      SinIndex =  (uint32_t)(((fAngleTmp-180.0f)*1024.0f)/360.0f);
      if(SinIndex>255)
      {
       SinIndex = 255;
      }
      fSinCos.fSin = -fSin_Table[(uint8_t)SinIndex];

      CosIndex =  (uint32_t)(((90.0f -(fAngleTmp-180.0f))*1024.0f)/360.0f);

      if(CosIndex>255)
      {
        CosIndex = 255;
      }
      fSinCos.fCos = -fSin_Table[(uint8_t)CosIndex];

    }
    break;
  case 4:
    {
      SinIndex =  (uint32_t)(((360.0f - fAngleTmp)*1024.0f)/360.0f);
      if(SinIndex>255)
      {
       SinIndex = 255;
      }
      fSinCos.fSin = -fSin_Table[(uint8_t)SinIndex];

      CosIndex =  (uint32_t)(((90.0f -(360.0f - fAngleTmp))*1024.0f)/360.0f);

      if(CosIndex>255)
      {
        CosIndex = 255;
      }
      fSinCos.fCos = +fSin_Table[(uint8_t)CosIndex];
    }
    break;
  }
  return (fSinCos);
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
 * @brief  This function transforms an Alpha Beta vector, which
  *         belong to a stationary qd reference frame , to a rotor flux
  *         synchronous reference frame (properly oriented), so as Vector_q and Vector_d.
  *                  Vector_q =  Vector_alpha *Cos(theta) - Vector_beta*Sin(theta)
  *                  Vector_d =  Vector_alpha *Sin(theta) + Vector_beta*Cos(theta)
  * @param  Vector_Input: Alpha Beta vector in Vector_s16_Components format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval qd Vector expressed in Vector_s16_Components format
  */
Vector_s16_Components MCM_Park_Generic(Vector_s16_Components Vector_Input, int16_t Theta)
{
  Vector_s16_Components Vector_Output;

  /* qd axes temp Vector components*/
  int32_t wVec_q_tmp1,wVec_q_tmp2,wVec_d_tmp1,wVec_d_tmp2;

  Trig_Components Local_Vector_Components;

  int32_t wVector_qd_tmp;
  int16_t hVector_qd_tmp;

  Local_Vector_Components = MCM_Trig_Functions(Theta);

  /*No overflow guaranteed*/
  wVec_q_tmp1 = Vector_Input.qVec_Component1 * (int32_t)Local_Vector_Components.hCos;
  wVec_q_tmp2 = Vector_Input.qVec_Component2 * (int32_t)Local_Vector_Components.hSin;

#ifdef FULL_MISRA_C_COMPLIANCY
  wVector_qd_tmp = (int16_t)(((wVec_q_tmp1)-(wVec_q_tmp2))/32768);
#else
/* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
  the compiler to perform the shift (instead of LSR logical shift right) */
  wVector_qd_tmp = (int16_t)(((wVec_q_tmp1)-(wVec_q_tmp2))>>15);
#endif

  /* Check saturation of Vq */
  if (wVector_qd_tmp > INT16_MAX)
  {
    hVector_qd_tmp = INT16_MAX;
  }
  else if (wVector_qd_tmp < (-32768))
  {
    hVector_qd_tmp = (-32768);
  }
  else
  {
    hVector_qd_tmp = (int16_t)(wVector_qd_tmp);
  }

  Vector_Output.qVec_Component1 = hVector_qd_tmp;

  if (Vector_Output.qVec_Component1 == (int16_t)(-32768))
  {
    Vector_Output.qVec_Component1 = -32767;
  }

  /*No overflow guaranteed*/
  wVec_d_tmp1 = Vector_Input.qVec_Component1 * (int32_t)Local_Vector_Components.hSin;

  /*No overflow guaranteed*/
  wVec_d_tmp2 = Vector_Input.qVec_Component2 * (int32_t)Local_Vector_Components.hCos;

#ifdef FULL_MISRA_C_COMPLIANCY
  wVector_qd_tmp = (int16_t)((wVec_d_tmp1+wVec_d_tmp2)/32768);
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
  the compiler to perform the shift (instead of LSR logical shift right) */
  wVector_qd_tmp = (int16_t)((wVec_d_tmp1+wVec_d_tmp2) >>15);
#endif

  /* Check saturation of Vd */
  if (wVector_qd_tmp > INT16_MAX)
  {
    hVector_qd_tmp = INT16_MAX;
  }
  else if (wVector_qd_tmp < (-32768))
  {
    hVector_qd_tmp = (-32768);
  }
  else
  {
    hVector_qd_tmp = (int16_t)(wVector_qd_tmp);
  }

  Vector_Output.qVec_Component2 = hVector_qd_tmp;

  if (Vector_Output.qVec_Component2 == (int16_t)(-32768))
  {
    Vector_Output.qVec_Component2 = -32767;
  }

  return(Vector_Output);
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
  * @brief  This function transforms a qd_Vector, that belong to
  *         a rotor flux synchronous rotating frame, to a stationary reference
  *         frame, so as to obtain an Alpha Beta Vector:
  *                  Valpha=  Vq*Cos(theta)+ Vd*Sin(theta)
  *                  Vbeta = -Vq*Sin(theta)+ Vd*Cos(theta)
  * @param  Vector_Input: qd_Vecor expressed in Vector_s16_Components format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Alpha Beta Vector expressed in Vector_s16_Components format
  */
Vector_s16_Components MCM_Rev_Park_Generic(Vector_s16_Components Vector_Input, int16_t Theta)
{

  Vector_s16_Components Vector_Output;

  /* Alpha Beta axes temp Vector components*/
  int32_t wVec_alpha_tmp1,wVec_alpha_tmp2,wVec_beta_tmp1,wVec_beta_tmp2;

  Trig_Components Local_Vector_Components;

  Local_Vector_Components = MCM_Trig_Functions(Theta);

  /*No overflow guaranteed*/
  wVec_alpha_tmp1 = Vector_Input.qVec_Component1 * (int32_t)Local_Vector_Components.hCos;
  wVec_alpha_tmp2 = Vector_Input.qVec_Component2 * (int32_t)Local_Vector_Components.hSin;

#ifdef FULL_MISRA_C_COMPLIANCY
  Vector_Output.qVec_Component1 = (int16_t)(((wVec_alpha_tmp1)+(wVec_alpha_tmp2))/32768);
#else
/* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
  the compiler to perform the shift (instead of LSR logical shift right) */
  Vector_Output.qVec_Component1 = (int16_t)(((wVec_alpha_tmp1)+(wVec_alpha_tmp2))>>15);
#endif

  wVec_beta_tmp1 = Vector_Input.qVec_Component1 * (int32_t)Local_Vector_Components.hSin;
  wVec_beta_tmp2 = Vector_Input.qVec_Component2 * (int32_t)Local_Vector_Components.hCos;

#ifdef FULL_MISRA_C_COMPLIANCY
  Vector_Output.qVec_Component2 = (int16_t)((wVec_beta_tmp2-wVec_beta_tmp1)/32768);
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
  the compiler to perform the shift (instead of LSR logical shift right) */
  Vector_Output.qVec_Component2 = (int16_t)((wVec_beta_tmp2-wVec_beta_tmp1) >>15);
#endif

  return(Vector_Output);
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
