/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
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
#include "main.h"
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/
#define M1_CHARGE_BOOT_CAP_MS  10
#define M2_CHARGE_BOOT_CAP_MS  10
#define STOPPERMANENCY_MS      400
#define STOPPERMANENCY_MS2     400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * M1_CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * M2_CHARGE_BOOT_CAP_MS)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)

/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES (uint32_t)(0.000*(PWM_PERIOD_CYCLES/2))
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES (uint32_t)(0*(PWM_PERIOD_CYCLES2/2))
/* USER CODE END Private define */

#define VBUS_TEMP_ERR_MASK (MC_OVER_VOLT| MC_UNDER_VOLT| MC_OVER_TEMP)

/* Private variables----------------------------------------------------------*/
FOCVars_t FOCVars[NBR_OF_MOTORS];
/* It used by V/F control*/
PWM_Handle_t * pwmHandle[NBR_OF_MOTORS];
PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
DOUT_handle_t *pR_Brake[NBR_OF_MOTORS];
DOUT_handle_t *pOCPDisabling[NBR_OF_MOTORS];
ACIM_VF_Handle_t *pACIM_VF[NBR_OF_MOTORS];

static volatile uint16_t hMFTaskCounterM1 = 0; //cstat !MISRAC2012-Rule-8.9_a
static volatile uint16_t hStopPermanencyCounterM1 = 0;

uint8_t bMCBootCompleted = 0;

/* USER CODE BEGIN Private Variables */

/* USER CODE END Private Variables */

static volatile uint8_t bACIM_MagnValidCount = 0;

/* Private functions ---------------------------------------------------------*/
void TSK_MediumFrequencyTaskM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */

/* USER CODE END Private Functions */

/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  */
__weak void MCboot( MCI_Handle_t* pMCIList[NBR_OF_MOTORS] )
{
  /* USER CODE BEGIN MCboot 0 */

  /* USER CODE END MCboot 0 */

  bMCBootCompleted = 0;
  /**********************************************************/
  /*    PWM and current sensing component initialization    */
  /**********************************************************/

  /* If the V/f is used the pwmcHandle is used to perform the
   * regular conversions (Temp, Bus Voltage, and user conversions)
   * Better name for this component should be something like
   * PWM and Analog Conversions (PWMAC)
   * In this case the PWMC child component is is used only for Analog Conversions
   *
   */

  pwmcHandle[M1] = &PWM_Handle_M1._Super;
  R3_2_Init(&PWM_Handle_M1);
  ASPEP_start (&aspepOverUartA);

  /* USER CODE BEGIN MCboot 1 */

  /* USER CODE END MCboot 1 */

  /* It used by V/F control*/
  pwmHandle[M1] = &PWM_ParamsM1;
  PWM_Init(pwmHandle[M1]);

  /**************************************/
  /*    Start timers synchronously      */
  /**************************************/
  startTimers();

  /********************************************************/
  /*   Bus voltage sensor component initialization        */
  /********************************************************/
  (void)RCM_RegisterRegConv(&VbusRegConv_M1);
  RVBS_Init(&BusVoltageSensor_M1);

  /*************************************************/
  /*   Power measurement component initialization  */
  /*************************************************/
  pMPM[M1]->pVBS = &(BusVoltageSensor_M1._Super);

  /*******************************************************/
  /*   Temperature measurement component initialization  */
  /*******************************************************/
  (void)RCM_RegisterRegConv(&TempRegConv_M1);
  NTC_Init(&TempSensor_M1);

  MCI_Init(&Mci[M1], MC_NULL, &FOCVars[M1],pwmcHandle[M1] );
  MCI_ExecSpeedRamp(&Mci[M1],
  (int16_t)(DEFAULT_TARGET_SPEED_RPM/6.0f),0); /*First ramp command */
  pMCIList[M1] = &Mci[M1];
  Mci[M1].pScale = &scaleParams_M1;

  /********************************************************/
  /*   ACIM V/F-control component initialization          */
  /********************************************************/
  pACIM_VF[M1] = pACIM_VF_Component_M1;
  pMCIList[M1]->pACIM_VF = pACIM_VF[M1];
  ACIM_VF_Init(pACIM_VF[M1]);

  /* USER CODE BEGIN MCboot 2 */

  /* USER CODE END MCboot 2 */

  bMCBootCompleted = 1;
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors
 * - Safety Task
 * - Power Factor Correction Task (if enabled)
 * - User Interface task.
 */
__weak void MC_RunMotorControlTasks(void)
{
  if ( bMCBootCompleted ) {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();

    /* Safety task is run after Medium Frequency task so that
     * it can overcome actions they initiated if needed. */
    TSK_SafetyTask();
  }
  else
  {
    /* Nothing to do */
  }
}

/**
 * @brief Performs stop process and update the state machine.This function
 *        shall be called only during medium frequency task
 */
void TSK_MF_StopProcessing(  MCI_Handle_t * pHandle, uint8_t motor)
{
  R3_2_SwitchOffPWM(pwmcHandle[motor]);
  PQD_Clear(pMPM[motor]);
  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance.
 *
 * It is to be clocked at the Systick frequency.
 */
__weak void MC_Scheduler(void)
{
/* USER CODE BEGIN MC_Scheduler 0 */

/* USER CODE END MC_Scheduler 0 */

  if (((uint8_t)1) == bMCBootCompleted)
  {
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      TSK_MediumFrequencyTaskM1();

      MCP_Over_UartA.rxBuffer = MCP_Over_UartA.pTransportLayer->fRXPacketProcess(MCP_Over_UartA.pTransportLayer,
                                                                                &MCP_Over_UartA.rxLength);
      if ( 0U == MCP_Over_UartA.rxBuffer)
      {
        /* Nothing to do */
      }
      else
      {
        /* Synchronous answer */
        if (0U == MCP_Over_UartA.pTransportLayer->fGetBuffer(MCP_Over_UartA.pTransportLayer,
                                                     (void **) &MCP_Over_UartA.txBuffer, //cstat !MISRAC2012-Rule-11.3
                                                     MCTL_SYNC))
        {
          /* no buffer available to build the answer ... should not occur */
        }
        else
        {
          MCP_ReceivedPacket(&MCP_Over_UartA);
          MCP_Over_UartA.pTransportLayer->fSendPacket(MCP_Over_UartA.pTransportLayer, MCP_Over_UartA.txBuffer,
                                                      MCP_Over_UartA.txLength, MCTL_SYNC);
          /* no buffer available to build the answer ... should not occur */
        }
      }

      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
      hMFTaskCounterM1 = (uint16_t)MF_TASK_OCCURENCE_TICKS;
    }
    if(hStopPermanencyCounterM1 > 0U)
    {
      hStopPermanencyCounterM1--;
    }
    else
    {
      /* Nothing to do */
    }
  }
  else
  {
    /* Nothing to do */
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */

  /* USER CODE END MediumFrequencyTask M1 0 */

  PQD_CalcElMotorPower(pMPM[M1]);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {
        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand) || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
            ACIM_VF_Clear(pACIM_VF[M1],&(BusVoltageSensor_M1._Super));
            Mci[M1].State = START;
          }
          else
          {
            /* nothing to be done, FW stays in IDLE state */
          }
          break;
        }

        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {
            Mci[M1].State = START;
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {
          }
          Mci[M1].State = START;
          break;
        }

        case START:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {
            MCI_ExecBufferedCommands(&Mci[M1]); /* Exec the speed ramp after changing of the speed sensor */
            PWM_SwitchOnPWM(pwmHandle[M1]);
            Mci[M1].State = RUN;
          }
          break;
        }

        /* Nothing to do for ACIM No REVUP*/
        case SWITCH_OVER:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {
              /* USER CODE BEGIN MediumFrequencyTask M1 1 */

              /* USER CODE END MediumFrequencyTask M1 1 */
              Mci[M1].State = RUN;
          }
          break;
        }

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(&Mci[M1], M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */
            MCI_ExecBufferedCommands(&Mci[M1]);
            ACIM_VF_FluxFreqCalc(pACIM_VF[M1]);
            /* USER CODE BEGIN MediumFrequencyTask M1 3 */

            /* USER CODE END MediumFrequencyTask M1 3 */
          }
          break;
        }

        case STOP:
        {
          /* Induction motor  V/F        */
          PWM_SwitchOffPWM(pwmHandle[M1]);
          ACIM_VF_Clear(pACIM_VF[M1],&(BusVoltageSensor_M1._Super));

          /* USER CODE BEGIN MediumFrequencyTask M1 4 */

          /* USER CODE END MediumFrequencyTask M1 4 */

          if(TSK_StopPermanencyTimeHasElapsedM1())
          {
            Mci[M1].State = IDLE;
          }
          else
          {
            /* Nothing to do */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* nothing to do, FW stays in FAULT_OVER state until acknowledgement */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        default:
          break;
      }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }

  /* USER CODE BEGIN MediumFrequencyTask M1 6 */

  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM1 == 0)
  {
    retVal = true;
  }
  else
  {
    /* Nothing to do */
  }
  return (retVal);
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
  * @brief  Executes the Motor Control duties that require a high frequency rate and a precise timing
  *
  *  This is mainly the FOC current control loop. It is executed depending on the state of the Motor Control
  * subsystem (see the state machine(s)).
  *
  * @retval Number of the  motor instance which FOC loop was executed.
  */
__weak uint8_t TSK_HighFrequencyTask(void)
{
  /* USER CODE BEGIN HighFrequencyTask 0 */

  /* USER CODE END HighFrequencyTask 0 */
  uint16_t returnValue;
  uint8_t bMotorNbr = 0;

  ACIM_VF_CalcAngle(pACIM_VF[M1]);

  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */

  returnValue = ACIM_VF_Controller(pACIM_VF[M1]);

  if(returnValue == MC_DURATION)
  {
    // Induction motor  V/F
    PWM_SwitchOffPWM(pwmHandle[M1]);
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }

  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */

  /* USER CODE BEGIN HighFrequencyTask 1 */

  /* USER CODE END HighFrequencyTask 1 */

  GLOBAL_TIMESTAMP++;
  if (0U == MCPA_UART_A.Mark)
  {
    /* Nothing to do */
  }
  else
  {
    MCPA_dataLog (&MCPA_UART_A);
  }

  return (bMotorNbr);
}

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances.
  *
  * Faults flags are updated here.
  */
__weak void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (bMCBootCompleted == 1)
  {
    TSK_SafetyTask_PWMOFF(M1);
    /* User conversion execution */
    RCM_ExecUserConv ();
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
  else
  {
    /* Nothing to do */
  }
}

/**
  * @brief  Safety task implementation if  MC.M1_ON_OVER_VOLTAGE == TURN_OFF_PWM
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  */
__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */

   uint16_t CodeReturn = MC_NO_ERROR;
   uint8_t lbMotor = M1;
  const uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
  /* Check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 = RCM_ExecRegularConv(&TempRegConv_M1);
    CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(&TempSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }

  /* Due to warning array subscript 1 is above array bounds of PWMC_Handle_t *[1] [-Warray-bounds] */
   CodeReturn |= PWMC_IsFaultOccurred(pwmcHandle[lbMotor]);     /* check for fault. It return MC_OVER_CURR or MC_NO_FAULTS
                                                     (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */

  if (M1 == bMotor)
  {
    uint16_t rawValueM1 =  RCM_ExecRegularConv(&VbusRegConv_M1);
    CodeReturn |= errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }
  MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn); /* process faults */

  if (MCI_GetFaultState(&Mci[bMotor]) != (uint32_t)MC_NO_FAULTS)
  {
    // Induction motor  V/F
    PWM_SwitchOffPWM(pwmHandle[M1]);
    ACIM_VF_Clear(pACIM_VF[M1],&(BusVoltageSensor_M1._Super));

    if (MCPA_UART_A.Mark != 0)
    {
      MCPA_flushDataLog (&MCPA_UART_A);
    }
    else
    {
      /* Nothing to do */
    }

    PQD_Clear(pMPM[bMotor]); //cstat !MISRAC2012-Rule-11.3

    /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 1 */

    /* USER CODE END TSK_SafetyTask_PWMOFF 1 */
  }
  else
  {
    /* no errors */
  }

  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}

/**
  * @brief  Safety task implementation if  MC.M1_ON_OVER_VOLTAGE == TURN_ON_R_BRAKE
  * @param  motor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  */
__weak void TSK_SafetyTask_RBRK(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_RBRK 0 */

  /* USER CODE END TSK_SafetyTask_RBRK 0 */
  uint16_t CodeReturn = MC_NO_ERROR;
  uint16_t BusVoltageFaultsFlag = MC_OVER_VOLT;
  uint8_t lbMotor = M1;

  uint16_t errMask[NBR_OF_MOTORS] = {VBUS_TEMP_ERR_MASK};
  /* Brake resistor management */
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 =  RCM_ExecRegularConv(&VbusRegConv_M1);
    BusVoltageFaultsFlag =  errMask[bMotor] & RVBS_CalcAvVbus(&BusVoltageSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }

  if (MC_OVER_VOLT == BusVoltageFaultsFlag)
  {
    DOUT_SetOutputState(pR_Brake[lbMotor], ACTIVE);
  }
  else
  {
    DOUT_SetOutputState(pR_Brake[lbMotor], INACTIVE);
  }
  /* Check for fault if FW protection is activated. It returns MC_OVER_TEMP or MC_NO_ERROR */
  if (M1 == bMotor)
  {
    uint16_t rawValueM1 = RCM_ExecRegularConv(&TempRegConv_M1);
    CodeReturn |= errMask[bMotor] & NTC_CalcAvTemp(&TempSensor_M1, rawValueM1);
  }
  else
  {
    /* Nothing to do */
  }

  CodeReturn |= PWMC_IsFaultOccurred (pwmcHandle[bMotor]);    /* check for fault. It return MC_OVER_CURR or MC_NO_FAULTS
                                                                 (for STM32F30x can return MC_OVER_VOLT in case of HW Overvoltage) */
  CodeReturn |= (BusVoltageFaultsFlag & MC_UNDER_VOLT);       /* MC_UNDER_VOLT generates fault if FW protection is activated,
                                                                 MC_OVER_VOLT doesn't generate fault */
  MCI_FaultProcessing(&Mci[bMotor], CodeReturn, ~CodeReturn); /* Update the STM according error code */

  if (MCI_GetFaultState(&Mci[bMotor]) != (uint32_t)MC_NO_FAULTS)
  {

    PWM_SwitchOffPWM(pwmHandle[M1]);
    if (MCPA_UART_A.Mark != 0)
    { /* Dual motor not yet supported */
      MCPA_flushDataLog (&MCPA_UART_A);
    }
    else
    {
      /* Nothing to do */
    }

    PQD_Clear(pMPM[bMotor]); //cstat !MISRAC2012-Rule-11.3
    /* USER CODE BEGIN TSK_SafetyTask_RBRK 1 */

    /* USER CODE END TSK_SafetyTask_RBRK 1 */
  }
  /* USER CODE BEGIN TSK_SafetyTask_RBRK 2 */

  /* USER CODE END TSK_SafetyTask_RBRK 2 */
}

/**
  * @brief  This function returns the reference of the MCInterface relative to
  *         the selected drive.
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval MCI_Handle_t * Reference to MCInterface relative to the selected drive.
  *         Note: it can be MC_NULL if MCInterface of selected drive is not
  *         allocated.
  */
__weak MCI_Handle_t * GetMCI(uint8_t bMotor)
{
  MCI_Handle_t * retVal = MC_NULL;
  if (bMotor < NBR_OF_MOTORS)
  {
    retVal = &Mci[bMotor];
  }
  else
  {
    /* Nothing to do */
  }
  return retVal;
}

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected
  * by the microcontroller and is used to put the system in safety condition.
  */
__weak void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */

  PWM_SwitchOffPWM(pwmHandle[M1]);
  MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0);

  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}

__weak void UI_HandleStartStopButton_cb (void)
{
/* USER CODE BEGIN START_STOP_BTN */
  if (MC_GetSTMStateMotor1() == IDLE)
  {
    /* Ramp parameters should be tuned for the actual motor */
    MC_StartMotor1();
  }
  else
  {
    MC_StopMotor1();
  }
/* USER CODE END START_STOP_BTN */
}

 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration
  */
__weak void mc_lock_pins (void)
{
LL_GPIO_LockPin(M1_PWM_UL_GPIO_Port, M1_PWM_UL_Pin);
LL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
LL_GPIO_LockPin(M1_PWM_VL_GPIO_Port, M1_PWM_VL_Pin);
LL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
LL_GPIO_LockPin(M1_PWM_WL_GPIO_Port, M1_PWM_WL_Pin);
LL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
LL_GPIO_LockPin(M1_OCP_GPIO_Port, M1_OCP_Pin);
LL_GPIO_LockPin(M1_BUS_VOLTAGE_GPIO_Port, M1_BUS_VOLTAGE_Pin);
LL_GPIO_LockPin(M1_TEMPERATURE_GPIO_Port, M1_TEMPERATURE_Pin);
}

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
