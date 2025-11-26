/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: control.h
 *
 * Code generated for Simulink model 'control'.
 *
 * Model version                  : 3.9
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Wed Nov 26 13:34:49 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: AMD->x86-64 (Linux 64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef control_h_
#define control_h_
#ifndef control_COMMON_INCLUDES_
#define control_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* control_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE[6];          /* '<S1>/Unit Delay' */
  real_T UnitDelay1_DSTATE[4];         /* '<S1>/Unit Delay1' */
  real_T UnitDelay2_DSTATE[6];         /* '<S1>/Unit Delay2' */
  real_T Integrator_DSTATE[6];         /* '<S41>/Integrator' */
  real_T v[6];                         /* '<S1>/MATLAB Function3' */
  int32_T clockTickCounter;            /* '<S1>/Pulse Generator' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T steerang;                     /* '<Root>/steerang' */
  real_T actspeed[6];                  /* '<Root>/actspeed' */
  real_T actang[4];                    /* '<Root>/actang' */
  real_T dist2goal;                    /* '<Root>/dist2goal' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T desspeed[6];                  /* '<Root>/desspeed' */
  real_T controlb[6];                  /* '<Root>/controlb' */
  real_T desang[4];                    /* '<Root>/desang' */
  real_T pwnenable[4];                 /* '<Root>/pwnenable' */
  real_T pwmrev[4];                    /* '<Root>/pwmrev' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void control_initialize(void);
extern void control_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('codegen/control')    - opens subsystem codegen/control
 * hilite_system('codegen/control/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'codegen'
 * '<S1>'   : 'codegen/control'
 * '<S2>'   : 'codegen/control/MATLAB Function'
 * '<S3>'   : 'codegen/control/MATLAB Function1'
 * '<S4>'   : 'codegen/control/MATLAB Function2'
 * '<S5>'   : 'codegen/control/MATLAB Function3'
 * '<S6>'   : 'codegen/control/PID Controller1'
 * '<S7>'   : 'codegen/control/PID Controller1/Anti-windup'
 * '<S8>'   : 'codegen/control/PID Controller1/D Gain'
 * '<S9>'   : 'codegen/control/PID Controller1/External Derivative'
 * '<S10>'  : 'codegen/control/PID Controller1/Filter'
 * '<S11>'  : 'codegen/control/PID Controller1/Filter ICs'
 * '<S12>'  : 'codegen/control/PID Controller1/I Gain'
 * '<S13>'  : 'codegen/control/PID Controller1/Ideal P Gain'
 * '<S14>'  : 'codegen/control/PID Controller1/Ideal P Gain Fdbk'
 * '<S15>'  : 'codegen/control/PID Controller1/Integrator'
 * '<S16>'  : 'codegen/control/PID Controller1/Integrator ICs'
 * '<S17>'  : 'codegen/control/PID Controller1/N Copy'
 * '<S18>'  : 'codegen/control/PID Controller1/N Gain'
 * '<S19>'  : 'codegen/control/PID Controller1/P Copy'
 * '<S20>'  : 'codegen/control/PID Controller1/Parallel P Gain'
 * '<S21>'  : 'codegen/control/PID Controller1/Reset Signal'
 * '<S22>'  : 'codegen/control/PID Controller1/Saturation'
 * '<S23>'  : 'codegen/control/PID Controller1/Saturation Fdbk'
 * '<S24>'  : 'codegen/control/PID Controller1/Sum'
 * '<S25>'  : 'codegen/control/PID Controller1/Sum Fdbk'
 * '<S26>'  : 'codegen/control/PID Controller1/Tracking Mode'
 * '<S27>'  : 'codegen/control/PID Controller1/Tracking Mode Sum'
 * '<S28>'  : 'codegen/control/PID Controller1/Tsamp - Integral'
 * '<S29>'  : 'codegen/control/PID Controller1/Tsamp - Ngain'
 * '<S30>'  : 'codegen/control/PID Controller1/postSat Signal'
 * '<S31>'  : 'codegen/control/PID Controller1/preInt Signal'
 * '<S32>'  : 'codegen/control/PID Controller1/preSat Signal'
 * '<S33>'  : 'codegen/control/PID Controller1/Anti-windup/Passthrough'
 * '<S34>'  : 'codegen/control/PID Controller1/D Gain/Disabled'
 * '<S35>'  : 'codegen/control/PID Controller1/External Derivative/Disabled'
 * '<S36>'  : 'codegen/control/PID Controller1/Filter/Disabled'
 * '<S37>'  : 'codegen/control/PID Controller1/Filter ICs/Disabled'
 * '<S38>'  : 'codegen/control/PID Controller1/I Gain/Internal Parameters'
 * '<S39>'  : 'codegen/control/PID Controller1/Ideal P Gain/Passthrough'
 * '<S40>'  : 'codegen/control/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S41>'  : 'codegen/control/PID Controller1/Integrator/Discrete'
 * '<S42>'  : 'codegen/control/PID Controller1/Integrator ICs/Internal IC'
 * '<S43>'  : 'codegen/control/PID Controller1/N Copy/Disabled wSignal Specification'
 * '<S44>'  : 'codegen/control/PID Controller1/N Gain/Disabled'
 * '<S45>'  : 'codegen/control/PID Controller1/P Copy/Disabled'
 * '<S46>'  : 'codegen/control/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S47>'  : 'codegen/control/PID Controller1/Reset Signal/Disabled'
 * '<S48>'  : 'codegen/control/PID Controller1/Saturation/Passthrough'
 * '<S49>'  : 'codegen/control/PID Controller1/Saturation Fdbk/Disabled'
 * '<S50>'  : 'codegen/control/PID Controller1/Sum/Sum_PI'
 * '<S51>'  : 'codegen/control/PID Controller1/Sum Fdbk/Disabled'
 * '<S52>'  : 'codegen/control/PID Controller1/Tracking Mode/Disabled'
 * '<S53>'  : 'codegen/control/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S54>'  : 'codegen/control/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S55>'  : 'codegen/control/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S56>'  : 'codegen/control/PID Controller1/postSat Signal/Forward_Path'
 * '<S57>'  : 'codegen/control/PID Controller1/preInt Signal/Internal PreInt'
 * '<S58>'  : 'codegen/control/PID Controller1/preSat Signal/Forward_Path'
 */
#endif                                 /* control_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
