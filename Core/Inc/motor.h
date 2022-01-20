#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// maximum currents in mA
#define BATTERYCURRENT_MAX 10000 // 10A, 36V battery, 350W limit
#define REGEN_CURRENT_MAX 6000

// parameters for speed calculation
#define WHEEL_CIRCUMFERENCE 690 // 690 for original M365 motor
#define GEAR_RATIO 15 // 15 for original M365 motor

// ADC channels used to measure currents
#define ADC_CHANA 3
#define ADC_CHANB 4
#define ADC_CHANC 5

#define CAL_I 38 // ADC counts * CAL_I = current in mA

#define P_FACTOR_I_Q 100
#define I_FACTOR_I_Q 2
#define P_FACTOR_I_D 100
#define I_FACTOR_I_D 10
#define MAX_D_FACTOR 1

#define SPEEDFILTER 3
#define SPEC_ANGLE -167026406L
#define REVERSE 1 // 1 for original M365 motor

// settings for speed PLL (angle estimation)
#define P_FACTOR_PLL 10 // 10 for original M365 motor
#define I_FACTOR_PLL 10 // 10 for original M365 motor

#define SIXSTEPTHRESHOLD 18000

typedef struct {
  q31_t i_q_setpoint_target;
  int16_t phase_current_limit;
  q31_t battery_voltage;
  q31_t battery_voltage_min;
  uint16_t field_weakening_current_max;
  int8_t system_state;
  int8_t mode;
  int8_t error_state;
  uint16_t adcData[6]; // buffer for ADC1 inputs
  int8_t speed_limit;
  uint32_t speed;
  bool brake_active;
  bool field_weakening_enable;
  uint32_t debug[10];
} MotorStatePublic_t;

enum angle_estimation {
  EXTRAPOLATION,
  SPEED_PLL,
};

enum {
  Stop,
  SixStep,
  Interpolation,
  PLL
};

enum errors {
  none = 0,
  hall = 18,
  lowbattery = 24,
  overcurrent = 4,
  brake = 15
};

void motor_init(MotorStatePublic_t* p_MotorStatePublic);
void motor_autodetect();
void motor_slow_loop(MotorStatePublic_t* p_MotorStatePublic);
void motor_disable_pwm();

#ifdef __cplusplus
}
#endif
