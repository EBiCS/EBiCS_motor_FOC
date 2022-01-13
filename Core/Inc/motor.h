#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// parameters for speed calculation
#define WHEEL_CIRCUMFERENCE 690 // 690 for original M365 motor
#define GEAR_RATIO 15 // 15 for original M365 motor

#define ADC_VOLTAGE 0
#define ADC_THROTTLE 1
#define ADC_TEMP 2
#define ADC_CHANA 3
#define ADC_CHANB 4
#define ADC_CHANC 5

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
  hall = 1,
  lowbattery = 2,
  overcurrent = 4
};

void motor_init(MotorStatePublic_t* p_MotorStatePublic);
void motor_autodetect();
void motor_slow_loop(MotorStatePublic_t* p_MotorStatePublic);
void motor_disable_pwm();

#ifdef __cplusplus
}
#endif
