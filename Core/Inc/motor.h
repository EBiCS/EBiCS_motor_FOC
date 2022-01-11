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

//#define DISABLE_DYNAMIC_ADC

#define P_FACTOR_I_Q 100
#define I_FACTOR_I_Q 2
#define P_FACTOR_I_D 100
#define I_FACTOR_I_D 10
#define MAX_D_FACTOR 1

#define SPEEDFILTER 3
#define SPEC_ANGLE -167026406L
#define REVERSE 1 // 1 for original M365 motor

// settings for speed PLL (angle estimation)
#define SPEED_PLL 1 // 1 for using PLL, 0 for angle extrapolation
#define P_FACTOR_PLL 10 // 10 for original M365 motor
#define I_FACTOR_PLL 10 // 10 for original M365 motor

#define TRIGGER_OFFSET_ADC 50
#define TRIGGER_DEFAULT 2020
#define _T 2028

#define iabs(x) (((x) >= 0)?(x):-(x))
#define sign(x) (((x) >= 0)?(1):(-1))

#ifdef DISABLE_DYNAMIC_ADC
#define _U_MAX	2000L  / /little lower than period of timer1 for proper phase current reading. Could be improved by dynamic timing of AD-conversion
#else
#define _U_MAX	2000L
#endif

// Square Root of 3
#define _SQRT3	28  //1.73205081*16

typedef struct {
	q31_t i_d;
	q31_t i_q;
	q31_t i_q_setpoint;
	q31_t i_d_setpoint;
	q31_t i_setpoint_abs;
	int32_t i_q_setpoint_temp;
	int32_t i_d_setpoint_temp;
	q31_t u_d;
	q31_t u_q;
	q31_t u_abs;
	q31_t Battery_Current;
	uint8_t char_dyn_adc_state;
	int8_t system_state;
	int8_t error_state;
	uint8_t shutdown;
	int16_t phase_current_limit;
  int16_t spec_angle;
  bool brake_active;
  int8_t angle_est;
	int16_t KV_detect_flag;
  bool hall_angle_detect_flag;
} MotorState_t;

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

typedef struct
{
	int16_t gain_p;
	int16_t gain_i;
	int16_t limit_i;
	int16_t limit_output;
	int16_t recent_value;
	int32_t setpoint;
	int32_t integral_part;
	int16_t max_step;
	int32_t out;
	int8_t shift;
} PI_control_t;

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
void motor_runPIcontrol();
void motor_slow_loop(MotorStatePublic_t* p_MotorStatePublic);
void motor_disable_pwm();

#ifdef __cplusplus
}
#endif
