#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

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

#define iabs(x) (((x) >= 0)?(x):-(x))
#define sign(x) (((x) >= 0)?(1):(-1))

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
	uint8_t hall_angle_detect_flag;
	uint8_t char_dyn_adc_state;
	uint8_t assist_level;
	uint8_t regen_level;
	int8_t Temperature;
	int8_t system_state;
	int8_t error_state;
	uint8_t shutdown;
	int8_t speed_limit;
	int16_t phase_current_limit;
  int16_t spec_angle;
  bool brake_active;
} MotorState_t;

typedef struct {
	q31_t i_d;
	q31_t i_q;
	q31_t i_q_setpoint;
	q31_t i_d_setpoint;
	q31_t u_d;
	q31_t u_q;
	q31_t u_abs;
	q31_t Battery_Current;
	uint8_t hall_angle_detect_flag;
	uint8_t assist_level;
	uint8_t regen_level;
	int8_t Temperature;
	int8_t system_state;
	int8_t mode;
	int8_t error_state;
	bool light;
	bool beep;
	uint8_t shutdown;
	int16_t phase_current_limit;
  uint16_t adcData[6]; // buffer for ADC1 inputs
  int8_t speed_limit;
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
  hall = 1,
  lowbattery = 2,
  overcurrent = 4
};

void motor_init(volatile MotorStatePublic_t* p_MotorStatePublic);
void motor_autodetect();
void motor_runPIcontrol();
void motor_slow_loop(volatile MotorStatePublic_t* p_MotorStatePublic, M365State_t* p_M365State);

#ifdef __cplusplus
}
#endif
