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

// motor hall sensor pins
#define HALL_3_Pin GPIO_PIN_0
#define HALL_3_GPIO_Port GPIOB
#define HALL_1_Pin GPIO_PIN_4
#define HALL_1_GPIO_Port GPIOB
#define HALL_2_Pin GPIO_PIN_5
#define HALL_2_GPIO_Port GPIOB

//#define FAST_LOOP_LOG
//#define DISABLE_DYNAMIC_ADC

#define P_FACTOR_I_Q 100
#define I_FACTOR_I_Q 2
#define P_FACTOR_I_D 1
#define I_FACTOR_I_D 1
#define MAX_D_FACTOR 1

#define PUSHASSIST_CURRENT 30
#define SIXSTEPTHRESHOLD 9000

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

enum {
  Stop,
  SixStep,
  Interpolation,
  PLL
};

enum modes {
  eco = 2,
  normal = 0,
  sport = 4
};

enum errors {hall=1,lowbattery=2,overcurrent=4};

void motor_init(volatile MotorStatePublic_t* p_MotorStatePublic);
void motor_autodetect();
void runPIcontrol();
void motor_slow_loop(volatile MotorStatePublic_t* p_MotorStatePublic, M365State_t* p_M365State);

#ifdef __cplusplus
}
#endif
