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

// ADC channels used to measure currents
#define ADC_CHANA 3
#define ADC_CHANB 4
#define ADC_CHANC 5

// ADC channel to measure the battery voltage
#define ADC_VOLTAGE 0

// calibration factors for voltage and current
#define CAL_BAT_V 13 	// ADC counts * CAL_BAT_V = Battery voltage in mV

// battery voltage limits in mV
#define BATTERYVOLTAGE_MIN 33000
#define BATTERYVOLTAGE_MAX 42000

#define CAL_I 38 // ADC counts * CAL_I = current in mA

// maximum currents in mA
#define BATTERYCURRENT_MAX 10000 // 10A, 36V battery, 350W limit
#define REGEN_CURRENT_MAX 6000

#define P_FACTOR_I_Q 100
#define I_FACTOR_I_Q 2
#define P_FACTOR_I_D 100
#define I_FACTOR_I_D 10
#define MAX_D_FACTOR 1

#define SPEEDFILTER 3
#define SPEC_ANGLE -167026406L
#define REVERSE 1 // 1 for original M365 motor

// M365 motor
#define P_FACTOR_PLL 8 // 8 for original M365 motor
#define I_FACTOR_PLL 8 // 8 for original M365 motor
#define SIXSTEPTHRESHOLD 9000

// TSDZ2 36V motor
// #define P_FACTOR_PLL 6
// #define I_FACTOR_PLL 8
// #define SIXSTEPTHRESHOLD 27000

#define Phase_Current_1_Pin GPIO_PIN_3
#define Phase_Current_1_GPIO_Port GPIOA
#define Phase_Current_2_Pin GPIO_PIN_4
#define Phase_Current_2_GPIO_Port GPIOA
#define Phase_Current_3_Pin GPIO_PIN_5
#define Phase_Current_3_GPIO_Port GPIOA

#define Batt_Voltage_Pin GPIO_PIN_2
#define Batt_Voltage_GPIO_Port GPIOA

typedef struct {
  uint16_t* pins; // The external interrupt/event controller consists of 19 edge detector lines used to generate
  GPIO_TypeDef** ports;
} PinsConfig_t;

typedef struct {
  PinsConfig_t motor;
  PinsConfig_t user;
  void (*user_exti_callback)(uint32_t* exti_pins);
} ExtiConfig_t;

typedef struct {
  PinsConfig_t motor;
  PinsConfig_t user;
} ADCConfig_t;

typedef struct {
  ExtiConfig_t exti; // EXTI interrupt configurations. There are 3 EXTI used by the motor to read motor hall sensors but user can use other pins and have a callback
  ADCConfig_t adc; // ADC configurations. There are 3 ADCs used by the motor to read motor phase currents but user can use other ADC pins and have later read their data
} MotorConfig_t;

typedef struct {
  q31_t i_q_setpoint_target;
  int16_t phase_current_limit;
  q31_t battery_voltage;
  q31_t battery_voltage_min;
  uint16_t field_weakening_current_max;
  int8_t system_state;
  int8_t mode;
  int8_t error_state;
  int8_t speed_limit;
  uint32_t speed;
  bool brake_active;
  bool field_weakening_enable;
  uint16_t adcData[6]; // buffer for ADC1 inputs
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

void motor_init(MotorConfig_t* motorConfig, MotorStatePublic_t* motorStatePublic);
void motor_autodetect();
void motor_slow_loop(MotorStatePublic_t* p_MotorStatePublic);
void motor_disable_pwm();

#ifdef __cplusplus
}
#endif
