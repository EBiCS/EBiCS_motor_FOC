#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <arm_math.h>
#include <stdbool.h>
#include "utils.h"
#include "motor.h"

void Error_Handler(void);

#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOD
#define UART1_Tx_Pin GPIO_PIN_6
#define UART1_Tx_GPIO_Port GPIOB
#define BrakeLight_Pin GPIO_PIN_15
#define BrakeLight_GPIO_Port GPIOA
#define Temp_Pin GPIO_PIN_0
#define Temp_GPIO_Port GPIOA
#define Throttle_Pin GPIO_PIN_1
#define Throttle_GPIO_Port GPIOA
#define Batt_Voltage_Pin GPIO_PIN_2
#define Batt_Voltage_GPIO_Port GPIOA
#define PWR_BTN_Pin GPIO_PIN_14
#define PWR_BTN_GPIO_Port GPIOC
#define TPS_ENA_Pin GPIO_PIN_15
#define TPS_ENA_GPIO_Port GPIOC

// ADC channels
#define ADC_VOLTAGE 0
#define ADC_THROTTLE 1
#define ADC_TEMP 2

extern void UserSysTickHandler(void);

typedef struct {
  q31_t battery_voltage;
  int16_t phase_current_limit;
  int16_t regen_current;
	int8_t temperature;
	int8_t mode;
	bool light;
	bool beep;
  int32_t i_q_setpoint_target;
  uint32_t speed;
	bool brake_active;
  uint32_t shutdown;
  int8_t speed_limit;
  int8_t error_state;
} M365State_t;

enum modes {
  eco = 2,
  normal = 0,
  sport = 4
};

void _Error_Handler(char*, int);

#ifdef __cplusplus
}
#endif
