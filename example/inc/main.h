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

// motor hall pins (will be used as input EXTI) (used by the motor control code)
#define HALL_1_GPIO_PORT    GPIOC
#define HALL_1_PIN          GPIO_PIN_13
#define HALL_2_GPIO_PORT    GPIOC
#define HALL_2_PIN          GPIO_PIN_14
#define HALL_3_GPIO_PORT    GPIOC
#define HALL_3_PIN          GPIO_PIN_15

// motor ADC pins (used by the motor control code)
#define MOTOR_PHASE_CURRENT_A_PORT  GPIOA
#define MOTOR_PHASE_CURRENT_A_PIN   GPIO_PIN_3
#define MOTOR_PHASE_CURRENT_B_PORT  GPIOA
#define MOTOR_PHASE_CURRENT_B_PIN   GPIO_PIN_4
#define MOTOR_PHASE_CURRENT_C_PORT  GPIOA
#define MOTOR_PHASE_CURRENT_C_PIN   GPIO_PIN_5

// battery voltage ADC pin (used by the motor control code)
#define BATTERY_VOLTAGE_PORT  GPIOA
#define BATTERY_VOLTAGE_PIN   GPIO_PIN_2

// throttle ADC pin (used by the user application code)
#define THROTTLE_PORT   GPIOA
#define THROTTLE_PIN    GPIO_PIN_7

// wheel speed sensor (used by the user application code)
#define WHELL_SPEED_SENSOR_PORT   GPIOB
#define WHELL_SPEED_SENSOR_PIN    GPIO_PIN_0

#define DEBUG_PORT   GPIOA
#define DEBUG_PIN    GPIO_PIN_1
#define DEBUG_PIN_CONFIG \
  GPIO_InitStruct.Pin = GPIO_PIN_1; \
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; \
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW; \
  HAL_GPIO_Init(DEBUG_PORT, &GPIO_InitStruct)

#define DEBUG_TOGGLE \
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1) // DEBUG

#define DEBUG_ON \
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, true) // DEBUG

#define DEBUG_OFF \
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, false) // DEBUG

extern UART_HandleTypeDef huart3;
extern volatile uint8_t ui8_UART_TxCplt_flag;

void Error_Handler(void);

extern void UserSysTickHandler(void);

void _Error_Handler(char*, int);

#ifdef __cplusplus
}
#endif
