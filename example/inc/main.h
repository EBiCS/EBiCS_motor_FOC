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

// motor hall pins (will be used as input EXTI)
#define HALL_1_PIN          GPIO_PIN_4
#define HALL_1_GPIO_PORT    GPIOB
#define HALL_2_PIN          GPIO_PIN_5
#define HALL_2_GPIO_PORT    GPIOB
#define HALL_3_PIN          GPIO_PIN_0
#define HALL_3_GPIO_PORT    GPIOB

// motor ADC pins
#define MOTOR_PHASE_CURRENT_A_PIN 3
#define MOTOR_PHASE_CURRENT_A_PORT GPIOA
#define MOTOR_PHASE_CURRENT_B_PIN 4
#define MOTOR_PHASE_CURRENT_B_PORT GPIOA
#define MOTOR_PHASE_CURRENT_C_PIN 5
#define MOTOR_PHASE_CURRENT_C_PORT GPIOA

// throttle ADC pin
#define BATTERY_VOLTAGE_PIN 2
#define BATTERY_VOLTAGE_PORT GPIOA
#define THROTTLE_PIN 7
#define THROTTLE_PORT GPIOA

// wheel speed sensor 
#define WHELL_SPEED_SENSOR_PIN GPIO_PIN_0
#define WHELL_SPEED_SENSOR_PORT GPIOB


#define DEBUG_PIN_CONFIG \
  GPIO_InitStruct.Pin = GPIO_PIN_15; \
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; \
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW; \
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct)

#define DEBUG_TOGGLE \
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15) // DEBUG

#define DEBUG_ON \
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, true) // DEBUG

#define DEBUG_OFF \
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, false) // DEBUG

void Error_Handler(void);

extern void UserSysTickHandler(void);

void _Error_Handler(char*, int);

#ifdef __cplusplus
}
#endif
