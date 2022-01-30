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

#define Throttle_Pin GPIO_PIN_1
#define Throttle_GPIO_Port GPIOA

// ADC channels
#define ADC_THROTTLE 1

extern void UserSysTickHandler(void);

void _Error_Handler(char*, int);

#ifdef __cplusplus
}
#endif
