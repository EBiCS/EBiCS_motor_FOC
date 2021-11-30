/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

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
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
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
#define Phase_Current_1_Pin GPIO_PIN_3
#define Phase_Current_1_GPIO_Port GPIOA
#define Phase_Current_2_Pin GPIO_PIN_4
#define Phase_Current_2_GPIO_Port GPIOA
#define Phase_Current_3_Pin GPIO_PIN_5
#define Phase_Current_3_GPIO_Port GPIOA
#define Phase_Voltage_1_Pin GPIO_PIN_6
#define Phase_Voltage_1_GPIO_Port GPIOA
#define Phase_Voltage_2_Pin GPIO_PIN_7
#define Phase_Voltage_2_GPIO_Port GPIOA
#define Phase_Voltage_3_Pin GPIO_PIN_1
#define Phase_Voltage_3_GPIO_Port GPIOB
#define HALL_3_Pin GPIO_PIN_0
#define HALL_3_GPIO_Port GPIOB
#define HALL_1_Pin GPIO_PIN_4
#define HALL_1_GPIO_Port GPIOB
#define HALL_2_Pin GPIO_PIN_5
#define HALL_2_GPIO_Port GPIOB

#define PWR_BTN_Pin GPIO_PIN_14
#define PWR_BTN_GPIO_Port GPIOC
#define TPS_ENA_Pin GPIO_PIN_15
#define TPS_ENA_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

#define PUSHASSIST_CURRENT 30
#define SIXSTEPTHRESHOLD 9000

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max);

void autodetect();
void runPIcontrol();

extern q31_t switchtime[3];
extern uint32_t ui32_tim1_counter;
extern uint32_t uint32_PAS_counter;

typedef struct {

	q31_t Voltage;
	uint32_t Speed;
	q31_t i_d;
	q31_t i_q_setpoint;
	q31_t i_q;
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
	int8_t gear_state;
	int8_t error_state;

} MotorState_t;

typedef struct {

	uint16_t wheel_cirumference;
	uint16_t p_Iq;
	uint16_t i_Iq;
	uint16_t p_Id;
	uint16_t i_Id;
	uint16_t TS_coeff;
	uint16_t PAS_timeout;
	uint16_t ramp_end;
	uint16_t throttle_offset;
	uint16_t throttle_max;
	uint16_t gear_ratio;
	uint8_t speedLimit;
	uint8_t pulses_per_revolution;
	uint16_t phase_current_max;
	int16_t spec_angle;

} MotorParams_t;

void _Error_Handler(char*, int);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
