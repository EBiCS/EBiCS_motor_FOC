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
#include <arm_math.h>
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
#define PWR_BTN_Pin GPIO_PIN_14
#define PWR_BTN_GPIO_Port GPIOC
#define TPS_ENA_Pin GPIO_PIN_15
#define TPS_ENA_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOD
#define NTC_Pin GPIO_PIN_0
#define NTC_GPIO_Port GPIOA
#define VBAT_Pin GPIO_PIN_2
#define VBAT_GPIO_Port GPIOA
#define CURR_A_Pin GPIO_PIN_3
#define CURR_A_GPIO_Port GPIOA
#define CURR_B_Pin GPIO_PIN_4
#define CURR_B_GPIO_Port GPIOA
#define CURR_C_Pin GPIO_PIN_5
#define CURR_C_GPIO_Port GPIOA
#define VOLT_A_Pin GPIO_PIN_6
#define VOLT_A_GPIO_Port GPIOA
#define VOLT_B_Pin GPIO_PIN_7
#define VOLT_B_GPIO_Port GPIOA
#define HALL_C_Pin GPIO_PIN_0
#define HALL_C_GPIO_Port GPIOB
#define VOLT_C_Pin GPIO_PIN_1
#define VOLT_C_GPIO_Port GPIOB
#define PHA_A_L_Pin GPIO_PIN_13
#define PHA_A_L_GPIO_Port GPIOB
#define PHA_B_L_Pin GPIO_PIN_14
#define PHA_B_L_GPIO_Port GPIOB
#define PHA_C_L_Pin GPIO_PIN_15
#define PHA_C_L_GPIO_Port GPIOB
#define PHA_A_H_Pin GPIO_PIN_8
#define PHA_A_H_GPIO_Port GPIOA
#define PHA_B_H_Pin GPIO_PIN_9
#define PHA_B_H_GPIO_Port GPIOA
#define PHA_C_H_Pin GPIO_PIN_10
#define PHA_C_H_GPIO_Port GPIOA
#define LIGHT_Pin GPIO_PIN_15
#define LIGHT_GPIO_Port GPIOA
#define HALL_A_Pin GPIO_PIN_4
#define HALL_A_GPIO_Port GPIOB
#define HALL_B_Pin GPIO_PIN_5
#define HALL_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define PUSHASSIST_CURRENT 30

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max);

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
	uint16_t spec_angle;

} MotorParams_t;

void _Error_Handler(char*, int);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
