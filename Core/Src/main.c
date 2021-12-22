/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "print.h"
#include "motor.h"
#include "FOC.h"
#include "config.h"
#include "eeprom.h"
#include "button_processing.h"
#include "M365_Dashboard.h"
#include <stdlib.h>
#include <arm_math.h>
/* USER CODE END Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

int c_squared;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int16_t i16_ph3_current = 0;
uint16_t i = 0;
uint16_t j = 0;
uint16_t k = 0;
volatile uint8_t ui8_adc_regular_flag = 0;
volatile int8_t i8_slow_loop_flag = 0;
volatile uint8_t ui8_print_flag = 0;
volatile uint8_t ui8_UART_flag = 0;
volatile uint8_t ui8_Push_Assist_flag = 0;
volatile uint8_t ui8_UART_TxCplt_flag = 1;
volatile uint8_t ui8_PAS_flag = 0;
volatile uint8_t ui8_SPEED_flag = 0;

uint32_t uint32_PAS_HIGH_counter = 0;
uint32_t uint32_PAS_HIGH_accumulated = 32000;
uint32_t uint32_PAS_fraction = 100;
uint32_t uint32_SPEED_counter = 32000;
uint32_t uint32_PAS = 32000;

uint8_t ui8_UART_Counter = 0;

uint32_t uint32_torque_cumulated = 0;
uint32_t uint32_PAS_cumulated = 32000;
uint16_t uint16_mapped_throttle = 0;
uint16_t uint16_mapped_PAS = 0;
uint16_t uint16_half_rotation_counter = 0;
uint16_t uint16_full_rotation_counter = 0;
int32_t int32_current_target = 0;

q31_t q31_Battery_Voltage = 0;

char buffer[256];
uint8_t assist_factor[10] = { 0, 51, 102, 153, 204, 255, 255, 255, 255, 255 };

uint16_t VirtAddVarTab[NB_OF_VAR] = { 0x01, 0x02, 0x03 };

static q31_t tics_lower_limit;
static q31_t tics_higher_limit;
//variables for display communication

#define iabs(x) (((x) >= 0)?(x):-(x))

MotorState_t MS;
MotorParams_t MP;

int16_t battery_percent_fromcapacity = 50; //Calculation of used watthours not implemented yet
int16_t wheel_time = 1000;//duration of one wheel rotation for speed calculation
int16_t current_display;				//pepared battery current for display

int16_t power;

static void dyn_adc_state(q31_t angle);
static void set_inj_channel(char state);
void get_standstill_position();
void runPIcontrol();
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max);
int32_t speed_to_tics(uint8_t speed);
int8_t tics_to_speed(uint32_t tics);
q31_t speed_PLL (q31_t ist, q31_t soll);

#define JSQR_PHASE_A 0b00011000000000000000 //3
#define JSQR_PHASE_B 0b00100000000000000000 //4
#define JSQR_PHASE_C 0b00101000000000000000 //5

#define ADC_VOLTAGE 0
#define ADC_THROTTLE 1
#define ADC_TEMP 2
#define ADC_CHANA 3
#define ADC_CHANB 4
#define ADC_CHANC 5

volatile uint32_t systick_cnt1 = 0;

void UserSysTickHandler(void) {
  systick_cnt1++;
}

int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
  HAL_SetTickFreq(HAL_TICK_FREQ_100HZ); // set systick at 10ms
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();

	calculate_tic_limits();

	//Virtual EEPROM init
	HAL_FLASH_Unlock();
	EE_Init();
	HAL_FLASH_Lock();

	M365Dashboard_init(huart1);
	PWR_init();

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
	printf_("Lishui FOC v0.9 \n ");
#endif
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		//display message processing
		search_DashboardMessage(&MS, &MP, huart1);
		checkButton(&MS);

#if 0 //(DISPLAY_TYPE == DISPLAY_TYPE_DEBUG) // && defined(FAST_LOOP_LOG))
		if(ui8_UART_TxCplt_flag){
	        sprintf_(buffer, "%d, %d, %d, %d, %d, %d\r\n", e_log[k][0], e_log[k][1], e_log[k][2],e_log[k][3],e_log[k][4],e_log[k][5]); //>>24
			i=0;
			while (buffer[i] != '\0')
			{i++;}
			ui8_UART_TxCplt_flag=0;
			HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&buffer, i);
			k++;
			if (k>299){
				k=0;
				ui8_debug_state=0;
				//Obs_flag=0;
			}
		}
#endif

#ifdef ADCTHROTTLE
		MS.i_q_setpoint=map(ui16_reg_adc_value,THROTTLEOFFSET,THROTTLEMAX,0,MS.phase_current_limit);
#endif

		if (i8_slow_loop_flag) {
       i8_slow_loop_flag = 0;

      // low pass filter measured battery voltage 
      static q31_t q31_batt_voltage_acc = 0;
      q31_batt_voltage_acc -= (q31_batt_voltage_acc >> 7);
      q31_batt_voltage_acc += adcData[ADC_VOLTAGE];
      q31_Battery_Voltage = (q31_batt_voltage_acc >> 7) * CAL_BAT_V;
		}

		//slow loop process, every 20ms
		if ((systick_cnt1 % 2) == 0) {
			if(MS.shutdown)MS.shutdown++;

			MS.Temperature = adcData[ADC_TEMP] * 41 >> 8; //0.16 is calibration constant: Analog_in[10mV/°C]/ADC value. Depending on the sensor LM35)
			MS.Voltage = q31_Battery_Voltage;
			printf_("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", MS.i_setpoint_abs, MS.i_q_setpoint, MS.i_q, MS.i_d_setpoint, MS.i_d, MS.u_abs, MS.u_q,MS.u_d,MS.Speed);
			if(MS.system_state==Stop||MS.system_state==SixStep) MS.Speed=0;
			else MS.Speed=tics_to_speed(q31_tics_filtered>>3);

			if (!MS.i_q_setpoint&&(uint16_full_rotation_counter > 7999
					|| uint16_half_rotation_counter > 7999)
					&& READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)) {
				CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM if motor is not turning
				MS.system_state=Stop;
				get_standstill_position();
				//printf_("shutdown %d\n", q31_rotorposition_absolute);
			}

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG && !defined(FAST_LOOP_LOG))
                //Jon Pry uses this crazy string for automated data collection
	  	//	sprintf_(buffer, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, T: %d, Q: %d, D: %d, %d, %d, S: %d, %d, V: %d, C: %d\r\n", int32_current_target, (int16_t) raw_inj1,(int16_t) raw_inj2, (int32_t) MS.char_dyn_adc_state, q31_rotorposition_hall, q31_rotorposition_absolute, (int16_t) (ui16_reg_adc_value),adcData[ADC_CHANA],adcData[ADC_CHANB],adcData[ADC_CHANC],i16_ph1_current,i16_ph2_current, uint16_mapped_throttle, MS.i_q, MS.i_d, MS.u_q, MS.u_d,q31_tics_filtered>>3,tics_higher_limit, adcData[ADC_VOLTAGE], MS.Battery_Current);
	  		sprintf_(buffer, "%d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", MS.i_q_setpoint, MS.i_q,q31_tics_filtered>>3, adcData[ADC_THROTTLE],MS.i_q_setpoint, MS.u_d, MS.u_q , MS.u_abs,  MS.Battery_Current);
	  	//	sprintf_(buffer, "%d, %d, %d, %d, %d, %d\r\n",(uint16_t)adcData[0],(uint16_t)adcData[1],(uint16_t)adcData[2],(uint16_t)adcData[3],(uint16_t)(adcData[4]),(uint16_t)(adcData[5])) ;

	  	  i=0;
		  while (buffer[i] != '\0')
		  {i++;}
		 HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&buffer, i);
		 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		 ui8_print_flag = 0;

#endif
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 2, 0);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_HalfDuplex_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	huart3.Instance = USART3;

	huart3.Init.BaudRate = 115200;

	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PWR_BTN_Pin */
  GPIO_InitStruct.Pin = PWR_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TPS_ENA_Pin */
  GPIO_InitStruct.Pin = TPS_ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TPS_ENA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

//	HAL_GPIO_WritePin(UART1_Tx_GPIO_Port, UART1_Tx_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin : UART1Tx_Pin */
	GPIO_InitStruct.Pin = UART1_Tx_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(UART1_Tx_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(BrakeLight_GPIO_Port, BrakeLight_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin : BrakeLight_Pin */
	GPIO_InitStruct.Pin = BrakeLight_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BrakeLight_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(HALL_1_GPIO_Port, HALL_1_Pin, GPIO_PIN_RESET);
}

/* USER CODE BEGIN 4 */

// regular ADC callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	ui32_reg_adc_value_filter -= ui32_reg_adc_value_filter >> 4;
	ui32_reg_adc_value_filter += adcData[ADC_THROTTLE]; //HAL_ADC_GetValue(hadc);
	ui16_reg_adc_value = ui32_reg_adc_value_filter >> 4;

	ui8_adc_regular_flag = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
	ui8_UART_TxCplt_flag = 1;

	if(UartHandle==&huart1)	HAL_HalfDuplex_EnableReceiver(&huart1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {

}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
		int32_t out_max) {
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min)
		return out_min;
	else if (x > in_max)
		return out_max;

	// map the input to the output range.
	// round up if mapping bigger ranges to smaller ranges
	else if ((in_max - in_min) > (out_max - out_min))
		return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1)
				+ out_min;
	// round down if mapping smaller ranges to bigger ranges
	else
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int32_t speed_to_tics(uint8_t speed) {
	return WHEEL_CIRCUMFERENCE * 5 * 3600 / (6 * GEAR_RATIO * speed * 10);
}

int8_t tics_to_speed(uint32_t tics) {
	return WHEEL_CIRCUMFERENCE * 5 * 3600 / (6 * GEAR_RATIO * tics * 10);;
}

void calculate_tic_limits(void){
	tics_lower_limit = WHEEL_CIRCUMFERENCE * 5 * 3600
			/ (6 * GEAR_RATIO * MS.speed_limit * 10); //tics=wheelcirc*timerfrequency/(no. of hallevents per rev*gear-ratio*speedlimit)*3600/1000000
	tics_higher_limit = WHEEL_CIRCUMFERENCE * 5 * 3600
			/ (6 * GEAR_RATIO * (MS.speed_limit + 2) * 10);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
