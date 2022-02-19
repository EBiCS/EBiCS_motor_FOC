#include "main.h"
#include "print.h"
#include "motor.h"

#define THROTTLEOFFSET 1100
#define THROTTLEMAX 3250
#define BRAKEOFFSET 50
#define BRAKEMAX 190

// motor current limits for invividual modes in mA
#define PH_CURRENT_MAX 20000

// motor current limit for regen in mA
#define REGEN_CURRENT 10000

// maximum current for field weakening in mA
#define FIELD_WEAKNING_CURRENT_MAX 0 //max id

// not need to optimize the motor_slow_loop
#pragma GCC push_options
#pragma GCC optimize ("O0")

UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

MotorStatePublic_t MSPublic;

volatile uint32_t systick_cnt = 0;

// every 1ms
void UserSysTickHandler(void) {
  static uint32_t c;  
  
  systick_cnt++;

  c++;
  // every 10ms
  if ((c % 10) == 0) {
    motor_slow_loop(&MSPublic);
  }

  // DEBUG_TOGGLE;
}

/**
 * Enable DMA controller clock
 */
static void DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  // DMA channel 3: used for USART3_RX
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
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
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void USART3_UART_Init(void) {

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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void GPIO_Init(void) {
  // init here any needed pins
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
    motor_disable_pwm();
  }
}

void _Error_Handler(char *file, int line) {
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
    motor_disable_pwm();
  }
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

void exti_callback() {

}

void init_motor() {

 // set the EXTI interrupts pins used by the motor hall sensors
  static GPIO_TypeDef* motor_exti_ports[] = {
    HALL_1_GPIO_PORT,
    HALL_2_GPIO_PORT,
    HALL_3_GPIO_PORT,
    NULL // MUST be 0 (null) terminated
  };
  static uint16_t motor_exti_pins[] = {
    HALL_1_PIN,
    HALL_2_PIN,
    HALL_3_PIN
  };

  // set the EXTI interrupts pins used by the user application
  // this next arrays must be static!!
  static GPIO_TypeDef* user_exti_ports[] = {
    WHELL_SPEED_SENSOR_PORT,
    NULL // MUST be 0 (null) terminated
  };
  static uint16_t user_exti_pins[] = {
    WHELL_SPEED_SENSOR_PIN
  };

  // set the ADC pins used by the motor to measure motor phase currents
  static GPIO_TypeDef* motor_adc_ports[] = {
    MOTOR_PHASE_CURRENT_A_PORT,
    MOTOR_PHASE_CURRENT_B_PORT,
    MOTOR_PHASE_CURRENT_C_PORT,
    BATTERY_VOLTAGE_PORT,
    NULL // MUST be 0 (null) terminated
  };
  static uint16_t motor_adc_pins[] = {
    MOTOR_PHASE_CURRENT_A_PIN,
    MOTOR_PHASE_CURRENT_B_PIN,
    MOTOR_PHASE_CURRENT_C_PIN,
    BATTERY_VOLTAGE_PIN
  };

  // set the ADC pins used by the user application
  static GPIO_TypeDef* user_adc_ports[] = {
    THROTTLE_PORT,
    NULL // MUST be 0 (null) terminated
  };
  static uint16_t user_adc_pins[] = {
    THROTTLE_PIN
  };
  #define THROTTLE_ADC_ARRAY_POSITION 4 // position 0, 1, 2 and 3 are used by the motor

  static MotorConfig_t motor_config; // this motor_config MUST be static!!
  // EXTI pins
  motor_config.exti.motor.pins = motor_exti_pins;
  motor_config.exti.motor.ports = motor_exti_ports;
  motor_config.exti.user.pins = user_exti_pins;
  motor_config.exti.user.ports = user_exti_ports;
  motor_config.exti.user_exti_callback = &exti_callback;

  // ADC pins
  motor_config.adc.motor.pins = motor_adc_pins;
  motor_config.adc.motor.ports = motor_adc_ports;
  motor_config.adc.user.pins = user_adc_pins;
  motor_config.adc.user.ports = user_adc_ports;

  MSPublic.brake_active = false;
  MSPublic.i_q_setpoint_target = 0; // start at 0 until throttle value is readed
  MSPublic.speed = 128000;
  MSPublic.speed_limit = 20; // 20km/h
  MSPublic.phase_current_limit = PH_CURRENT_MAX;
  MSPublic.field_weakening_current_max = FIELD_WEAKNING_CURRENT_MAX;
  MSPublic.battery_voltage_min = BATTERYVOLTAGE_MIN;

  motor_init(&motor_config, &MSPublic);
}

int main(void) {
  
  // Reset of all peripherals, Initializes the Flash interface and the Systick
  HAL_Init();

  // Configure the system clock
  // board do not have any external crystal and so we use internal clock. Final clock is 64MHz
  SystemClock_Config();

  // init GPIOS
  GPIO_Init();

  // init DMA for ADC, USART_1 and USART_3
  DMA_Init();

  // init USART_3 for debug
  USART3_UART_Init();

  init_motor();

  // at begin, if throttle is at least halfway, do motor autodetect
  if (MSPublic.adcData[THROTTLE_ADC_ARRAY_POSITION] > (THROTTLEOFFSET + ((THROTTLEMAX - THROTTLEOFFSET) >> 1))) {
    motor_autodetect();
  }

  while (1) {

    //slow loop process, every 20ms
    static uint32_t systick_cnt_old = 0;
    if ((systick_cnt_old != systick_cnt) && // only at a change
        (systick_cnt % 20) == 0) { // every 20ms
      systick_cnt_old = systick_cnt;

      // low pass filter torque signal
      static uint32_t ui32_throttle_acc = 0;
      uint16_t ui16_throttle;
      ui32_throttle_acc -= ui32_throttle_acc >> 4;
      ui32_throttle_acc += MSPublic.adcData[THROTTLE_ADC_ARRAY_POSITION];
      ui16_throttle = ui32_throttle_acc >> 4;

      // map throttle to motor current
      // check to see if throttle value is at least half of the expected offset, if not, probably the throttle is not connected 
      if (ui16_throttle > (THROTTLEOFFSET >> 1)) {
        MSPublic.i_q_setpoint_target = map(ui16_throttle, THROTTLEOFFSET, THROTTLEMAX, 0, PH_CURRENT_MAX);
      } else {
        MSPublic.i_q_setpoint_target = 0;
      }

      // DEBUG
      static uint8_t debug_cnt = 0;
      if (++debug_cnt > 13) { // every 13 * 20 ms = 260ms
        debug_cnt = 0;
        printf_("%d, %d\n", MSPublic.debug[0], MSPublic.debug[1] * CAL_I);
      }
    }
  }
}
#pragma GCC pop_options
