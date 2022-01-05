#include <stdlib.h>
#include <arm_math.h>
#include "FOC.h"
#include "main.h"
#include "motor.h"
#include "print.h"
#include "eeprom.h"

// EEPROM number of variables to use
uint16_t VirtAddVarTab[NB_OF_VAR] = {
  0x01,
  0x02,
  0x03
};

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

uint16_t adcData1[6 * sizeof(uint32_t)]; // point to buffer for ADC1 inputs

#define JSQR_PHASE_A 0b00011000000000000000 //3
#define JSQR_PHASE_B 0b00100000000000000000 //4
#define JSQR_PHASE_C 0b00101000000000000000 //5

MotorState_t MS;

volatile int16_t i16_ph1_current = 0;
volatile int16_t i16_ph2_current = 0;
volatile int16_t i16_ph2_current_filter = 0;
volatile uint8_t ui8_adc_inj_flag = 0;
q31_t raw_inj1;
q31_t raw_inj2;
volatile uint8_t ui8_hall_state = 0;
volatile uint8_t ui8_hall_state_old = 0;
volatile uint8_t ui8_hall_case = 0;
volatile bool hall_angle_detect_flag = false;
uint8_t ui8_BC_limit_flag = 0;
uint16_t ui16_tim2_recent = 0;
volatile uint16_t ui16_timertics = 5000; //timertics between two hall events for 60° interpolation
volatile uint8_t ui8_6step_flag = 0;
volatile q31_t q31_rotorposition_absolute;
volatile q31_t q31_rotorposition_hall;
q31_t q31_rotorposition_motor_specific = SPEC_ANGLE;
volatile q31_t q31_rotorposition_PLL = 0;
volatile q31_t q31_angle_per_tic = 0;
volatile int8_t i8_recent_rotor_direction = 1;
volatile int16_t i16_hall_order = 1;

// rotor angle scaled from degree to q31 for arm_math. -180°-->-2^31, 0°-->0, +180°-->+2^31 read in from EEPROM
volatile q31_t Hall_13 = 0;
volatile q31_t Hall_32 = 0;
volatile q31_t Hall_26 = 0;
volatile q31_t Hall_64 = 0;
volatile q31_t Hall_51 = 0;
volatile q31_t Hall_45 = 0;

q31_t switchtime[3];
volatile uint8_t ui8_overflow_flag = 0;
char char_dyn_adc_state = 1;
char char_dyn_adc_state_old = 1;
volatile q31_t q31_tics_filtered = 128000;
q31_t q31_t_Battery_Current_accumulated = 0;
int8_t i8_direction = REVERSE;
volatile int8_t i8_reverse_flag = 1; //for temporaribly reverse direction
q31_t q31_u_d_temp = 0;
q31_t q31_u_q_temp = 0;
int16_t i16_sinus = 0;
int16_t i16_cosinus = 0;
uint16_t uint16_half_rotation_counter = 0;
uint16_t uint16_full_rotation_counter = 0;
volatile uint8_t ui8_adc_offset_done_flag = 0;

static q31_t tics_lower_limit;
static q31_t tics_higher_limit;

//Rotor angle scaled from degree to q31 for arm_math. -180°-->-2^31, 0°-->0, +180°-->+2^31
const q31_t DEG_0 = 0;
const q31_t DEG_plus60 = 715827883;
const q31_t DEG_plus120 = 1431655765;
const q31_t DEG_plus180 = 2147483647;
const q31_t DEG_minus60 = -715827883;
const q31_t DEG_minus120 = -1431655765;

volatile uint16_t ui16_reg_adc_value;
volatile uint32_t ui32_reg_adc_value_filter;
volatile uint8_t ui8_adc_regular_flag = 0;
uint16_t ui16_ph1_offset = 0;
uint16_t ui16_ph2_offset = 0;
uint16_t ui16_ph3_offset = 0;

volatile uint8_t ui8_KV_detect_flag = 0;
uint16_t ui16_KV_detect_counter = 0; // for getting timing of the KV detect
static int16_t ui32_KV = 0;

uint32_t uint32_SPEEDx100_cumulated = 0;

static q31_t iq_cum = 0;
static q31_t id_cum = 0;
static q31_t uq_cum = 0;
static q31_t ud_cum = 0;

// structs for PI_control
PI_control_t PI_iq;
PI_control_t PI_id;

volatile MotorStatePublic_t* p_MotorStatePublic;

// regular ADC callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	ui8_adc_regular_flag = 1;
}

static void motor_disable_pwm(void) {
  CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
}

static void motor_enable_pwm(void) {
  SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
}

static bool motor_pwm_is_enabled(void) {
  return READ_BIT(TIM1->BDTR, TIM_BDTR_MOE);
}

q31_t speed_PLL(q31_t actual, q31_t target) {
  static q31_t q31_d_i = 0;

  q31_t delta = target - actual;
  q31_t q31_p = (delta >> P_FACTOR_PLL);
  q31_d_i += (delta >> I_FACTOR_PLL);

  q31_t q31_d_dc = q31_p + q31_d_i;

  if ((actual == 0 && target == 0) ||
    motor_pwm_is_enabled() == false) {
    q31_d_i = 0;
  }

  return q31_d_dc;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// Hall sensor event processing
	if (GPIO_Pin == HALL_1_Pin || GPIO_Pin == HALL_2_Pin || GPIO_Pin == HALL_3_Pin) // check for right interrupt source
	{
		ui8_hall_state = ((GPIOB->IDR & 1) << 2) | ((GPIOB->IDR >> 4) & 0b11); // mask input register with Hall 1 - 3 bits

		if (ui8_hall_state == ui8_hall_state_old)
			return;

		ui8_hall_case = ui8_hall_state_old * 10 + ui8_hall_state; // old state + current state

    // only process, if autodetect process is finished
		if (hall_angle_detect_flag == false) {
			ui8_hall_state_old = ui8_hall_state;
		}

		ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last hall event

		if (ui16_tim2_recent > 100) { // debounce
			ui16_timertics = ui16_tim2_recent; // save timertics since last hall event
			q31_tics_filtered -= q31_tics_filtered >> 3;
			q31_tics_filtered += ui16_timertics;
			__HAL_TIM_SET_COUNTER(&htim2, 0); // reset tim2 counter
			ui8_overflow_flag = 0;
		}

		switch (ui8_hall_case) // 12 cases for each transition from one stage to the next. 6x forward, 6x reverse
		{
      // 6 cases for forward direction
      case 64:
        q31_rotorposition_hall = Hall_64;
        i8_recent_rotor_direction = -i16_hall_order;
        uint16_full_rotation_counter = 0;
        break;

      case 45:
        q31_rotorposition_hall = Hall_45;
        i8_recent_rotor_direction = -i16_hall_order;
        break;

      case 51:
        q31_rotorposition_hall = Hall_51;
        i8_recent_rotor_direction = -i16_hall_order;
        break;

      case 13:
        q31_rotorposition_hall = Hall_13;
        i8_recent_rotor_direction = -i16_hall_order;
        uint16_half_rotation_counter = 0;
        break;

      case 32:
        q31_rotorposition_hall = Hall_32;
        i8_recent_rotor_direction = -i16_hall_order;
        break;

      case 26:
        q31_rotorposition_hall = Hall_26;
        i8_recent_rotor_direction = -i16_hall_order;
        break;

      // 6 cases for reverse direction
      case 46:
        q31_rotorposition_hall = Hall_64;
        i8_recent_rotor_direction = i16_hall_order;
        break;

      case 62:
        q31_rotorposition_hall = Hall_26;
        i8_recent_rotor_direction = i16_hall_order;
        break;

      case 23:
        q31_rotorposition_hall = Hall_32;
        i8_recent_rotor_direction = i16_hall_order;
        uint16_half_rotation_counter = 0;
        break;

      case 31:
        q31_rotorposition_hall = Hall_13;
        i8_recent_rotor_direction = i16_hall_order;
        break;

      case 15:
        q31_rotorposition_hall = Hall_51;
        i8_recent_rotor_direction = i16_hall_order;
        break;

      case 54:
        q31_rotorposition_hall = Hall_45;
        i8_recent_rotor_direction = i16_hall_order;
        uint16_full_rotation_counter = 0;
        break;
		}

#ifdef SPEED_PLL
		q31_angle_per_tic = speed_PLL(q31_rotorposition_PLL, q31_rotorposition_hall);
#endif
	}
}

void get_standstill_position() {
	// HAL_Delay(100); // will BLOCK because it is called inside systick interrupt
	HAL_GPIO_EXTI_Callback(GPIO_PIN_4); // read in initial rotor position

	switch (ui8_hall_state) {
    // 6 cases for forward direction
    case 2:
      q31_rotorposition_hall = Hall_32;
      break;

    case 6:
      q31_rotorposition_hall = Hall_26;
      break;

    case 4:
      q31_rotorposition_hall = Hall_64;
      break;

    case 5:
      q31_rotorposition_hall = Hall_45;
      break;

    case 1:
      q31_rotorposition_hall = Hall_51;
      break;

    case 3:
      q31_rotorposition_hall = Hall_13;
      break;
    }

	q31_rotorposition_absolute = q31_rotorposition_hall;
}

int32_t speed_to_tics(uint8_t speed) {
	return WHEEL_CIRCUMFERENCE * 5 * 3600 / (6 * GEAR_RATIO * speed * 10);
}

int8_t tics_to_speed(uint32_t tics) {
	return WHEEL_CIRCUMFERENCE * 5 * 3600 / (6 * GEAR_RATIO * tics * 10);;
}

void motor_autodetect() {
	motor_enable_pwm();
	hall_angle_detect_flag = true;
	q31_rotorposition_absolute = 1 << 31;
  i16_hall_order = 1; // reset hall order
  MS.i_d_setpoint = 200; // set MS.id to appr. 2000mA
	MS.i_q_setpoint = 0;

	HAL_Delay(5);
	for (uint32_t i = 0; i < 1080; i++) { // 1080 = 360 * 3, 3 turns, just to make sure at least a full turn happens

		q31_rotorposition_absolute += 11930465; //drive motor in open loop with steps of 1 degree
    HAL_Delay(5);

		if (ui8_hall_state_old != ui8_hall_state) {

	    switch (ui8_hall_case) // 12 cases for each transition from one stage to the next. 6x forward, 6x reverse
			{
        // 6 cases for forward direction
        case 64:
          Hall_64 = q31_rotorposition_absolute;
          break;

        case 45:
          Hall_45 = q31_rotorposition_absolute;
          break;

        case 51:
          Hall_51 = q31_rotorposition_absolute;
          break;

        case 13:
          Hall_13 = q31_rotorposition_absolute;
          break;

        case 32:
          Hall_32 = q31_rotorposition_absolute;
          break;

        case 26:
          Hall_26 = q31_rotorposition_absolute;
          break;

        // 6 cases for reverse direction
        case 46:
          Hall_64 = q31_rotorposition_absolute;
          break;

        case 62:
          Hall_26 = q31_rotorposition_absolute;
          break;

        case 23:
          Hall_32 = q31_rotorposition_absolute;
          break;

        case 31:
          Hall_13 = q31_rotorposition_absolute;
          break;

        case 15:
          Hall_51 = q31_rotorposition_absolute;
          break;

        case 54:
          Hall_45 = q31_rotorposition_absolute;
          break;
			} // end case

			ui8_hall_state_old = ui8_hall_state;
		}
	}

  motor_disable_pwm();
  TIM1->CCR1 = 1023; // set initial PWM values
  TIM1->CCR2 = 1023;
  TIM1->CCR3 = 1023;
  hall_angle_detect_flag = false;
  MS.i_d = 0;
  MS.i_q = 0;
  MS.u_d = 0;
  MS.u_q = 0;
  q31_tics_filtered = 1000000;

  // store variables on flash memory
	HAL_FLASH_Unlock();

	if (i8_recent_rotor_direction == 1) {
		EE_WriteVariable(EEPROM_POS_HALL_ORDER, 1);
		i16_hall_order = 1;
	} else {
		EE_WriteVariable(EEPROM_POS_HALL_ORDER, -1);
		i16_hall_order = -1;
	}
	EE_WriteVariable(EEPROM_POS_HALL_45, Hall_45 >> 16);
	EE_WriteVariable(EEPROM_POS_HALL_51, Hall_51 >> 16);
	EE_WriteVariable(EEPROM_POS_HALL_13, Hall_13 >> 16);
	EE_WriteVariable(EEPROM_POS_HALL_32, Hall_32 >> 16);
	EE_WriteVariable(EEPROM_POS_HALL_26, Hall_26 >> 16);
	EE_WriteVariable(EEPROM_POS_HALL_64, Hall_64 >> 16);

	HAL_FLASH_Lock();

	hall_angle_detect_flag = false;
	// ui8_KV_detect_flag = 20;
}

int16_t internal_tics_to_speedx100 (uint32_t tics) {
	return WHEEL_CIRCUMFERENCE * 50 * 3600 / (6 * GEAR_RATIO * tics);
}

void calculate_tic_limits(int8_t speed_limit) {
  // tics = wheelcirc * timerfrequency / (no. of hallevents per rev * gear-ratio * speedlimit) * 3600 / 1000000
	tics_lower_limit = WHEEL_CIRCUMFERENCE * 5 * 3600 / (6 * GEAR_RATIO * speed_limit * 10);
	tics_higher_limit = WHEEL_CIRCUMFERENCE * 5 * 3600 / (6 * GEAR_RATIO * (speed_limit + 2) * 10);
}

q31_t get_battery_current(q31_t iq,q31_t id,q31_t uq,q31_t ud) {
	q31_t ibatq;
	q31_t ibatd;
	ibatq = (iq * uq *CAL_I) >> 11;
	ibatd = (id * ud *CAL_I) >> 11;

	return abs(ibatd) + abs(ibatq);
}

// call every 10ms
void motor_slow_loop(volatile MotorStatePublic_t* p_MotorStatePublic, M365State_t* p_M365State) {

  MS.brake_active = p_M365State->brake_active;

  // i_q current limits
  if (p_M365State->i_q_setpoint_target > p_M365State->phase_current_limit) {
    MS.i_q_setpoint_temp = p_M365State->phase_current_limit;
  }
  if (p_M365State->i_q_setpoint_target < -p_M365State->phase_current_limit) {
    MS.i_q_setpoint_temp = -p_M365State->phase_current_limit;
  }

	calculate_tic_limits(p_M365State->speed_limit);

  // ramp down current at speed limit
  MS.i_q_setpoint_temp = map(q31_tics_filtered >> 3, tics_higher_limit, tics_lower_limit, 0, p_M365State->i_q_setpoint_target);

  // see if we should do flux weakening: i_d current
  if (p_M365State->mode == sport) {
    MS.i_d_setpoint_temp = -map(p_M365State->speed,(ui32_KV * p_M365State->Voltage/100000) - 8, (ui32_KV*p_M365State->Voltage / 100000) + 30, 0, FW_CURRENT_MAX);
  } else {
    MS.i_d_setpoint_temp = 0;
  }

  // check and limit absolute value of current vector
  arm_sqrt_q31((MS.i_q_setpoint_temp * MS.i_q_setpoint_temp + MS.i_d_setpoint_temp * MS.i_d_setpoint_temp) << 1, &MS.i_setpoint_abs);
  MS.i_setpoint_abs = (MS.i_setpoint_abs >> 16) + 1;

  if (hall_angle_detect_flag == false) {
    if (MS.i_setpoint_abs > MS.phase_current_limit) {
      MS.i_q_setpoint = (MS.i_q_setpoint_temp * MS.phase_current_limit) / MS.i_setpoint_abs; // division!
      MS.i_d_setpoint = (MS.i_d_setpoint_temp * MS.phase_current_limit) / MS.i_setpoint_abs; // division!
      MS.i_setpoint_abs = MS.phase_current_limit;
    } else {
      MS.i_q_setpoint=MS.i_q_setpoint_temp;
      MS.i_d_setpoint=MS.i_d_setpoint_temp;
    }
  }

  // detect KV
  if (ui8_KV_detect_flag > 0) {
    MS.i_q_setpoint = ui8_KV_detect_flag;
    MS.i_d_setpoint = 0;
    
    if (MS.u_q > 0) {
      ui32_KV -= ui32_KV >> 4;
      ui32_KV += (uint32_SPEEDx100_cumulated) / ((p_M365State->Voltage * MS.u_q) >> (21 - SPEEDFILTER)); //unit: kph*100/V
    }
    
    if (HAL_GetTick() - ui16_KV_detect_counter > 1000) {
      
      ui8_KV_detect_flag++;
      ui16_KV_detect_counter = HAL_GetTick();
      
      if (MS.u_abs > 1900) {
        ui8_KV_detect_flag = 0;
        ui32_KV = ui32_KV >> 4;

        HAL_FLASH_Unlock();
        EE_WriteVariable(EEPROM_POS_KV, (int16_t) (ui32_KV));
        HAL_FLASH_Lock();
      }
    }
  }

  uint32_SPEEDx100_cumulated -= uint32_SPEEDx100_cumulated >> SPEEDFILTER;
  uint32_SPEEDx100_cumulated += internal_tics_to_speedx100(q31_tics_filtered >> 3);

  // low pass filter next signals
  iq_cum -= iq_cum >> 8;
  iq_cum += MS.i_q;

  id_cum -= id_cum >> 8;
  id_cum += MS.i_d;

  uq_cum -= uq_cum >> 8;
  uq_cum += MS.u_q;

  ud_cum -= ud_cum >> 8;
  ud_cum += MS.u_d;

  MS.Battery_Current = get_battery_current(iq_cum >> 8, id_cum >> 8, uq_cum >> 8, ud_cum >> 8) * sign(iq_cum) * i8_direction * i8_reverse_flag;

  // if we should startup the motor
  if (MS.i_q_setpoint && motor_pwm_is_enabled() == 0) {
    // set initial PWM values
    TIM1->CCR1 = 1023; 
    TIM1->CCR2 = 1023;
    TIM1->CCR3 = 1023;
    
    speed_PLL(0,0); // reset integral part
    uint16_half_rotation_counter = 0;
    uint16_full_rotation_counter = 0;
    __HAL_TIM_SET_COUNTER(&htim2, 0); //reset tim2 counter
    ui16_timertics = 40000; //set interval between two hallevents to a large value
    i8_recent_rotor_direction = i8_direction * i8_reverse_flag;

    if (MS.system_state == Stop) {
      speed_PLL(0,0); //reset integral part
    } else {
      PI_iq.integral_part = ((p_M365State->speed << 11) * 100000 / (ui32_KV * p_M365State->Voltage)) << PI_iq.shift;
      PI_iq.out = PI_iq.integral_part;
    }

    motor_enable_pwm();
    get_standstill_position();
    
  } else {
#ifdef KILL_ON_ZERO
                if(uint16_mapped_throttle==0&&READ_BIT(TIM1->BDTR, TIM_BDTR_MOE)){
      CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM if motor is not turning
      get_standstill_position();
                        printf_("shutdown %d\n", q31_rotorposition_absolute);
                }
#endif

    // calculate wheel speed
    if (MS.system_state == Stop || MS.system_state == SixStep) {
      p_M365State->speed = 0;
    }	else {
      p_M365State->speed = tics_to_speed(q31_tics_filtered >> 3);
    }

    // see if PWM should be disable
    if (MS.i_q_setpoint == 0 &&
        (uint16_full_rotation_counter > 7999 || uint16_half_rotation_counter > 7999) &&
        motor_pwm_is_enabled()) {
      motor_disable_pwm();
      MS.system_state = Stop;
      get_standstill_position();
    }
  }
}

/**
 * Enable DMA controller clock
 */
static void DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	// DMA channel 1: used for ADC
  /* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void ADC1_Init(void) {
	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 6;
	hadc1.Init.NbrOfDiscConversion = 0;

	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the ADC multi-mode
	 */
  ADC_MultiModeTypeDef multimode;
	multimode.Mode = ADC_DUALMODE_REGSIMULT_INJECSIMULT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Injected Channel
	 */
  ADC_InjectionConfTypeDef sConfigInjected;
	sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
	sConfigInjected.InjectedNbrOfConversion = 1;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4; // Hier bin ich nicht sicher ob Trigger out oder direkt CC4
	sConfigInjected.AutoInjectedConv = DISABLE; //muß aus sein
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	HAL_ADC_Stop(&hadc1); //ADC muß gestoppt sein, damit Triggerquelle gesetzt werden kann.
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
  ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	/**Configure Regular Channel
	 */
	sConfig.Channel = JSQR_PHASE_A >> 15;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	/**Configure Regular Channel
	 */
	sConfig.Channel = JSQR_PHASE_B >> 15;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfig.Channel = JSQR_PHASE_C >> 15;
	sConfig.Rank = ADC_REGULAR_RANK_6;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void ADC2_Init(void) {

	ADC_InjectionConfTypeDef sConfigInjected;

	/**Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE; //hier auch Scan enable?!
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Injected Channel
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
	sConfigInjected.InjectedNbrOfConversion = 1;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void TIM1_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim1.Init.Period = _T;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_OC_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; //TODO: depends on gate driver!
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = _T - 1;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 32;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void TIM2_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 128;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 64000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

static void GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure GPIO pins for motor hall sensors */
	GPIO_InitStruct.Pin = HALL_1_Pin | HALL_2_Pin | HALL_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	/*Configure peripheral I/O remapping */
	__HAL_AFIO_REMAP_PD01_ENABLE();

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim3) {

    if (uint16_full_rotation_counter < 8000) {
			uint16_full_rotation_counter++;	//full rotation counter for motor standstill detection
    }

    if (uint16_half_rotation_counter < 8000) {
			uint16_half_rotation_counter++;	//half rotation counter for motor standstill detectio
    }
  }
}

//assuming, a proper AD conversion takes 350 timer tics, to be confirmed. DT+TR+TS deadtime + noise subsiding + sample time
void dyn_adc_state(q31_t angle) {
	if (switchtime[2] > switchtime[0] && switchtime[2] > switchtime[1]) {
		MS.char_dyn_adc_state = 1; // -90° .. +30°: Phase C at high dutycycles
		if (switchtime[2] > 1500)
			TIM1->CCR4 = switchtime[2] - TRIGGER_OFFSET_ADC;
		else
			TIM1->CCR4 = TRIGGER_DEFAULT;
	}

	if (switchtime[0] > switchtime[1] && switchtime[0] > switchtime[2]) {
		MS.char_dyn_adc_state = 2; // +30° .. 150° Phase A at high dutycycles
		if (switchtime[0] > 1500)
			TIM1->CCR4 = switchtime[0] - TRIGGER_OFFSET_ADC;
		else
			TIM1->CCR4 = TRIGGER_DEFAULT;
	}

	if (switchtime[1] > switchtime[0] && switchtime[1] > switchtime[2]) {
		MS.char_dyn_adc_state = 3; // +150 .. -90° Phase B at high dutycycles
		if (switchtime[1] > 1500)
			TIM1->CCR4 = switchtime[1] - TRIGGER_OFFSET_ADC;
		else
			TIM1->CCR4 = TRIGGER_DEFAULT;
	}
}

static void set_inj_channel(char state) {
	switch (state) {
	case 1: //Phase C at high dutycycles, read current from phase A + B
		ADC1->JSQR = JSQR_PHASE_A; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
		ADC1->JOFR1 = ui16_ph1_offset;
		ADC2->JSQR = JSQR_PHASE_B; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
		ADC2->JOFR1 = ui16_ph2_offset;
		break;

	case 2: //Phase A at high dutycycles, read current from phase C + B
		ADC1->JSQR = JSQR_PHASE_C; //ADC1 injected reads phase C, JSQ4 = 0b00110, decimal 6
		ADC1->JOFR1 = ui16_ph3_offset;
		ADC2->JSQR = JSQR_PHASE_B; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
		ADC2->JOFR1 = ui16_ph2_offset;
		break;

	case 3: //Phase B at high dutycycles, read current from phase A + C
		ADC1->JSQR = JSQR_PHASE_A; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
		ADC1->JOFR1 = ui16_ph1_offset;
		ADC2->JSQR = JSQR_PHASE_C; //ADC2 injected reads phase C, JSQ4 = 0b00110, decimal 6
		ADC2->JOFR1 = ui16_ph3_offset;
		break;
	}
}

// PI Control
q31_t PI_control(PI_control_t* PI_c)
{
  q31_t q31_p; //proportional part
  q31_p = ((PI_c->setpoint - PI_c->recent_value)*PI_c->gain_p);
  PI_c->integral_part += ((PI_c->setpoint - PI_c->recent_value)*PI_c->gain_i);

  if (PI_c->integral_part > PI_c->limit_i << PI_c->shift) PI_c->integral_part = PI_c->limit_i << PI_c->shift;
  if (PI_c->integral_part < -(PI_c->limit_i << PI_c->shift)) PI_c->integral_part = -(PI_c->limit_i << PI_c->shift);
  if (motor_pwm_is_enabled() == false) PI_c->integral_part = 0 ; // reset integral part if PWM is disabled

  //avoid too big steps in one loop run
  if (q31_p+PI_c->integral_part > PI_c->out+PI_c->max_step) PI_c->out+=PI_c->max_step;
  else if  (q31_p+PI_c->integral_part < PI_c->out-PI_c->max_step)PI_c->out-=PI_c->max_step;
  else PI_c->out=(q31_p+PI_c->integral_part);

  if (PI_c->out>PI_c->limit_output << PI_c->shift) PI_c->out = PI_c->limit_output<< PI_c->shift;
  if (PI_c->out<-(PI_c->limit_output << PI_c->shift)) PI_c->out = -(PI_c->limit_output<< PI_c->shift); // allow no negative voltage.
  if (motor_pwm_is_enabled() == false) PI_c->out = 0 ; //reset output if PWM is disabled

  return (PI_c->out>>PI_c->shift);
}

void motor_runPIcontrol() {
	// PI-control processing

  //Check battery current limit
  if (MS.Battery_Current > BATTERYCURRENT_MAX)
    ui8_BC_limit_flag = 1;
  if (MS.Battery_Current < -REGEN_CURRENT_MAX)
    ui8_BC_limit_flag = 1;

  //reset battery current flag with small hysteresis
  if (MS.i_q * i8_direction * i8_reverse_flag > 100) { // motor mode
    if (get_battery_current(MS.i_q_setpoint,MS.i_d_setpoint,uq_cum>>8,ud_cum>>8)
        < (BATTERYCURRENT_MAX * 7) >> 3)
      ui8_BC_limit_flag = 0;
  } else { //generator mode
    if (get_battery_current(MS.i_q_setpoint,MS.i_d_setpoint,uq_cum>>8,ud_cum>>8)
        > (-REGEN_CURRENT_MAX * 7) >> 3) // Battery current not negative yet!!!!
      ui8_BC_limit_flag = 0;
  }

  if (!ui8_BC_limit_flag) {
    PI_iq.recent_value = MS.i_q;
    PI_iq.setpoint = MS.i_q_setpoint;
  } else {
    if(!MS.brake_active){
    // if(HAL_GPIO_ReadPin(Brake_GPIO_Port, Brake_Pin)){
      PI_iq.recent_value=  (MS.Battery_Current>>6)*i8_direction*i8_reverse_flag;
      PI_iq.setpoint = (BATTERYCURRENT_MAX>>6)*i8_direction*i8_reverse_flag;
      }
    else{
      PI_iq.recent_value=  (MS.Battery_Current>>6)*i8_direction*i8_reverse_flag;
      PI_iq.setpoint = (-REGEN_CURRENT_MAX>>6)*i8_direction*i8_reverse_flag;
      }
  }
  q31_u_q_temp = PI_control(&PI_iq);

  // control id
  PI_id.recent_value = MS.i_d;
  PI_id.setpoint = MS.i_d_setpoint;
  q31_u_d_temp = -PI_control(&PI_id); // control direct current to zero

  arm_sqrt_q31((q31_u_d_temp*q31_u_d_temp+q31_u_q_temp*q31_u_q_temp)<<1,&MS.u_abs);
  MS.u_abs = (MS.u_abs>>16)+1;

  if (MS.u_abs > _U_MAX) {
    MS.u_q = (q31_u_q_temp * _U_MAX) / MS.u_abs; // division!
    MS.u_d = (q31_u_d_temp * _U_MAX) / MS.u_abs; // division!
    MS.u_abs = _U_MAX;
  } else {
    MS.u_q = q31_u_q_temp;
    MS.u_d = q31_u_d_temp;
  }
}

/* TIM3 init function 8kHz interrupt frequency for regular adc triggering */
static void TIM3_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 7813;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);
}

void motor_init(volatile MotorStatePublic_t* motorStatePublic) {
  p_MotorStatePublic = motorStatePublic; // local pointer of MotorStatePublic

	// Virtual EEPROM init
	HAL_FLASH_Unlock();
	EE_Init();
	HAL_FLASH_Lock();

  // init IO pins
	GPIO_Init();

  // init DMA for ADC
  DMA_Init();

	// initialize MS struct.
	hall_angle_detect_flag = false;
	MS.assist_level = 1;
	MS.regen_level = 7;
	MS.i_q_setpoint = 0;
	MS.i_d_setpoint = 0;

	MS.phase_current_limit = PH_CURRENT_MAX_NORMAL;
	MS.speed_limit = SPEEDLIMIT_NORMAL;

  // init PI structs
  PI_id.gain_i = I_FACTOR_I_D;
  PI_id.gain_p = P_FACTOR_I_D;
  PI_id.setpoint = 0;
  PI_id.limit_output = _U_MAX;
  PI_id.max_step = 5000;
  PI_id.shift = 10;
  PI_id.limit_i = 1800;

  PI_iq.gain_i = I_FACTOR_I_Q;
  PI_iq.gain_p = P_FACTOR_I_Q;
  PI_iq.setpoint = 0;
  PI_iq.limit_output = _U_MAX;
  PI_iq.max_step = 5000;
  PI_iq.shift = 10;
  PI_iq.limit_i = _U_MAX;

	// ADC init and run calibration
	ADC1_Init();
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	ADC2_Init();
	if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK) {
		Error_Handler();
	}

  // enable external trigger
	SET_BIT(ADC1->CR2, ADC_CR2_JEXTTRIG);
	__HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
	SET_BIT(ADC2->CR2, ADC_CR2_JEXTTRIG);
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);

	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) p_MotorStatePublic->adcData, 6);
  HAL_ADC_Start_IT(&hadc2);

  // Timers
	TIM1_Init(); //Hier die Reihenfolge getauscht!
	TIM2_Init();
  TIM3_Init();

	// Start Timer 1
	if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // turn on complementary channel
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

	TIM1->CCR4 = TRIGGER_DEFAULT; //ADC sampling just before timer overflow (just before middle of PWM-Cycle)

	// Start Timer 2
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
		Error_Handler();
	}

  // Start Timer 3
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
		Error_Handler();
	}

	motor_disable_pwm();
	TIM1->CCR1 = 1023; //set initial PWM values
	TIM1->CCR2 = 1023;
	TIM1->CCR3 = 1023;

	HAL_Delay(1000);

  // average measured ADC phase currents, to store as the offset of each one
	for (uint32_t i = 0; i < 16; i++) {		
	  HAL_Delay(5);
		ui16_ph1_offset += p_MotorStatePublic->adcData[ADC_CHANA];
		ui16_ph2_offset += p_MotorStatePublic->adcData[ADC_CHANB];
		ui16_ph3_offset += p_MotorStatePublic->adcData[ADC_CHANC];
	}
	ui16_ph1_offset = ui16_ph1_offset >> 4;
	ui16_ph2_offset = ui16_ph2_offset >> 4;
	ui16_ph3_offset = ui16_ph3_offset >> 4;

  ADC1->JSQR = JSQR_PHASE_A; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
  ADC1->JOFR1 = ui16_ph1_offset;

  ui8_adc_offset_done_flag = 1;

  EE_ReadVariable(EEPROM_POS_HALL_ORDER, &i16_hall_order);
  // set variables to value from emulated EEPROM only if valid
  if (i16_hall_order != 0xFFFF) {
    int16_t temp;
    EE_ReadVariable(EEPROM_POS_HALL_45, &temp);
    Hall_45 = temp << 16;
    EE_ReadVariable(EEPROM_POS_HALL_51, &temp);
    Hall_51 = temp << 16;
    EE_ReadVariable(EEPROM_POS_HALL_13, &temp);
    Hall_13 = temp << 16;
    EE_ReadVariable(EEPROM_POS_HALL_32, &temp);
    Hall_32 = temp << 16;
    EE_ReadVariable(EEPROM_POS_HALL_26, &temp);
    Hall_26 = temp << 16;
    EE_ReadVariable(EEPROM_POS_HALL_64, &temp);
    Hall_64 = temp << 16;
    EE_ReadVariable(EEPROM_POS_KV, &ui32_KV);
  } else {
    motor_autodetect();
  }

	HAL_Delay(5);

  motor_disable_pwm();

	get_standstill_position();
}

// injected ADC
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//for oszi-check of used time in FOC procedere
	//HAL_GPIO_WritePin(UART1_Tx_GPIO_Port, UART1_Tx_Pin, GPIO_PIN_SET);

	if (!ui8_adc_offset_done_flag) {
		i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
		ui8_adc_inj_flag = 1;
	} else {

#ifdef DISABLE_DYNAMIC_ADC
  i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
#else
  switch (MS.char_dyn_adc_state) //read in according to state
  {
    case 1: //Phase C at high dutycycles, read from A+B directly
      raw_inj1 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
      i16_ph1_current = raw_inj1;

      raw_inj2 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
      i16_ph2_current = raw_inj2;
      break;

    case 2: //Phase A at high dutycycles, read from B+C (A = -B -C)
      raw_inj2 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
      i16_ph2_current = raw_inj2;

      raw_inj1 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
      i16_ph1_current = -i16_ph2_current - raw_inj1;
      break;

    case 3: //Phase B at high dutycycles, read from A+C (B=-A-C)
      raw_inj1 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
      i16_ph1_current = raw_inj1;
      raw_inj2 = (q31_t) HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
      i16_ph2_current = -i16_ph1_current - raw_inj2;
      break;

    case 0: //timeslot too small for ADC
      //do nothing
      break;
  } // end case
#endif

  __disable_irq(); // ENTER CRITICAL SECTION!!!!!!!!!!!!!

  // extrapolate recent rotor position
  ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last event

  if (hall_angle_detect_flag == false) {

    if (ui16_timertics < SIXSTEPTHRESHOLD && ui16_tim2_recent < 200) {
      ui8_6step_flag = 0; 
    }
    
    if (ui16_timertics > (SIXSTEPTHRESHOLD * 6) >> 2) { 
      ui8_6step_flag = 1;
    }

#ifdef SPEED_PLL
    q31_rotorposition_PLL += q31_angle_per_tic;
#endif

    if (ui16_tim2_recent < ui16_timertics + (ui16_timertics >> 2) && !ui8_overflow_flag && !ui8_6step_flag) { // prevent angle running away at standstill
#ifdef SPEED_PLL
			q31_rotorposition_absolute = q31_rotorposition_PLL;
			MS.system_state = PLL;
#else
      q31_rotorposition_absolute = q31_rotorposition_hall
         + (q31_t) (i16_hall_order * i8_recent_rotor_direction
        * ((10923 * ui16_tim2_recent) / ui16_timertics)
        << 16); //interpolate angle between two hallevents by scaling timer2 tics, 10923<<16 is 715827883 = 60�
      
      MS.system_state = Interpolation;
#endif
    } else {
      ui8_overflow_flag = 1;
      q31_rotorposition_absolute = q31_rotorposition_hall - i8_direction * 357913941; // offset of 30 degree to get the middle of the sector
      MS.system_state = SixStep;
    }
  }

  __enable_irq(); //EXIT CRITICAL SECTION!!!!!!!!!!!!!!

#ifndef DISABLE_DYNAMIC_ADC
  //get the Phase with highest duty cycle for dynamic phase current reading
  dyn_adc_state(q31_rotorposition_absolute);

  //set the according injected channels to read current at Low-Side active time
  if (MS.char_dyn_adc_state != char_dyn_adc_state_old) {
    set_inj_channel(MS.char_dyn_adc_state);
    char_dyn_adc_state_old = MS.char_dyn_adc_state;
  }
#endif

  // call FOC procedure if PWM is enabled
  if (motor_pwm_is_enabled()) {
    FOC_calculation(i16_ph1_current, i16_ph2_current, q31_rotorposition_absolute, &MS);
  }

  //set PWM
  TIM1->CCR1 = (uint16_t) switchtime[0];
  TIM1->CCR2 = (uint16_t) switchtime[1];
  TIM1->CCR3 = (uint16_t) switchtime[2];
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

