/*
 * m365
 *
 * Copyright (c) 2021 Francois Deslandes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "button_processing.h"
#include "main.h"
#include "config.h"
#include "motor.h"

volatile uint32_t main_loop_counter;
static uint8_t power_button_state = 0;

uint8_t buttonState() {
    static const uint32_t DEBOUNCE_MILLIS = 20 ;
    bool buttonstate = HAL_GPIO_ReadPin( PWR_BTN_GPIO_Port, PWR_BTN_Pin ) == GPIO_PIN_SET ;
    uint32_t buttonstate_ts = HAL_GetTick() ;

    uint32_t now = HAL_GetTick() ;
    if( now - buttonstate_ts > DEBOUNCE_MILLIS )
    {
        if( buttonstate != (HAL_GPIO_ReadPin( PWR_BTN_GPIO_Port, PWR_BTN_Pin ) == GPIO_PIN_SET))
        {
            buttonstate = !buttonstate ;
            buttonstate_ts = now ;
        }
    }
    return buttonstate ;
}

eButtonEvent getButtonEvent()
{
    static const uint32_t DOUBLE_GAP_MILLIS_MAX 	= 250;
    static const uint32_t SINGLE_PRESS_MILLIS_MAX 	= 300;
    static const uint32_t LONG_PRESS_MILLIS_MAX 	= 5000;

    static uint32_t button_down_ts = 0 ;
    static uint32_t button_up_ts = 0 ;
    static bool double_pending = false ;
    static bool button_down = false ; ;

    eButtonEvent button_event = NO_PRESS ;
    uint32_t now = HAL_GetTick() ;

    if( button_down != buttonState() ) {
        button_down = !button_down ;
        if( button_down ) {
            button_down_ts = now ;
        } else {
            button_up_ts = now ;
            if( double_pending ) {
                button_event = DOUBLE_PRESS ;
                double_pending = false ;
            }
            else {
                double_pending = true ;
            }
        }
    }

    uint32_t diff =  button_up_ts - button_down_ts;
    if (!button_down && double_pending && now - button_up_ts > DOUBLE_GAP_MILLIS_MAX) {
    	double_pending = false ;
    	button_event = SINGLE_PRESS ;
	} else if (!button_down && double_pending && diff >= SINGLE_PRESS_MILLIS_MAX && diff <= LONG_PRESS_MILLIS_MAX) {
		double_pending = false ;
		button_event = LONG_PRESS ;
	} else if (button_down && now - button_down_ts > LONG_PRESS_MILLIS_MAX) {
		double_pending = false ;
		button_event = VERY_LONG_PRESS ;
	}

    return button_event ;
}

void checkButton(M365State_t *M365State) {

  // check the shutdown counter
  if (M365State->shutdown > 16) power_control(DEV_PWR_OFF);
  
  if (main_loop_counter > 25) {
    switch (getButtonEvent()) {
        case NO_PRESS:
          break;

        case SINGLE_PRESS:
          M365State->light = !M365State->light;
          break;
        
        case VERY_LONG_PRESS:
          motor_autodetect();
          break;
        
        case LONG_PRESS:
          M365State->beep = 1;
          
          if (M365State->shutdown == 0)
            M365State->shutdown = 1;
          
          break;

        case DOUBLE_PRESS:
          M365State->mode=M365State->mode+2;
          
          if(M365State->mode>4)M365State->mode=0;
            set_mode(M365State);
        
          break;
      }
  }
  main_loop_counter++;
}

void PWR_init() {
	/* Check button pressed state at startup */
	power_button_state = buttonState();

    /* Power ON board temporarily, ultimate decision to keep hardware ON or OFF is made later */
	power_control(DEV_PWR_ON);

	}

void power_control(uint8_t pwr)
{
	if(pwr == DEV_PWR_ON) {
		/* Turn the PowerON line high to keep the board powered on even when
		 * the power button is released
		 */
		HAL_GPIO_WritePin(TPS_ENA_GPIO_Port, TPS_ENA_Pin, GPIO_PIN_SET);
	} else if(pwr == DEV_PWR_OFF) {

		//motors_free(0, NULL);
		//sleep_x_ticks(2000);
		//stop_motors();


		while(HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin));
		HAL_GPIO_WritePin(TPS_ENA_GPIO_Port, TPS_ENA_Pin, GPIO_PIN_RESET);
		while(1);
	} else if(pwr == DEV_PWR_RESTART) {

		//motors_free(0, NULL);
		//sleep_x_ticks(2000);
		//stop_motors();

		/* Restart the system */
		NVIC_SystemReset();
	}
}

void set_mode(M365State_t *M365State){

	switch(M365State->mode){
		case eco:
			M365State->phase_current_limit=PH_CURRENT_MAX_ECO;
			M365State->speed_limit=SPEEDLIMIT_ECO;
			break;

		case normal:
			M365State->phase_current_limit=PH_CURRENT_MAX_NORMAL;
			M365State->speed_limit=SPEEDLIMIT_NORMAL;
      break;

		case sport:
			M365State->phase_current_limit=PH_CURRENT_MAX_SPORT;
			M365State->speed_limit=SPEEDLIMIT_SPORT;
      break;
	}
	calculate_tic_limits(M365State->speed_limit);
}


