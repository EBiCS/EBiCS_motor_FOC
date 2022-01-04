/*
 * FOC.h
 *
 *  Created on: 25.01.2019
 *      Author: stancecoke
 */

#ifndef FOC_H_
#define FOC_H_

#include <arm_math.h>
#include "motor.h"
#include "config.h"
//exportetd functions
void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta,  MotorState_t* MS_FOC);
q31_t PI_control_i_q (q31_t ist, q31_t soll);
q31_t PI_control_i_d (q31_t ist, q31_t soll, q31_t clamp);

// Maximum Voltage applyed

#ifdef DISABLE_DYNAMIC_ADC
#define _U_MAX	2000L  //little lower than period of timer1 for proper phase current reading. Could be improved by dynamic timing of AD-conversion
#else
#define _U_MAX	2000L
#endif



// Square Root of 3
#define _SQRT3	28  //1.73205081*16

#define ADC_DUR 250 //minimal duration for proper ADC reading deadtime + noise subsiding + sample time

#endif /* FOC_H_ */
