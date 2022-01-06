/*
 * FOC.c
 *
 *  Created on: 25.01.2019
 *      Author: Stancecoke
 */
#include "main.h"
#include "config.h"
#include "FOC.h"
#include "stm32f1xx_hal.h"
#include <arm_math.h>
#include <stdlib.h>

q31_t q31_i_q_fil = 0;
q31_t q31_i_d_fil = 0;

void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta) {

//SVPWM according to chapter 4.9 of UM1052

	q31_t q31_U_alpha = (_SQRT3 * _T * q31_u_alpha) >> 4;
	q31_t q31_U_beta = -_T * q31_u_beta;
	q31_t X = q31_U_beta;
	q31_t Y = (q31_U_alpha + q31_U_beta) >> 1;
	q31_t Z = (q31_U_beta - q31_U_alpha) >> 1;

	//Sector 1 & 4
	if ((Y >= 0 && Z < 0 && X > 0) || (Y < 0 && Z >= 0 && X <= 0)) {
		switchtime[0] = ((_T + X - Z) >> 12) + (_T >> 1); // right shift 11 for dividing by peroid (=2^11), right shift 1 for dividing by 2
		switchtime[1] = switchtime[0] + (Z >> 11);
		switchtime[2] = switchtime[1] - (X >> 11);
	}

	//Sector 2 & 5
	if ((Y >= 0 && Z >= 0) || (Y < 0 && Z < 0)) {
		switchtime[0] = ((_T + Y - Z) >> 12) + (_T >> 1);
		switchtime[1] = switchtime[0] + (Z >> 11);
		switchtime[2] = switchtime[0] - (Y >> 11);
	}

	//Sector 3 & 6
	if ((Y < 0 && Z >= 0 && X > 0) || (Y >= 0 && Z < 0 && X <= 0)) {
		switchtime[0] = ((_T + Y - X) >> 12) + (_T >> 1);
		switchtime[2] = switchtime[0] - (Y >> 11);
		switchtime[1] = switchtime[2] + (X >> 11);
	}
}

void FOC_calculation(int16_t int16_i_as, int16_t int16_i_bs, q31_t q31_teta, MotorState_t *MS) {

  q31_t q31_i_alpha = 0;
	q31_t q31_i_beta = 0;
	q31_t q31_u_alpha = 0;
	q31_t q31_u_beta = 0;
	q31_t q31_i_d = 0;
	q31_t q31_i_q = 0;

	q31_t sinevalue = 0, cosinevalue = 0;

		// Clark transformation
	arm_clarke_q31((q31_t) int16_i_as, (q31_t) int16_i_bs, &q31_i_alpha, &q31_i_beta);

	arm_sin_cos_q31(q31_teta, &sinevalue, &cosinevalue);
  // limit output valyes of arm_sin_cos_q31() as some seem wrong
	if (sinevalue == -2147483648) sinevalue = -2147483647;
	if (cosinevalue == 2147483648) cosinevalue = 2147483647;

	// Park transformation
	arm_park_q31(q31_i_alpha, q31_i_beta, &q31_i_d, &q31_i_q, sinevalue, cosinevalue);

	q31_i_q_fil -= q31_i_q_fil >> 4;
	q31_i_q_fil += q31_i_q;
	MS->i_q = q31_i_q_fil >> 4;

	q31_i_d_fil -= q31_i_d_fil >> 4;
	q31_i_d_fil += q31_i_d;
	MS->i_d = q31_i_d_fil >> 4;

	// Control iq
  
  //set static volatage for hall angle detection
	if (MS->KV_detect_flag) {  
    MS->u_q = MS->KV_detect_flag;
    MS->u_d = 0;
  } else {
    motor_runPIcontrol();
  }

	// inverse Park transformation
	arm_inv_park_q31(MS->u_d, MS->u_q, &q31_u_alpha, &q31_u_beta, -sinevalue, cosinevalue);

	// call SVPWM calculation
	svpwm(q31_u_alpha, q31_u_beta);
}

