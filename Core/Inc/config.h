/*
 * config.h
 *
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"

#define TRIGGER_OFFSET_ADC 50
#define TRIGGER_DEFAULT 2020
#define _T 2028
#define CAL_BAT_V 14 // factor for ADC steps to mV
#define CAL_I 38 // factor for ADC steps to mA
//#define ADCTHROTTLE
#define THROTTLEOFFSET 45
#define THROTTLEMAX 175
#define BRAKEOFFSET 50
#define BRAKEMAX 190
#define WHEEL_CIRCUMFERENCE 550 //690 for original M365 motor
#define GEAR_RATIO 24 //15 for original M365 motor

#define SPEEDLIMIT_ECO 20
#define SPEEDLIMIT_NORMAL 100
#define SPEEDLIMIT_SPORT 100
#define PH_CURRENT_MAX_ECO 500
#define PH_CURRENT_MAX_NORMAL 1000
#define PH_CURRENT_MAX_SPORT 1500

#define FW_CURRENT_MAX 400 //max id
#define KV 6 //kph*10 per volt, 7 for original M365 motor

#define BATTERYCURRENT_MAX 8000
#define SPEEDFILTER 3

#define SPEC_ANGLE -167026406L

#define REVERSE -1 // -1 for original M365 motor

#define REGEN_CURRENT 1000
#define REGEN_CURRENT_MAX 10000

#endif /* CONFIG_H_ */
