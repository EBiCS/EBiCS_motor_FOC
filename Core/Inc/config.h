/*
 * config.h
 *
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"
#define DISPLAY_TYPE_DEBUG (1<<0)							// For ASCII-Output in Debug mode);

#define TRIGGER_OFFSET_ADC 50
#define TRIGGER_DEFAULT 2020
#define _T 2028
#define CAL_BAT_V 14
#define CAL_V 25
#define CAL_I 38LL<<8
#define INDUCTANCE 6LL
#define RESISTANCE 40LL
#define FLUX_LINKAGE 1200LL
#define GAMMA 9LL
#define BATTERY_LEVEL_1 323000
#define BATTERY_LEVEL_2 329000
#define BATTERY_LEVEL_3 344000
#define BATTERY_LEVEL_4 368000
#define BATTERY_LEVEL_5 380000
//#define ADCTHROTTLE
#define THROTTLEOFFSET 45
#define THROTTLEMAX 175
#define BRAKEOFFSET 40
#define BRAKEMAX 190
#define WHEEL_CIRCUMFERENCE 550 //690 for original M365 motor
#define GEAR_RATIO 11 //15 for original M365 motor


#define FW_CURRENT_MAX 0 //max id
#define KV 10 //kph*10 per volt

#define BATTERYCURRENT_MAX 9000
#define SPEC_ANGLE -167026406L //BionX IGH3 -143165476

#define REVERSE -1

#define VOLTAGE_MIN 300
#define REGEN_CURRENT 1000


#define REGEN_CURRENT_MAX 10000

#define SPEED_PLL
#define P_FACTOR_PLL 9 //7 for original M365 motor
#define I_FACTOR_PLL 10 //7 for original M365 motor

#endif /* CONFIG_H_ */
