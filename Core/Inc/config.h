/*
 * config.h
 *
 *  Automatically created by Lishui Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include "stdint.h"
#define DISPLAY_TYPE_M365DASHBOARD (1<<1)
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
#define P_FACTOR_I_Q 100
#define I_FACTOR_I_Q 2
#define P_FACTOR_I_D 1
#define I_FACTOR_I_D 1
#define MAX_D_FACTOR 1
//#define ADCTHROTTLE
#define THROTTLEOFFSET 45
#define THROTTLEMAX 175
#define BRAKEOFFSET 40
#define BRAKEMAX 190
#define WHEEL_CIRCUMFERENCE 550
#define GEAR_RATIO 22

#define SPEEDLIMIT_ECO 20
#define SPEEDLIMIT_NORMAL 25
#define SPEEDLIMIT_SPORT 50
#define PH_CURRENT_MAX_ECO 500
#define PH_CURRENT_MAX_NORMAL 1000
#define PH_CURRENT_MAX_SPORT 1500

#define FW_CURRENT_MAX 0 //max id
#define KV 10 //kph*10 per volt

#define BATTERYCURRENT_MAX 9000
#define SPEC_ANGLE -167026406L //BionX IGH3 -143165476

#define DISPLAY_TYPE DISPLAY_TYPE_M365DASHBOARD
#define REVERSE -1

#define VOLTAGE_MIN 300
#define REGEN_CURRENT 1000
//#define FAST_LOOP_LOG
//#define DISABLE_DYNAMIC_ADC

#define REGEN_CURRENT_MAX 10000

//#define SPEED_PLL
#define P_FACTOR_PLL 7
#define I_FACTOR_PLL 7

#endif /* CONFIG_H_ */
