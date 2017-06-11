/*
 * Battery_Control.c
 *
 *  Created on: 11 cze 2017
 *      Author: Fero
 */

#include "Battery_Control.h"

Battery_Control BattControl = { .low_level_threshold = BATT_CRITICAL_LEVEL,
		.bat_voltage = 0,
		.battlvlToStr = battery_lvl_to_char,
		.evalBattLevel = eval_battery_level };
/**
 * Function responsible for converting raw ADC value for battery
 * into percentage format
 * @param battCntrlPtr - interface to Battery_Control module
 */
int eval_battery_level(p_BattCNTRL battCntrlPtr) {

	float batt_level_percent = (float) battCntrlPtr->bat_voltage / 4095.0;
	batt_level_percent *= 100;
	return batt_level_percent;
}

/**
 * Function responsible for converting value of battery
 * into string
 * @param battCntrlPtr - interface to Battery_Control module
 */
void battery_lvl_to_char(p_BattCNTRL battCntrlPtr) {
	itoa(battCntrlPtr->evalBattLevel(battCntrlPtr),
			battCntrlPtr->str_battery_level, 10);
}
