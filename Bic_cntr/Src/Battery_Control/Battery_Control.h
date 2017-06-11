/*
 * Battery_Control.h
 *
 *  Created on: 11 cze 2017
 *      Author: Fero
 */

#ifndef BATTERY_CONTROL_BATTERY_CONTROL_H_
#define BATTERY_CONTROL_BATTERY_CONTROL_H_

#define BATT_CRITICAL_LEVEL 35

typedef struct Battery_Control *p_BattCNTRL;

int eval_battery_level(p_BattCNTRL);
void battery_lvl_to_char(p_BattCNTRL);

/**
 * Module responsible for Battery state control
 */
typedef struct Battery_Control {
	short int low_level_threshold;
	uint16_t bat_voltage;
	char str_battery_level[3];
	void (*battlvlToStr)(p_BattCNTRL);
	int (*evalBattLevel)(p_BattCNTRL);
} Battery_Control;

#endif /* BATTERY_CONTROL_BATTERY_CONTROL_H_ */
