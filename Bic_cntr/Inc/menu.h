/*
 * menu.h
 *
 *  Created on: 8 kwi 2017
 *      Author: Fero
 */

#ifndef MENU_H_
#define MENU_H_
#include "lcd_pattern.h"
#include "common_defs.h"

#define MAIN_MENU 0	//default state - presents actual velocity
#define STAT_MENU 1	//stored overall average results
#define STAT_MENU2 2	//stored overall average results
#define USB_CONF_MENU 3 //actual configuration (radius, soft version, etc.)
#define BATT_LEVEL 4



#define MENU_SIZE 5

#define MAX_NR_PATTERNS 10

/***
 * Struct responsible for storing menu state.
 * State is used to render properly given values
 * in appropriate pattern.
 */
typedef struct menu_state {
	short int state;
	lcd_pattern patterns[MAX_NR_PATTERNS];
	short int substate;
} menu_state;

void tick_time();
extern void time_to_string();
extern char * eval_velocity();
int draw_state_lcd(menu_state *ms);

#endif /* MENU_H_ */
