/*
 * menu.c
 *
 *  Created on: 8 kwi 2017
 *      Author: Fero
 */

#include "menu.h"
#include "tm_stm32_hd44780.h"

int draw_state_lcd(menu_state *ms) {
	TM_HD44780_Clear();
	//lcd_pattern * pttrn = &(ms.patterns);
	for (int i = 0; i < MAX_NR_PATTERNS; i++) {
		TM_HD44780_Puts(ms->patterns[i].row_pos, ms->patterns[i].col_pos,
				ms->patterns[i].text);
	}

	return 0;
}
