/*
 * menu.c
 *
 *  Created on: 8 kwi 2017
 *      Author: Fero
 */

#include "menu.h"
#include "tm_stm32_hd44780.h"
extern volatile int round_time_ms;

int draw_state_lcd(menu_state *ms) {
	TM_HD44780_Clear();
	for (int i = 0; i < MAX_NR_PATTERNS; i++) {
		TM_HD44780_Puts(ms->patterns[i].row_pos, ms->patterns[i].col_pos,
				ms->patterns[i].text);
	}

	return 0;
}

char * eval_velocity(){
	char result [16] = {0};
	//			TM_HD44780_Puts(0,0,"CONTACTRON !!!");
				float radius = 0.5/1000;
				float time_s = round_time_ms/1000.0;
				float velocity = 2*3.14* radius/(time_s/(3600));
				char buf2[4];
				char buf[16];
				int decimal_part = ((int)(velocity*100.0)) % 100;
				itoa((int)velocity,buf,10);
				itoa(decimal_part,buf2,10);

				strncpy(result,buf,2);
				strncat(result,",",1);
				strncat(result,buf2,2);
//				strncat(result," km/h",5);
				TM_HD44780_Puts(0,1,result);
				Delayms(100);
				TM_HD44780_Clear();
	return result;
}
