/*
 * lcd_pattern.h
 *
 *  Created on: 8 kwi 2017
 *      Author: Fero
 */

#ifndef LCD_PATTERN_H_
#define LCD_PATTERN_H_
#include "string.h"

#define FIRST_ROW 0
#define SECOND_ROW 1


typedef struct lcd_pattern{
short int row_pos;
short int col_pos;
char* text;
}lcd_pattern;


#endif /* LCD_PATTERN_H_ */
