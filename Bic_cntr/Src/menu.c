/*
 * menu.c
 *
 *  Created on: 8 kwi 2017
 *      Author: Fero
 */

#include "menu.h"
#include "tm_stm32_hd44780.h"

extern volatile int round_time_ms;
extern uint16_t bat_voltage;
extern uint8_t received_data[USB_COMM_BUF_SIZE];
extern uint8_t received_data_flag;
extern short int retry_count;
extern uint8_t data_to_send[USB_COMM_BUF_SIZE];
extern uint8_t message_length;
short int retry_count = 0;

extern volatile long long periods;
char battery_level[3];
char velocity_string[6];
char time_sec[3];
char time_hrs[3];
char time_min[3];
short int t_secs;
short int t_mins;
short int t_hours;
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

int eval_battery_level(){

	float batt_level_percent = (float)bat_voltage/4095.0;
	batt_level_percent*=100;
	return batt_level_percent;
}
void battery_lvl_to_char(float batt_level_percent){
	itoa(batt_level_percent, battery_level, 10);
}
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
				strcpy(velocity_string, result);
//				TM_HD44780_Puts(3,FIRST_ROW,velocity_string);

	return velocity_string;
}

inline void tick_time() {

	t_secs++;
	if (t_secs > 59) {
		t_secs = 0;
		t_mins++;
		if (t_mins > 59) {
			t_mins = 0;
			t_hours++;
			if (t_hours > 59) {
				t_hours = 0;
				//TODO: Handle days (maybe if overflowed -> dump entry into disk ?)
			}
		}
	}

}

inline void time_to_string() {
	itoa(t_secs, time_sec, 10);
	itoa(t_mins, time_min, 10);
	itoa(t_hours, time_hrs, 10);
}
/*
 * Function responsible for handling usb-communication state machine
 */
int usb_set(int *state) {

	char buf[USB_COMM_BUF_SIZE] = { 0 };

	for (int i = 0; i < USB_COMM_BUF_SIZE; i++)
		buf[i] = (char) received_data[i];

	if (*state == USB_SUBSTATE_INIT) {
		if (strstr(buf, "host")) {
			*state = USB_SUBSTATE_ACK;
			retry_count = 0;
			return USB_SUBSTATE_ACK;
		} else {
			retry_count++;
		}
	}

	if (*state == USB_SUBSTATE_ACK) {
		message_length = sprintf(data_to_send, "provide_set_param\r\n");

		CDC_Transmit_FS(data_to_send, message_length);
		TM_HD44780_Puts(0, SECOND_ROW, received_data);
		*state = USB_SUBSTATE_CONF;
		return USB_SUBSTATE_CONF;
	}

	if (*state == USB_SUBSTATE_CONF) {
		if (strstr(buf, "set")) {
			message_length = sprintf(data_to_send, "set_param_conf\n\r");

			CDC_Transmit_FS(data_to_send, USB_COMM_BUF_SIZE);
			TM_HD44780_Puts(0, SECOND_ROW, received_data);
			Delayms(1000);
			*state = USB_SUBSTATE_FINISHED;
			return USB_SUBSTATE_FINISHED;
		} else {
			retry_count++;
		}
	}
	return USB_STATEM_FAILURE;
}
