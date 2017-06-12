/*
 * PC_interface.c
 *
 *  Created on: 12 cze 2017
 *      Author: Fero
 */

#include "PC_Interface.h"
#include "lcd_pattern.h"
#include "tm_stm32_hd44780.h"
#include "usbd_def.h"

PC_Interface usbInterfaceMod = {

};

void copy_data_to_string(p_PCIntPtr PCIptr){
	for (int i = 0; i < USB_COMM_BUF_SIZE; i++)
		PCIptr->messageBuffer[i] = (char) PCIptr->receivedData[i];
}

/*
 * Function responsible for handling usb-communication state machine
 */
int state_machine(p_PCIntPtr PCIptr) {
	int res = USBD_FAIL;
	int message_length = 0;

	switch(PCIptr->usbState){
	case USB_SUBSTATE_INIT:
		TM_HD44780_Puts(0, SECOND_ROW, PCIptr->receivedData);

		if (strstr(PCIptr->messageBuffer, "hello_host")) {
			PCIptr->usbState = USB_SUBSTATE_ACK;
			return USB_SUBSTATE_ACK;
		} else {
			TM_HD44780_Puts(0, SECOND_ROW, "Init state!");
		}
		break;
	case USB_SUBSTATE_ACK:
		TM_HD44780_Puts(0, SECOND_ROW, PCIptr->dataToSend);

		message_length = sprintf(PCIptr->dataToSend, "provide_set_param\r\n");
		res = CDC_Transmit_FS(PCIptr->dataToSend, message_length);

		if (res != USBD_OK){
			TM_HD44780_Puts(0, SECOND_ROW, "Couldn't send message!");
		}else {
			TM_HD44780_Puts(0, SECOND_ROW, "Proceeding to CONF state!");
			PCIptr->usbState = USB_SUBSTATE_CONF;
			return PCIptr->usbState;
		}
		break;
	case USB_SUBSTATE_CONF:
		if (strstr(PCIptr->messageBuffer, "param")) {
			PCIptr->usbState = USB_SUBSTATE_FINISHED;
			return USB_SUBSTATE_FINISHED;
		} else {
			TM_HD44780_Puts(0, SECOND_ROW, "Unrecognized param sent!");
		}
		break;
	case USB_SUBSTATE_FINISHED:
		message_length = sprintf(PCIptr->dataToSend, "configured done!\r\n");
		res = CDC_Transmit_FS(PCIptr->dataToSend, message_length);

		if (res != USBD_OK){
					TM_HD44780_Puts(0, SECOND_ROW, "Couldn't send message!");
				}else {
					TM_HD44780_Puts(0, SECOND_ROW, "Proceeding to INIT state!");
					PCIptr->usbState = USB_SUBSTATE_INIT;
					return USB_SUBSTATE_INIT;
				}
		break;
	default :
		TM_HD44780_Puts(0, SECOND_ROW, "Unknown state!");
		return USB_SUBSTATE_ERROR;
		break;
	}

