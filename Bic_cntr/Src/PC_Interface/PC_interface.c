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
#include "usbd_cdc_if.h"

PC_Interface usbInterfaceMod = {
		.receivedData = {0},
		.dataToSend = {0},
		.messageBuffer = {0},
		.receivedDataFlag = 0,
		.runStateMachine = state_machine
};

/**
 * Wrapping function displaying string in begginning of second row
 * @param str - string to put on display
 */
void lcdPutStrSecRow(char* str) {
	TM_HD44780_Puts(0, SECOND_ROW, str);
	Delayms(600);
}

/**
 * Function responsible for converting data from USB buffer into string
 */
void copy_data_to_string(p_PCIntPtr PCIptr) {
	for (int i = 0; i < USB_COMM_BUF_SIZE; i++)
		PCIptr->messageBuffer[i] = (char) PCIptr->receivedData[i];
}

/**
 * Function responsible for handling usb-communication state machine
 * @param PCIptr - interface to PC interface module
 */
int state_machine(p_PCIntPtr PCIptr) {
	int res = USBD_FAIL;
	int message_length = 0;

	switch (PCIptr->usbState) {
	case USB_SUBSTATE_INIT:
		lcdPutStrSecRow((char*)PCIptr->receivedData);

		if (strstr((char*)PCIptr->receivedData, "hello_host")) {
			PCIptr->usbState = USB_SUBSTATE_ACK;
			return USB_SUBSTATE_ACK;
		} else {
			lcdPutStrSecRow("Init state!");
			PCIptr->usbState = USB_SUBSTATE_INIT;
		}
		break;

	case USB_SUBSTATE_ACK:
		lcdPutStrSecRow((char*)PCIptr->dataToSend);

		message_length = sprintf((char*)PCIptr->dataToSend, "provide_set_param\r\n");
		res = CDC_Transmit_FS(PCIptr->dataToSend, message_length);

		if (res != USBD_OK) {
			lcdPutStrSecRow("Couldn't send message!");
			PCIptr->usbState = USB_SUBSTATE_ERROR;
		} else {
			lcdPutStrSecRow("Nxt-> CONF state!");
			PCIptr->usbState = USB_SUBSTATE_CONF;
		}
		break;

	case USB_SUBSTATE_CONF:
		if (strstr((char*)PCIptr->receivedData, "param")) {
			PCIptr->usbState = USB_SUBSTATE_FINISHED;
		} else {
			lcdPutStrSecRow("Unrecog. par. sent!");
			PCIptr->usbState = USB_SUBSTATE_ERROR;
		}
		break;

	case USB_SUBSTATE_FINISHED:
		message_length = sprintf((char*)PCIptr->dataToSend, "configure done!\r\n");
		res = CDC_Transmit_FS(PCIptr->dataToSend, message_length);

		if (res != USBD_OK) {
			lcdPutStrSecRow("Couldn't send message!");
			PCIptr->usbState = USB_SUBSTATE_ERROR;
		} else {
			lcdPutStrSecRow("Nxt-> INIT state!");
			PCIptr->usbState = USB_SUBSTATE_INIT;
		}
		break;

	case USB_SUBSTATE_ERROR:
		lcdPutStrSecRow("Error!!!");
		lcdPutStrSecRow("Nxt -> Init state");

		PCIptr->usbState = USB_SUBSTATE_INIT;
		break;
	default:
		lcdPutStrSecRow("Unknown state!");
		PCIptr->usbState = USB_SUBSTATE_ERROR;
		break;
	}
	return PCIptr->usbState;
}

