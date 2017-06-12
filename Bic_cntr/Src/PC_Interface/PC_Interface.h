/*
 * PC_Interface.h
 *
 *  Created on: 12 cze 2017
 *      Author: Fero
 */

#ifndef PC_INTERFACE_PC_INTERFACE_H_
#define PC_INTERFACE_PC_INTERFACE_H_

#define USB_COMM_BUF_SIZE 50

typedef struct PC_Interface *p_PCIntPtr;

typedef enum USB_SUBSTATE{
	USB_SUBSTATE_INIT,
	USB_SUBSTATE_ACK,
	USB_SUBSTATE_CONF,
	USB_SUBSTATE_FINISHED,
	USB_SUBSTATE_ERROR
}USB_SUBSTATE;

/**
 * Module responsible for handling basic parameters,
 * such as : time, distance, velocity
 */
typedef struct PC_Interface{
	USB_SUBSTATE usbState;

	uint8_t receivedDataFlag;
	uint8_t receivedData[USB_COMM_BUF_SIZE];
	uint8_t dataToSend[USB_COMM_BUF_SIZE];
	char messageBuffer[USB_COMM_BUF_SIZE];

	void (*battlvlToStr)(p_PCIntPtr);
	void (*evalVelocity)(p_PCIntPtr);
	void (*aggrParams)(p_PCIntPtr);
	void (*timeToStr) (p_PCIntPtr);
	void (*evalDist) (p_PCIntPtr);
	void (*tickTime) (p_PCIntPtr);
	void (*resetBasParams) (p_PCIntPtr);
} PC_Interface;


#endif /* PC_INTERFACE_PC_INTERFACE_H_ */
