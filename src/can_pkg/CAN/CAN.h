/*
 * CANBus.h
 *
 *  Created on: Feb 26, 2024
 *      Author: Najib
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <asm/termbits.h> /* struct termios2 */
#include <time.h>
#include <ctype.h>
#include <signal.h>
#include <sys/time.h>

#include "stdint.h"
#include "PrjTypedef.h"

/*APIs....................................................*/
void CANInit(void);
int inject_data_frame(const char *hex_id, const char *hex_data , int data_len);
int frame_recv(unsigned char *frame);
// void CANMessageParser(unsigned char *Data , int len);


typedef struct {
	uint8_t CANMessageIDType;
	uint32_t CANMessgeID;
	uint8_t CANMessageType;
	uint8_t CANMessageLength;
	uint8_t CANMessageData[8];
} CANMessage;

STATUS_TypeDef SendCANMessage(CANMessage Message);

#endif /* INC_CANBUS_H_ */
