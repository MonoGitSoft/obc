/*
 * Com.h
 *
 *  Created on: Sep 8, 2016
 *      Author: mono
 */

#ifndef COM_H_
#define COM_H_
#include <stdint.h>

extern volatile uint8_t Command;
extern volatile uint16_t param;

void UartCom(void);
void SendEncoder(void);
void SendPose(void);
void ComErrorDetectio(void);

#endif /* COM_H_ */
