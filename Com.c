/*
 * Com.c
 *
 *  Created on: Sep 8, 2016
 *      Author: mono
 */
#include "Com.h"
#include "RP6uart.h"
#include "RP6RobotBaseLib.h"
#include "DeadReckoning.h"
#include <string.h>


volatile uint8_t Command = 0;
volatile uint8_t uart_command = 0;
volatile uint8_t waitCommand = 1;
volatile uint8_t getCommand = 2;
volatile uint8_t uartState = 1;
volatile uint8_t waitParam = 3;
volatile uint16_t param;


uint16_t Sum(uint8_t *data, int lenght) {
	uint16_t crc = 0;
	for(int i = 0; i < lenght; i++) {
		crc += (uint16_t)data[i];
	}
	return crc;
}

void SendEncoder(void) {
	uint16_t crc;
	uint8_t sendBuf[6];
    sendBuf[0] = 	 (uint8_t)(mleft_abs_dist);
    sendBuf[1] = 	 (uint8_t)(mleft_abs_dist>>8);
    sendBuf[2] = 	 (uint8_t)(mright_abs_dist);
    sendBuf[3] = 	 (uint8_t)(mright_abs_dist>>8);
    crc = Sum(&sendBuf[0],4);
    sendBuf[4] = (uint8_t)(crc);
    sendBuf[5] = (uint8_t)(crc>>8);
    for(int i = 0; i < 6; i++) {
    	writeChar((char)sendBuf[i]);
    }
}

void SendPose(void) {
	uint16_t crc;
	uint8_t sendBuf[14];
    uint8_t *x_array;
    uint8_t *y_array;
    uint8_t *theta_array;
    uint8_t *crc_array = (uint8_t*)(&crc);
    x_array = (uint8_t*)(&x);
    y_array = (uint8_t*)(&y);
    theta_array = (uint8_t*)(&theta);
    sendBuf[0] = x_array[0];
    sendBuf[1] = x_array[1];
    sendBuf[2] = x_array[2];
    sendBuf[3] = x_array[3];
    sendBuf[4] = y_array[0];
    sendBuf[5] = y_array[1];
    sendBuf[6] = y_array[2];
    sendBuf[7] = y_array[3];
    sendBuf[8] = theta_array[0];
    sendBuf[9] = theta_array[1];
    sendBuf[10] = theta_array[2];
    sendBuf[11] = theta_array[3];
    crc = Sum(&sendBuf[0],12);
    sendBuf[12] = crc_array[0];
    sendBuf[13] = crc_array[1];
    for(int i = 0; i < 14; i++) {
    	writeChar((char)sendBuf[i]);
    }
    if( (permission.move == 1) || (permission.rotate_left == 1) || (permission.rotate_right == 1)) {
    	writeChar('m');
    }
    else {
    	writeChar('s');
    }
}

void CheckBufSize(void) {
	if(getBufferLength() > 0) {
		uartState = getCommand;
	}
}

void ComErrorDetectio(void) {
	if((getBufferLength() > 5) || ((uartState != 3) && (getBufferLength() > 1)) )  {
		uartState = getCommand;
		clearReceptionBuffer();
	}
}

void CheckCommand(void) {
	char c;
	char ack = 'a';
	char notAck[2];
	c = readChar();
	clearReceptionBuffer();
	uart_command = (uint8_t)c;
	if(uart_command < 4 && uart_command > 0 ) {
		uartState = waitParam;
		ack = (uint8_t)c;
		writeChar(ack);
		return;
	}
	else if( uart_command == 8){
		uartState = waitCommand;
		Command = uart_command;
		uart_command = 0;
		return;
	}
	else if(uart_command == 5){
		uartState = waitCommand;
		Command = uart_command;
		uart_command = 0;
		return;
	}
	else if(uart_command == 6){
		writeChar(ack);
		uartState = waitCommand;
		Command = uart_command;
		uart_command = 0;
		return;
	}
	notAck[0] = 'C';
	notAck[1] = 'C';
	writeStringLength(&notAck[0],2,0);
	uartState = waitCommand;
}

void GetParam(void) {
	char ack = 'a';
	char notAck[2];
	uint8_t param_crc[4];
	uint8_t * temp_param;
	uint8_t * temp_crc;
	uint16_t crc;
	uint16_t calc_crc;
	temp_crc = &crc;
	temp_param = &param;
	if(getBufferLength() >= 4) {
		readChars((char*)&param_crc,4);
		clearReceptionBuffer();
		memcpy(temp_param,&param_crc[2],2);
		memcpy(temp_crc,&param_crc[0],2);
		calc_crc = (uint16_t)param_crc[2] + (uint16_t)param_crc[3];
		uint8_t s_crc = (uint8_t)calc_crc;
		if(calc_crc != crc) {
			notAck[0] = uartState;//(char)param_crc[2];
			notAck[1] = 'S';//(char)param_crc[3];
			clearReceptionBuffer();
			writeStringLength(&notAck[0],2,0);
			return;
		}
		else {
			ack = param_crc[2];
			clearReceptionBuffer();
			writeChar(ack);
			Command = uart_command;
			uart_command = 0;
			uartState = waitCommand;
			return;
		}
	}
}

void UartCom(void) {
	switch(uartState) {
	case 1: CheckBufSize(); break;
	case 2: CheckCommand(); break;
	case 3: GetParam(); break;
	}
}



