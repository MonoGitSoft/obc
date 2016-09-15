
#include "RP6RobotBaseLib.h" 	// The RP6 Robot Base Library.
#include "RP6I2CslaveTWI.h"	// Always needs to be included!
#include <math.h>
#include <stdlib.h>
#include <util/atomic.h>
#include "Control.h"
#include "Com.h"
#include "DeadReckoning.h"
#include "RobotParam.h"



void bumpersStateChanged(void)
{
	if(bumper_left || bumper_right)
	{
		moveAtSpeed(0,0);  // stop moving!
		setLEDs(0b010000);
		startStopwatch1();
	}
}

/**
 * This function checks Stopwatch1 all the time. If stopwatch 1 is
 * not running, the function does not do anything. As soon as the
 * stopwatch is started, two LEDs begin to blink!
 */
void blink(void)
{
	if(getStopwatch1() > 500) // 500ms
	{
		statusLEDs.LED2 = !statusLEDs.LED2; // Toggle LED bit in LED shadow register...
		statusLEDs.LED5 = !statusLEDs.LED5;
		updateStatusLEDs();
		setStopwatch1(0);
	}
}



void command_processor(void)
{
    if( Command != 0 )
    {
        switch( Command )
        {
            case MOVE_FORWARD: Set_Distance(((float)param/RES_R));permission.move = 1; stand = 0;break; //set dist,add permission to move
            case ROTATE_LEFT: Rotate_Left((uint16_t)(((float)param*(PI/180)*(DIAM))/(RES_R))) ;stand = 0;permission.rotate_left = 1; break;
            case ROTATE_RIGHT: Rotate_Right((uint16_t)(((float)param*(PI/180)*(DIAM))/(RES_R)));  stand = 0;permission.rotate_right = 1;break;
            case SEND_POSE: SendPose();break;
            case SEND_ENCODER: SendEncoder();break;
            case STREAM: permission.stream = 1;
        }
        Command = 0;
    }
}

/*void updateReadReg(void)
{
    if(!I2CTWI_readBusy)
        {
            uint8_t *x_array;
            uint8_t *y_array;
            uint8_t *theta_array;
            x_array = (uint8_t*)(&x);
            y_array = (uint8_t*)(&y);
            theta_array = (uint8_t*)(&theta);
            sendBuf[I2C_REG_DIST_LEFT_L] = 	 (uint8_t)(mleft_abs_dist);
            sendBuf[I2C_REG_DIST_LEFT_H] = 	 (uint8_t)(mleft_abs_dist>>8);
            sendBuf[I2C_REG_DIST_RIGHT_L] = 	 (uint8_t)(mright_abs_dist);
            sendBuf[I2C_REG_DIST_RIGHT_H] = 	 (uint8_t)(mright_abs_dist>>8);
            sendBuf[I2C_REG_X] = x_array[0];
            sendBuf[I2C_REG_X + 1] = x_array[1];
            sendBuf[I2C_REG_X + 2] = x_array[2];
            sendBuf[I2C_REG_X + 3] = x_array[3];
            sendBuf[I2C_REG_Y] = y_array[0];
            sendBuf[I2C_REG_Y + 1] = y_array[1];
            sendBuf[I2C_REG_Y + 2] = y_array[2];
            sendBuf[I2C_REG_Y + 3] = y_array[3];
            sendBuf[I2C_REG_THETA] = theta_array[0];
            sendBuf[I2C_REG_THETA + 1] = theta_array[1];
            sendBuf[I2C_REG_THETA + 2] = theta_array[2];
            sendBuf[I2C_REG_THETA + 3] = theta_array[3];
           // sendBuf[I2C_REG_MAX_BEAV_L] = 	 (uint8_t)(max_beav_jel);
            //sendBuf[I2C_REG_MAX_BEAV_H] = 	 (uint8_t)(max_beav_jel>>8);
            //sendBuf[I2C_REG_SPEED_L] = (uint8_t)(getLeftSpeed());
            //sendBuf[I2C_REG_SPEED_H] = (uint8_t)(mleft_speed>>8);
        }
}*/


void task_manage(void)
{
	UartCom();
    command_processor();
    task_ADC();
    Control();
    dead_reckoning();
    ComErrorDetectio();
}


int main(void)
{
	initRobotBase();
	/*Start_Control_Timer();
	Set_Distance(0);
	permission.move = 0;*/
	// Set Bumpers state changed event handler:
	BUMPERS_setStateChangedHandler(bumpersStateChanged);
	powerON(); 	// Turn Encoders, Motor Current Sensors
				// (and ACS IR Receiver and PWRON LED) on.
    //set_dist(1000);
// ATTENTION: Automatic Motor controll will not work without this!// enyit menyél elöre
    //set_PI_regulator(AP_DEFAULT, 0.3); // optimum keresésnél 0.1 -> 0.2 KELL IRNI!!!!!! Optimlizááálssá
	// Main loop:t
   // write_debug();
    Start_Control_Timer();
    I2CTWI_initSlave(0x20);
    I2CTWI_readRegisters[0]='H';
    I2CTWI_readRegisters[1]='e';
    I2CTWI_readRegisters[2]='l';
    I2CTWI_readRegisters[3]='l';
    I2CTWI_readRegisters[4]='o';
    Set_Des_Speed(8);
   // Set_Distance(1000);permission.move = 1; stand = 0;
    while( true )
	{
    	task_manage();
    }
	return 0;
}
