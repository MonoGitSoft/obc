
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
            case MOVE_FORWARD: Set_Distance((uint16_t)((float)param/RES_R));permission.move = 1; stand = 0;break; //set dist,add permission to move
            case ROTATE_LEFT: Rotate_Left((uint16_t)(((float)param*(PI/180)*(DIAM))/(RES_R))) ;stand = 0;permission.rotate_left = 1; break;
            case ROTATE_RIGHT: Rotate_Right((uint16_t)(((float)param*(PI/180)*(DIAM))/(RES_R)));  stand = 0;permission.rotate_right = 1;break;
            case SEND_POSE: SendPose();break;
            case SEND_ENCODER: SendEncoder();break;
            case STREAM: permission.stream = 1;
        }
        Command = 0;
    }
}

void ss_command_processor(void)
{
    if( Command != 0 )
    {
        switch( Command )
        {
            case MOVE_FORWARD: ss_Set_Distance((uint16_t)((float)param/RES_R));permission.move = 1; stand = 0;break; //set dist,add permission to move
            case ROTATE_LEFT: ss_Rotate_Left((uint16_t)(((float)param*(PI/180)*(DIAM))/(RES_R))); stand = 0;permission.rotate_left = 1; break;
            case ROTATE_RIGHT: ss_Rotate_Right((uint16_t)(((float)param*(PI/180)*(DIAM))/(RES_R)));  stand = 0;permission.rotate_right = 1;break;
            case SEND_POSE: SendPose();break;
            case SEND_ENCODER: SendEncoder();break;
            case STREAM: permission.stream = 1;
        }
        Command = 0;
    }
}

void task_manage(void)
{
	UartCom();
    ss_command_processor();
    task_ADC();
    //Control();
    ss_Control();
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
    //-----------SS test
   /* setMotorDir(FWD,FWD);
    ss_Set_Des_Speed(5);
    ss_Set_Distance(100);
    permission.move = 1;
    stand = 0;*/
    //ss test-------------
    while( true )
	{
    	task_manage();
    }
	return 0;
}
