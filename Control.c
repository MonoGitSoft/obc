#include "Control.h"
#include "RP6uart.h"
#include "RobotParam.h"

volatile uint8_t start_flag = 1;
volatile uint8_t stand = 1;

// Speep Control I contorller
volatile float des_speed = 0;
volatile float speed_l = 0;
volatile float speed_r = 0;
volatile float s_y_l = 0;
volatile float s_y_r = 0;

//Path Controller

volatile float c_error = 0;
volatile float c_integral = 0;
volatile float inter = 0;

volatile uint16_t distance = 0;

//Space-State control



volatile float left_res = 0.23;
volatile float right_res = 0.23;


void Start_Control_Timer(void)
{
    startStopwatch8();
    startStopwatch7();
    startStopwatch6();
    startStopwatch5();
}


void Set_Des_Speed(float req_des_speed)
{
    des_speed = req_des_speed;
}

void Speed_Measure(void)
{
    if( getStopwatch8() > 20)
    {
        setStopwatch8(0);
        speed_l = (float)mleft_my_speed;
        speed_r = (float)mright_my_speed;
        mright_my_speed = 0;
        mleft_my_speed = 0;
    }
}

void Speed_Control(void)
{
    if(getStopwatch7() > 20)
    {
        setStopwatch7(0);
        s_y_l = s_y_l + 0.3*(des_speed - speed_l);
        s_y_r = s_y_r + 0.31*(des_speed - speed_r);
    } // y = yz-1 + 0.014*e e = des_speed - speed
}

void Set_Distance(uint16_t dist)
{
	setMotorDir(FWD,FWD);
    Set_Des_Speed(6);
    distance = dist;
}

void Rotate_Left(uint16_t angle){
    setMotorDir(BWD,FWD);
    Set_Des_Speed(4);
    distance = angle;
}

void Rotate_Right(uint16_t angle){
    setMotorDir(FWD,BWD);
    Set_Des_Speed(4);
    distance = angle;
}

void Path_Control(void)
{
    c_error = ((float)mleft_dist*RES_L - (float)mright_dist*RES_R)/RES_E;
    if(getStopwatch5() > 2)
    {
        setStopwatch5(0);
        c_integral = c_integral + c_error*0.001;
    }
    inter = 2*c_error + 2*c_integral;
}

void My_Stop(void)
{
    setMotorPower(0,0);
    while( mleft_speed != 0 && mright_speed != 0 )
    {
    }
}


void Reset_Control(void)
{
    s_y_l = 0;
    s_y_r = 0;
    distance = 0;
    mleft_dist = 0;
    mright_dist = 0;
}

void Intervention(void)
{
    int m_left = (int)(s_y_l - inter);
    int m_right = (int)(s_y_r + inter);
    if(m_left < 0)
        m_left = 0;
    if(m_right < 0)
        m_right = 0;
    if(m_left > 110)
        m_left = 110;
    if(m_right > 110)
        m_right = 110;
    setMotorPower(m_left,m_right);
}


void Sub_Control(void)
{
    if((mright_dist > distance) && (stand == 0) )
    {
        My_Stop();
        stand = 1; // ja megállasldalsd
        Reset_Control();
        permission.move = 0;
        permission.rotate_left = 0;
        permission.rotate_right = 0;
    }
    else
    {
        Speed_Control();
        Path_Control();
        Intervention();
    }
}


void Control(void)
{
    Speed_Measure();
    if( permission.move || permission.rotate_left || permission.rotate_right ) {
        Sub_Control();
    }
    else
    {
        if(getStopwatch7() > 5)
        {
            setStopwatch5(0);
            setStopwatch7(0);
            setStopwatch8(0);
        }
    }
}

void Control_Debug(void)
{
    if(getStopwatch6() > 20)
    {
            writeInteger(mright_dist,DEC),writeString("\n");
            writeInteger(distance,DEC),writeString("\n");
            writeInteger(stand,DEC),writeString("\n");
            setStopwatch6(0);
    }
}
