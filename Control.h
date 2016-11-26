#ifndef CONTROL_H
#define CONTROL_H

#include "RP6RobotBaseLib.h"

void Speed_Measure(void);
void Control(void);
void Start_Control_Timer(void);
void Set_Des_Speed(float des_speed);
void Set_Distance(uint16_t dist);
void ss_Set_Des_Speed(float ss_req_speed);
void ss_Control(void);
void Control_Debug(void);
void Update_States(void);
void Rotate_Right(uint16_t angle);
void Rotate_Left(uint16_t angle);
void Stream(void);
extern volatile float speed_l;
extern volatile float speed_r;
extern volatile uint16_t distance;
extern volatile uint8_t stand;


#endif
