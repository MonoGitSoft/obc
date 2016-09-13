/*
 * DeadReckoning.c
 *
 *  Created on: Sep 13, 2016
 *      Author: mono
 */

#include "DeadReckoning.h"
#include "RP6RobotBaseLib.h"
#include <math.h>
#include "RobotParam.h"


volatile float x = 0;
volatile float y = 0;
volatile float theta = 0;


float dist_left(void)
{
    float l_dist= (float)mleft_dist_pose;
    mleft_dist_pose = 0;
    if( permission.rotate_left )
        return (float)(0 - (float)l_dist*RES_L);
    else
        return (float)l_dist*RES_L;
}

float dist_right(void)
{
    float r_dist = (float)mright_dist_pose;
    mright_dist_pose = 0;
    if( permission.rotate_right )
        return (float)(0 - (float)r_dist*RES_R);
    else
        return (float)r_dist*RES_R;
}

void dead_reckoning(void)
{
    float dtheta;
    float S_l;
    float S_r;
    float S_p;
    S_l = dist_left();
    S_r = dist_right();
    S_p = (S_r + S_l)/2;
    dtheta = (S_r - S_l)/(DIAM*2);
    x = x + S_p*cos(theta + dtheta/2);
    y = y + S_p*sin(theta + dtheta/2);
    theta = theta + dtheta;
    if( theta > PI )
        theta = -(float)(2*PI - theta);
    if( theta < -PI )
        theta = (float)(2*PI + theta);
}
