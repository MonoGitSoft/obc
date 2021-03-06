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


volatile float alap[2][1] = {0,0};
volatile float delta_alap[2][1] = {0,0};
volatile float alap_integral[2][1] = {0,0};
volatile float beav_i[2][1] = {0,0};
volatile float beav_k[2][1] = {0,0};
volatile float states[5][1] = {0,0,0,0,0};
volatile float y_u[4][1] = {0,0,0,0};
volatile float des_states[5][1] = {0,0,0,0,0}; // 1 delta 2 l_sped 3 l_current 4 r_spd 5 r_current
volatile float K[2][5] = {16.0107, 3.0701, 8.4564, -1.8307, 0.0446, -8.8480, -0.1964, 0.1420, 1.5125, 0.3961};
volatile float N_x[5][2] = {1.0000, 0, 0, 1.0000, 0, -0.5857, 0, 1.0000, 0, -14.4306};
volatile float K_i[2][2] = {32.1774, 6.0667, -5.0574, 6.4677};
volatile float Fc[2][5] = {0.0000, -0.7673, 0.0143, 0.0000, 0.0000, 116.6236, -0.8267, -1.7850, -1.0812, 0.4927};
volatile float GHc[2][4] = {0.0000, 0.9357, 0.0092, 0.0000, -116.6236, 2.3214, -0.0195, 0.0156};
volatile float left_res = 0.23;
volatile float right_res = 0.23;

float y_k = 0;
float y_k_1 = 0;
float u_k = 0;
float u_k_1 = 0;
float u_k_2 = 0;

float a_0 = 0.9024;
float b_0 = -0.0796;
float b_1 = 0.07183;

float r_y_k = 0;
float r_y_k_1 = 0;
float r_u_k = 0;
float r_u_k_1 = 0;
float r_u_k_2 = 0;

float r_a_0 = 0.9019;
float r_b_0 = -1.883;
float r_b_1 = 1.698;

void m_multiply(int f_r, int f_c, int s_r, int s_c, float f_m[f_r][f_c], float s_m[s_r][s_c],float r_m[f_r][s_c])
{

    int d = 0;
    int c = 0;
    int k;
    float sum = 0;

    for (c = 0; c < f_r; c++)
    {
      for (d = 0; d < s_c; d++)
      {
        for (k = 0; k < s_r; k++)
        {
          sum = sum + f_m[c][k]*s_m[k][d];
        }
        r_m[c][d] = sum;
        sum = 0;
      }
    }
  }

void vector_sum(int r, float a[r][1], float b[r][1], float c[r][1]) // a+b = c
{
    int i;
    for(i = 0; i < r; i++)
    {
        c[i][0] = a[i][0] + b[i][0];
    }
}

void vector_sub(int r, float a[r][1], float b[r][1], float c[r][1]) //a-b=c
{
    int i;
    for(i = 0; i < r; i++)
    {
        c[i][0] = a[i][0] - b[i][0];
    }
}

void vector_multiply_skalar(int r,float a[r][1], float s) //a = a*s
{
    int i;
    for(i = 0; i < r;i++)
        a[i][0] = a[i][0]*s;
}


void Start_Control_Timer(void)
{
    startStopwatch8();
    startStopwatch7();
    startStopwatch6();
    startStopwatch5();
    startStopwatch4();
}


void Set_Des_Speed(float req_des_speed)
{
    des_speed = req_des_speed;
}

void ss_Set_Des_Speed(float ss_req_speed)
{
    alap[1][0] = ss_req_speed;//(ss_req_speed*0.23)/0.01;
    m_multiply(5,2,2,1,N_x,alap,des_states);
}

void Speed_Measure(void)
{
    if( getStopwatch8() > 10)
    {
        setStopwatch8(0);
        speed_l = (float)mleft_my_speed;
        speed_r = (float)mright_my_speed;
        mright_my_speed = 0;
        mleft_my_speed = 0;
    }
}

void zero(int r,int c, float a[r][c])
{
    int i;
    int k;
    for(i = 0; i < r; i++)
        for(k = 0; k < c; k++)
            a[i][k] = 0;
}

float Left_Current(float u) {
	u_k = u;
	y_k = a_0*y_k_1 + b_0*u_k_1 + b_1*u_k_2;
	y_k_1 = y_k;
	u_k_1 = u_k;
	u_k_2 = u_k_1;
	return y_k;
}

float Right_Current(float u) {
    r_u_k = u;
    r_y_k = r_a_0*r_y_k_1 + r_b_0*r_u_k_1 + r_b_1*r_u_k_2;
    r_y_k_1 = r_y_k;
    r_u_k_1 = r_u_k;
    r_u_k_2 = r_u_k_1;
    return r_y_k;
}

void Reset_Current(void) {
	r_y_k = 0;
	r_y_k_1 = 0;
	r_u_k = 0;
	r_u_k_1 = 0;
	r_u_k_2 = 0;
	y_k = 0;
	y_k_1 = 0;
	u_k = 0;
	u_k_1 = 0;
	u_k_2 = 0;
}

void Update_States(void)
{
    float error;
    error = ((float)mleft_dist*left_res - (float)mright_dist*right_res)*10;
    int l;
    int r;
    if( getStopwatch8() > 20)
    {
        setStopwatch8(0);
        setStopwatch5(0);
        float temp[2][1];
        l = (int)beav_k[0][0];
        r = (int)beav_k[1][0];
        states[0][0] = (float)mleft_dist*0.15 - (float)mright_dist*0.15;
        states[1][0] = (float)mleft_my_speed;//*0.23)/0.01; //speed
        states[2][0] = Left_Current((float)l);
        states[3][0] = (float)mright_my_speed;//*0.23)/0.01; // speed
        states[4][0] = Right_Current((float)r);
        temp[0][0] = states[0][0];
        temp[1][0] = states[1][0];
        vector_sub(2,alap,temp,delta_alap);// integralashoz
        vector_multiply_skalar(2,delta_alap,0.02);//Ts
        vector_sum(2,alap_integral,delta_alap,alap_integral);//integraloook
        m_multiply(2,2,2,1,K_i,alap_integral,beav_i); //K_i*hibajel integral
        vector_sub(5,des_states,states,states); //N_x*r -->> x_r-x
        m_multiply(2,5,5,1,K,states,beav_k); //K
        vector_sum(2,beav_k,beav_i,beav_k);//sum beavjeél
        if(beav_k[0][0] < 0)
           beav_k[0][0] = 0;
        if(beav_k[0][0] > 120)
            beav_k[0][0] = 120;
        if(beav_k[1][0] < 0)
           beav_k[1][0] = 0;
        if(beav_k[1][0] > 120)
            beav_k[1][0] = 120;
        setMotorPower((int)beav_k[0][0],(int)beav_k[1][0]);
      /*  writeInteger(mleft_dist - mright_dist,DEC);writeString(" ");
        writeInteger(mleft_my_speed,DEC);writeString(" ");
        writeInteger(mright_my_speed,DEC);writeString(" ");
        writeInteger((int)states[2][0],DEC);writeString(" ");
        writeInteger((int)states[4][0],DEC);writeString(" ");
        writeInteger((int)beav_k[0][0],DEC);writeString(" ");
        writeInteger((int)beav_k[1][0],DEC);writeString(" ");
        writeChar('\n');*/
        mright_my_speed = 0;
        mleft_my_speed = 0;
    }
    //setMotorPower(beav_k[0][0] - (int)error,beav_k[1][0] + (int)error);
}

void ss_Reset_Control(void)
{
    zero(2,1,alap);
    zero(2,1,delta_alap);
    zero(2,1,alap_integral);
    zero(2,1,beav_i);
    zero(2,1,beav_k);
    zero(5,1,states);
    zero(4,1,y_u);
    distance = 0;
    mleft_dist = 0;
    mright_dist = 0;
}

void Speed_Control(void)
{
    if(getStopwatch7() > 20)
    {
        setStopwatch7(0);
        s_y_l = s_y_l + 0.3*(des_speed - speed_l);
        s_y_r = s_y_r + 0.31*(des_speed - speed_r);
      /*  if(s_y_l > 120 ) {
        	s_y_l = 120;
        }
        if(s_y_r > 120 ) {
        	s_y_r = 120;
        }*/
    } // y = yz-1 + 0.014*e e = des_speed - speed
}

void Set_Distance(uint16_t dist)
{
	setMotorDir(FWD,FWD);
    Set_Des_Speed(6);
    distance = dist;
}

void ss_Set_Distance(uint16_t dist)
{
	setMotorDir(FWD,FWD);
	ss_Set_Des_Speed(5);
    distance = dist;
}

void ss_Rotate_Left(uint16_t angle){
    setMotorDir(BWD,FWD);
    ss_Set_Des_Speed(4);
    distance = angle;
}

void ss_Rotate_Right(uint16_t angle){
    setMotorDir(FWD,BWD);
    ss_Set_Des_Speed(4);
    distance = angle;
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
    inter = 3.25*c_error + 1.2*c_integral;
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
    if(m_left > 180)
        m_left = 180;
    if(m_right > 180)
        m_right = 180;
    setMotorPower(m_left,m_right);
}

void ss_Sub_Control(void)
{
    if((mright_dist > distance) && (stand == 0))
    {

        My_Stop();
        stand = 1;
        ss_Reset_Control();
        Reset_Current();
        permission.move = 0;
        permission.rotate_left = 0;
        permission.rotate_right = 0;

    }
    else
    {
        Update_States();
    }
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

void ss_Control(void)
{
    if( permission.move || permission.rotate_left || permission.rotate_right )
        ss_Sub_Control();
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
    if(getStopwatch6() > 10)
    {
            writeInteger((int)s_y_l,DEC),writeString("\n");
            writeInteger((int)s_y_r,DEC),writeString("\n");
            writeInteger((int)inter,DEC),writeString("\n");
            setStopwatch6(0);
    }
}
