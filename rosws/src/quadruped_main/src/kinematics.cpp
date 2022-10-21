#include "kinematics/kinematics.h"
#include <iostream>
#define PI 3.14159

//============ GAIT PARAMS =============================================
//special macros for testing
#define RATE 6 //Hz
#define STILLTIME 0.3

#define STEP_SIZE 40
#define STEP_SIZE_Y 30
#define STEP_HEIGHT 40
#define N_TICKS 100 // number of ticks per cycle

#define W_ROBOT 195 // Width of the robot from the end of the leg to another leg (y direction)
#define L_ROBOT 283.7  // Length of the robot from the end of the leg to another leg (x direction)
#define THETA_MAX 1.07 // maximum yaw range (test)
#define R_MAX 155.5635 // maximum of leg length for yaw stance (test)
//============ INVERSE KINEMATICS PARAMS ====================================
//link lenght
#define LEG_LENGHT 115
#define VERT_OFFSET  162.6345 //must be adjustable (default 45 degree angle between links)
#define HOR_OFFSET  20 //NOT ACTUAL VALUE!!!   <=================================
//motor offset 
#define HIP_ANGLE_OFFSET 1.5708 //90 degree
#define FEMUR_ANGLE_OFFSET 0.7854 //45 degree
#define TIBIA_ANGLE_OFFSET 1.5708 //90degree1
//==========================================================================================
const float beta = atan(W_ROBOT/L_ROBOT);
const float P = sqrt(pow(W_ROBOT, 2) + pow(L_ROBOT, 2));
//==========================================================================================
/*
    Leg Class
*/

//constuctor
Leg::Leg(   leg_index _leg_i,
            uint8_t _waist_motor_id,
            uint8_t _femur_motor_id,
            uint8_t _tibia_motor_id,
            int _waist_offset,
            int _femur_offset,
            int _tibia_offset)
{
    leg_i = _leg_i;

    waist_motor_id = _waist_motor_id;
    femur_motor_id = _femur_motor_id;
    tibia_motor_id = _tibia_motor_id;

    waist_offset = _waist_offset;
    femur_offset = _femur_offset;
    tibia_offset = _tibia_offset;

    hipAngle = PI/2;
    femurAngle = PI/4;
    tibiaAngle = PI/2;

}

void Leg::compute_IK_XYZ(float x, float y, float z, float roll, float pitch, float yaw) {

	float y_roll;
	float z_roll;
	float x_pitch;
	float z_pitch;
	float x_yaw;
	float y_yaw;

    // roll
    if (leg_i == 0 || leg_i == 2) {
        y_roll = W_ROBOT*(1-cos(roll))/2;
        z_roll = -W_ROBOT*sin(roll)/2;
    }
    else {
        y_roll = -W_ROBOT*(1-cos(-roll))/2;
        z_roll = -W_ROBOT*sin(-roll)/2;
    }

    // pitch
    if (leg_i == 0 || leg_i == 1) {
        x_pitch = L_ROBOT*(1-cos(pitch))/2;
        z_pitch = -L_ROBOT*sin(pitch)/2;
    }
    else {
        x_pitch = -L_ROBOT*(1-cos(-pitch))/2;
        z_pitch = -L_ROBOT*sin(-pitch)/2;
    }
    
    // yaw
    float alpha;
    float phi_yaw;
    float r_l;
    alpha = PI/2 - abs(yaw/2);
    r_l = cos(alpha) * P;
    if (leg_i == 0 ||  leg_i == 3) {
        if (yaw > 0) phi_yaw = PI - alpha - beta;
        else phi_yaw = PI - alpha - beta - abs(yaw);
    }
    else if (leg_i == 0 ||  leg_i == 3) {
        if (yaw > 0) phi_yaw = PI - alpha - beta - abs(yaw);
        else phi_yaw = PI - alpha - beta;
    }
    
    int operate;
    if (yaw<0) operate = -1;
    else operate = 1;

    if (leg_i == 0) {
        x_yaw = operate * r_l * cos(phi_yaw);
        y_yaw = (-1) * operate * r_l * sin(phi_yaw);
    }
    else if (leg_i == 3) {
        x_yaw = (-1) * operate * r_l * cos(phi_yaw);
        y_yaw = operate * r_l * sin(phi_yaw);
    }
    else if (leg_i == 1) {
        x_yaw = (-1) * operate * r_l * cos(-phi_yaw);
        y_yaw = (-1) * operate * r_l * sin(-phi_yaw);
    }
    else if (leg_i == 2) {
        x_yaw = operate * r_l * cos(-phi_yaw);
        y_yaw = operate * r_l * sin(-phi_yaw);
    }
    
    x += x_pitch + x_yaw;
    y += y_roll + y_yaw;
    z += z_roll + z_pitch;

    // refference from front left leg
    
    float horr_offset = HOR_OFFSET;
    float leg_lenght = LEG_LENGHT;

    if (leg_i == 1 || leg_i == 3) {
        y *= -1;
    }

    //only x,y,z, for now
    float l4_sqrt;
    float l3_x0_sqrt;
    float l3_sqrt;
    float beta;
    float h;
    float phi;
    float theta;
    float zeta;

    h = VERT_OFFSET - z;
    l4_sqrt = pow((h), 2) + pow(y +  horr_offset, 2);
    l3_x0_sqrt = l4_sqrt - pow(horr_offset, 2);
    l3_sqrt = l3_x0_sqrt + pow(x,2);

    beta = asin(sqrt(l3_x0_sqrt/l4_sqrt)) - atan2(h, y +  horr_offset);
    phi = acos((l3_sqrt / (2*pow(leg_lenght, 2))) - 1);
    theta = atan2(x, h) + PI/2;
    zeta = atan2(leg_lenght * sin(phi), leg_lenght * (1+cos(phi)));

    if (leg_i == 1 || leg_i == 3) {
        beta *= -1;
    }

    hipAngle = beta + PI/2 - roll;
    femurAngle = PI - (theta - zeta) - pitch;
    tibiaAngle = PI - phi; // new leg design
}


Leg leg_FL( FL,     //leg index
            0,      //waist motor number
            4,      //femur motor number
            8,      //tibia motor number
            70,      //waist offset
            0,     //femur offset
            10);    //tibia offset

Leg leg_FR( FR,
            1,
            5,
            9,
            -50,
            -20,
            50);

Leg leg_BL( BL,
            2,
            6,
            10,
            50,
            20,
            -70);

Leg leg_BR( BackR,
            3,
            7,
            11,
            -50,
            -150,
            40);


/*
    No class
*/

float map(float val, int min_old, int max_old, int min_new, int max_new) {
    float new_val;
    new_val = val/(max_old-min_old) * (max_new-min_new) + min_new;
    return new_val;
}


void test_rpy(STATE state) {
    float roll;
    float pitch;
    float yaw;
    roll = map(state.com_vx, 0, 255, -10, 10);
    pitch = map(state.com_vy, 0, 255, -10, 10);
    yaw = map(state.com_vz, 0, 255, -10, 10);
	leg_FL.compute_IK_XYZ(0, 0, 0, roll, pitch, yaw);
    leg_BR.compute_IK_XYZ(0, 0, 0, roll, pitch, yaw);
    leg_FR.compute_IK_XYZ(0, 0, 0, roll, pitch, yaw);
    leg_BL.compute_IK_XYZ(0, 0, 0, roll, pitch, yaw);
}
void gait_controller(STATE &state) {
    
    /* 
    UNFINISHED
        Add trasnsition

    GOALS:
        Determine the steps' frequency from the input velocity
    
    Conditions:
        The step lenght is fixed

    Input:
        Crawl/Trot velocity command (mm/s)
        Time

    Output:
        Contact mode of each legs (Swing legs, Stance legs)
        Phase of each swing (ticks) since each step is executed

    */

    if (state.exphase == TROT || state.exphase == STEP_TROT || state.exphase == STOP_TROT) {
        
        float incremented_ticks;
        float period_x; //ms for 1 cycle
        float ms_per_ticks;

        //period_x = 1000 * STEP_SIZE/state.c_x;
        period_x = 1000 * 1/RATE;
        ms_per_ticks = period_x / N_TICKS;
        incremented_ticks = ceil(state.dt/ ms_per_ticks); //ceil or floor works better???
        state.ticks += incremented_ticks;
        if (state.ticks > N_TICKS) state.ticks = N_TICKS;
    }

    else {
        state.ticks = 0;
    }

    //read state change command
    if (state.com_vx == 127) {
        state.comphase = STILL;
    }
    else {
        state.comphase = TROT;
    }

    if (state.exphase == STILL && state.comphase == TROT) {
        state.exphase = STEP_TROT;
    }

    else if (state.comphase == TROT && state.exphase == STEP_TROT && state.ticks > 90) {
        state.exphase = TROT;
    }

    else if (state.comphase == STILL && state.exphase == TROT  && state.ticks > 90) {
        state.exphase = STOP_TROT;
    }

    else if (state.comphase == STILL && state.exphase == STOP_TROT  && state.ticks > 90) {
        state.exphase = STILL;
    }

    if (state.ticks == 100) {
        state.ticks = 0;
        state.pairs = !state.pairs;
    }
    test_rpy(state);
    //compute_swing(state);
    //compute_stance(state);


    //static_trot(state);
    //stand(state);
}

float exec_tick = N_TICKS - STILLTIME*N_TICKS;
void static_trot(STATE state) {

    float z;

    if (state.ticks < exec_tick + 1 && state.ticks < exec_tick/2)  {
        z = STEP_HEIGHT*state.ticks/(exec_tick/2);
    }

    else if (state.ticks < exec_tick + 1 && state.ticks > exec_tick/2) {
        z = STEP_HEIGHT - STEP_HEIGHT*(state.ticks/(exec_tick/2)-1);
    }

    else {
        z = 0;
    }

    if (state.pairs) {
        leg_FL.compute_IK_XYZ(0, 0, z, 0, 0, 0);
        leg_BR.compute_IK_XYZ(0, 0, z, 0, 0, 0);
    }

    else {
        leg_FR.compute_IK_XYZ(0, 0, z, 0, 0, 0);
        leg_BL.compute_IK_XYZ(0, 0, z, 0, 0, 0);
    }
}

void stand(STATE state) {
    leg_FL.compute_IK_XYZ(0, 0, 0, 0, 0, 0);
    leg_BR.compute_IK_XYZ(0, 0, 0, 0, 0, 0);
    leg_FR.compute_IK_XYZ(0, 0, 0, 0, 0, 0);
    leg_BL.compute_IK_XYZ(0, 0, 0, 0, 0, 0);
}


void compute_stance(STATE state) {
    /*
    GOALS:
        Compute the (X,Y,Z,R,P,Y) of each leg from the phase ticks for the stance legs

    Input:
        Contact mode
        Ticks

    Output:
        (X,Y,Z,R,P,Y) for each stance legs
    */
   
    float x;
    float y;
    float z;
    //x = STEP_SIZE/2 - (STEP_SIZE*state.ticks/100);
    //y = STEP_SIZE_Y/2 - (STEP_SIZE_Y*state.ticks/100);
   
    switch (state.exphase)
    {
    case STEP_TROT:
        x = 0 - (STEP_SIZE*state.ticks/200); 
        break;
    
    case STOP_TROT:
        x = STEP_SIZE/2 - (STEP_SIZE*state.ticks/200);
        break;
    
    case TROT:
        x = STEP_SIZE/2 - (STEP_SIZE*state.ticks/100);
        //y = STEP_SIZE_Y/2 - (STEP_SIZE_Y*state.ticks/100); 
        break;

    default:
        x = 0;
	y = 0;
	z = 0;
        break;
    }

    if (state.pairs) {

        leg_FL.compute_IK_XYZ(x, y, 0, 0, 0, 0);
        leg_BR.compute_IK_XYZ(x, y, 0, 0, 0, 0);
    }

    else {
        leg_FR.compute_IK_XYZ(x, y, 0, 0, 0, 0);
        leg_BL.compute_IK_XYZ(x, y, 0, 0, 0, 0);
    }
}

void compute_swing(STATE state) {
    /*
    GOALS:
        Compute the (X,Y,Z,R,P,Y) of each leg from the phase ticks for the swing legs

    Input:
        Contact mode
        Ticks

    Output:
        (X,Y,Z,R,P,Y) for each swing legs
    */

    float x;
    float y;
    float z;
    //float y = (STEP_SIZE_Y*state.ticks/200) - (STEP_SIZE_Y/4);
    //float z = STEP_HEIGHT*sin(state.ticks*(PI/100));

    switch (state.exphase)
    {
    case STEP_TROT:
        x = (STEP_SIZE*state.ticks/400);
	z = STEP_HEIGHT*sin(state.ticks*(PI/100));
        break;
    
    case STOP_TROT:
        x = (STEP_SIZE*state.ticks/400) - (STEP_SIZE/4); 
	z = STEP_HEIGHT*sin(state.ticks*(PI/100));
        break;

    case TROT:
        x = (STEP_SIZE*state.ticks/200) - (STEP_SIZE/4); 
	z = STEP_HEIGHT*sin(state.ticks*(PI/100));
        break;
    
    default:
        x = 0;
	y = 0;
	z = 0;
        break;
    }

   if (!state.pairs) {
        leg_FL.compute_IK_XYZ(x, y, z, 0, 0, 0);
        leg_BR.compute_IK_XYZ(x, y, z, 0, 0, 0);
    }

    else {
        leg_FR.compute_IK_XYZ(x, y, z, 0, 0, 0);
        leg_BL.compute_IK_XYZ(x, y, z, 0, 0, 0);
    }

}
