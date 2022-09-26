#include "kinematics/kinematics.h"
#define PI 3.14159

//============ GAIT PARAMS =============================================
//special macros for testing
#define RATE 4 //Hz
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

void Leg::compute_IK_XYZ(float x, float y, float z) {
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

    hipAngle = beta + PI/2;
    femurAngle = PI - (theta - zeta);
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

void gait_controller(STATE &state) {
    
    /* 
    UNFINISHED
        Add overlap time support
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

    float incremented_ticks;
    float period_x; //ms for 1 cycle
    float ms_per_ticks;

    if (state.ticks == 100) {
        state.ticks = 0;
        state.pairs = !state.pairs;
    }

    //period_x = 1000 * STEP_SIZE/state.c_x;
    period_x = 1000 * 1/RATE;
    ms_per_ticks = period_x / N_TICKS;
    incremented_ticks = ceil(state.dt/ ms_per_ticks); //ceil or floor works better???
    state.ticks += incremented_ticks;
    if (state.ticks > N_TICKS) state.ticks = N_TICKS;

    compute_swing(state);
    compute_stance(state);

    //static_trot(state);

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
        leg_FL.compute_IK_XYZ(0, 0, z);
        leg_BR.compute_IK_XYZ(0, 0, z);
    }

    else {
        leg_FR.compute_IK_XYZ(0, 0, z);
        leg_BL.compute_IK_XYZ(0, 0, z);
    }
}

void stand(STATE state) {
    leg_FL.compute_IK_XYZ(0, 0, 0);
    leg_BR.compute_IK_XYZ(0, 0, 0);
    leg_FR.compute_IK_XYZ(0, 0, 0);
    leg_BL.compute_IK_XYZ(0, 0, 0);
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
   x = STEP_SIZE/2 - (STEP_SIZE*state.ticks/100);
   y = STEP_SIZE_Y/2 - (STEP_SIZE_Y*state.ticks/100);

    if (state.pairs) {
        leg_FL.compute_IK_XYZ(x, y, 0);
        leg_BR.compute_IK_XYZ(x, y, 0);
    }

    else {
        leg_FR.compute_IK_XYZ(x, y, 0);
        leg_BL.compute_IK_XYZ(x, y, 0);
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

   float x = (STEP_SIZE*state.ticks/200) - (STEP_SIZE/4);
   float y = (STEP_SIZE_Y*state.ticks/200) - (STEP_SIZE_Y/4);
   float z = STEP_HEIGHT*sin(state.ticks*(PI/100));

   if (!state.pairs) {
        leg_FL.compute_IK_XYZ(x, y, z);
        leg_BR.compute_IK_XYZ(x, y, z);
    }

    else {
        leg_FR.compute_IK_XYZ(x, y, z);
        leg_BL.compute_IK_XYZ(x, y, z);
    }
   
}

void yaw_stance(COMMAND command, float &a, float &b, float &c) {

    float theta = command.yaw;

    float alpha;
    float r_l;
    float beta;
    float phi;
    float x;
    float y;
    float z;
    float r;
    

    alpha = PI/2 - theta/2;
    r_l = 2 * sqrt(pow(W_ROBOT, 2) + pow(L_ROBOT, 2))/2 * cos(alpha);
    beta = atan(W_ROBOT/L_ROBOT);
    phi = PI - beta - alpha;
    r = theta/THETA_MAX*(R_MAX - VERT_OFFSET) + VERT_OFFSET;

    x = r_l * cos(phi);
    y = r_l * sin(phi);
    z = VERT_OFFSET - pow(( pow(r,2) - pow(x,2) - pow(y,2) ), 0.5);
    
    leg_FL.compute_IK_XYZ(x, y, z);
    leg_FR.compute_IK_XYZ(-x, y, z);
    leg_BL.compute_IK_XYZ(x, -y, z);
    leg_BR.compute_IK_XYZ(-x, -y, z);
}