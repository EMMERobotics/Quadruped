#include <math.h>
#include "kinematics.h"
#include <string>
#include <iostream>
#define PI 3.14159
#define _POSIX

//============ GAIT PARAMS =============================================
#define STEP_SIZE 40
#define STEP_HEIGHT 10
#define N_TICKS 100 // number of ticks per cycle

//============ INVERSE KINEMATICS PARAMS ====================================
//link lenght
#define LEG_LENGHT 110
#define VERT_OFFSET  155.5635 //must be adjustable (default 45 degree angle between links)
#define HOR_OFFSET  20 //NOT ACTUAL VALUE!!!   <=================================
//motor offset 
#define HIP_ANGLE_OFFSET 1.5708 //90 degree
#define FEMUR_ANGLE_OFFSET 0.7854 //45 degree
#define TIBIA_ANGLE_OFFSET 1.5708 //90degree1
//==========================================================================================

const int offset_tibia_1 = 90;
const int offset_tibia_2 = 5;
const int offset_tibia_3 = 100;
const int offset_tibia_4 = 0;

const int offset_femur_1 = -10;
const int offset_femur_2 = 10;
const int offset_femur_3 = 40;
const int offset_femur_4 = -10;

const int offset_waist_1 = -50;
const int offset_waist_2 = -25;
const int offset_waist_3 = 45;
const int offset_waist_4 = 0;


/*
    Leg Class
*/

//constuctor
Leg::Leg(leg_index _leg_i)
{
    leg_i = _leg_i;

    hipAngle = PI/2;
    femurAngle = PI/4;
    tibiaAngle = PI/2;
    current_x = 0;
    current_y = 0;
    current_z = 0; //relative to its default pose with 45 degree angle between links}

}

void Leg::compute_IK_XYZ(float x, float y, float z) {

    float horr_offset = HOR_OFFSET;

    if (leg_i == 2 || leg_i == 4) {
        y *= -1;
        horr_offset *= -1;  
    }

    //only x,y,z, for now
    float hip_dis;
    float leg_dis;
    float theta;

    hip_dis = pow((VERT_OFFSET - z), 2) + pow(y +  HOR_OFFSET, 2); //squared
    leg_dis = hip_dis - pow(HOR_OFFSET, 2) + pow(x,2);
    hip_dis = sqrt(hip_dis);
    theta = atan(x/(VERT_OFFSET - z));
    
    hipAngle = acos(HOR_OFFSET/hip_dis) + atan((y +  HOR_OFFSET)/(VERT_OFFSET - z));
    femurAngle = atan(x/(VERT_OFFSET - z)) + acos(leg_dis/ (2*LEG_LENGHT*sqrt(leg_dis)));
    tibiaAngle = acos((2*pow(LEG_LENGHT,2) - leg_dis)/(2*pow(LEG_LENGHT,2)));
}

void Leg::motor(float hipAngle, float femurAngle, float tibiaAngle) {
    
    int waist_val = hipAngle/180*2000 + 500;
    int femur_val = femurAngle/180*2000 + 500;
    int tibia_val = tibiaAngle/180*2000 + 500;

    switch (leg_i)
    {
    case FL:
        /* code */
        String waist = "#1P" + String(waist_val + offset_waist_1) + "T400" + "\r\n";
        String femur = "#5P" + String(femur_val + offset_femur_1) + "T400" + "\r\n";
        String tibia = "#9P" + String(tibia_val + offset_tibia_1) + "T400" + "\r\n";
        break;
    
    case FR:
        String waist = "#2P" + String(waist_val + offset_waist_2) + "T400" + "\r\n";
        String femur = "#6P" + String(femur_val + offset_femur_2) + "T400" + "\r\n";
        String tibia = "#10P" + String(tibia_val + offset_tibia_2) + "T400" + "\r\n";
        break;

    case BL:
        String waist = "#3P" + String(waist_val + offset_waist_3) + "T400" + "\r\n";
        String femur = "#7P" + String(femur_val + offset_femur_3) + "T400" + "\r\n";
        String tibia = "#11P" + String(tibia_val + offset_tibia_3) + "T400" + "\r\n";
        break;

    case BackR:

        String waist = "#4P" + String(waist_val + offset_waist_4) + "T400" + "\r\n";
        String femur = "#8P" + String(femur_val + offset_femur_4) + "T400" + "\r\n";
        String tibia = "#12P" + String(tibia_val + offset_tibia_4) + "T400" + "\r\n";
        break;
    
    default:
        //RAISE ERROR 
        break;
    }
}

Leg leg_FL(FL);
Leg leg_FR(FR);
Leg leg_BL(BL);
Leg leg_BR(BackR);

/*
    No class
*/

//special macros for testing
#define RATE 4 //Hz
#define STILLTIME 0.3 //percent of the gait cycle that all 4 legs will be on the ground

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
    period_x = 1000 * 1/4;
    ms_per_ticks = period_x / N_TICKS;
    incremented_ticks = ceil(state.dt/ ms_per_ticks); //ceil or floor works better???
    state.ticks += 4;
    //std::cout << state.ticks << std::endl;
    if (state.ticks > N_TICKS) state.ticks = N_TICKS;

    //compute_stance(state);
    //compute_swing(state);

    static_trot(state);
    
//    return state.ticks, state.pairs
}

float exec_tick = N_TICKS - STILLTIME*N_TICKS;
#define Z_HEIGHT 20

void static_trot(STATE state) {

    float z;

    if (state.ticks < exec_tick + 1 && state.ticks < exec_tick/2)  {
        z = Z_HEIGHT*state.ticks/(exec_tick/2);
    }

    else if (state.ticks < exec_tick + 1 && state.ticks > exec_tick/2) {
        z = Z_HEIGHT - Z_HEIGHT*(state.ticks/(exec_tick/2)-1);
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
//    std::cout << z << std::endl;
    #ifdef _POSIX
//    std::cout << state.ticks << std::endl;
        //std::cout << z <<std::endl;
    #endif
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
   x = STEP_SIZE/2 - (STEP_SIZE*state.ticks/100);

    if (state.pairs) {
        leg_FL.compute_IK_XYZ(x, 0, 0);
        leg_BR.compute_IK_XYZ(x, 0, 0);
    }

    else {
        leg_FR.compute_IK_XYZ(x, 0, 0);
        leg_BL.compute_IK_XYZ(x, 0, 0);
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
}
