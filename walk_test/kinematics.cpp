#include <math.h>
#include "kinematics.h"


Kinematics::Kinematics(int _leg_i, int _dt)
{
    state.leg_i = _leg_i;
    state.dt = _dt;

    state.hipAngle = PI/2;
    state.femurAngle = PI/4;
    state.tibiaAngle = PI/2;
    state.c_x = 0;
    state.c_y = 0;
    state.c_z = 0;
    state.ticks = 0;
    state.p_x = 0;
    state.p_y = 0;
    state.p_z = 0; //relative to its default pose with 45 degree angle between links}

}

void Kinematics::parse_command(COMMAND command, STATE state) {  //not necessary, just parse command direcly to gait_controller()
    // will add RPY later
    state.c_x = command.v_x;
    state.c_y = command.v_y;
    state.c_z = command.v_z;
}

void Kinematics::gait_controller(STATE state) {

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

    int incremented_ticks;
    float period_x; //ms for 1 cycle
    float ms_per_ticks;

    if (state.ticks == 100) {
        state.ticks = 0;
        state.swingPhase = !state.swingPhase;
    }

    period_x = 1000 * STEP_SIZE/state.c_x;
    ms_per_ticks = period_x / N_TICKS;
    incremented_ticks = ceil(state.dt/ ms_per_ticks); //ceil or floor works better???

    state.ticks += incremented_ticks;
    if (state.ticks > N_TICKS) state.ticks = N_TICKS;

    if (!state.swingPhase) compute_stance(state);
    if (state.swingPhase) compute_swing(state);

} 

void Kinematics::compute_stance(STATE state) {
    /*
    GOALS:
        Compute the (X,Y,Z,R,P,Y) of each leg from the phase ticks for the stance legs

    Input:
        Contact mode
        Ticks

    Output:
        (X,Y,Z,R,P,Y) for each stance legs
    */

    state.p_x = STEP_SIZE/2 - (STEP_SIZE*state.ticks/100);
}

void Kinematics::compute_swing(STATE state) {
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

// BUGGEEDDDDDD

void Kinematics::compute_IK(STATE state) {

    float horr_offset = HOR_OFFSET;

    if (state.leg_i == 2 || state.leg_i == 4) {
        state.p_y *= -1;
        horr_offset *= -1;  
    }

    //only x,y,z, for now
    float hip_dis;
    float leg_dis;
    float theta;

    hip_dis = pow((VERT_OFFSET - state.p_z), 2) + pow(state.p_y +  HOR_OFFSET, 2); //squared
    leg_dis = hip_dis - pow(HOR_OFFSET, 2) + pow(state.p_x,2);
    hip_dis = sqrt(hip_dis);
    theta = atan(state.p_x/(VERT_OFFSET - state.p_z));
    
    state.hipAngle = acos(HOR_OFFSET/hip_dis) + atan((state.p_y +  HOR_OFFSET)/(VERT_OFFSET - state.p_z));
    state.femurAngle = atan(state.p_x/(VERT_OFFSET -state.p_z)) + acos(leg_dis/ (2*LEG_LENGHT*sqrt(leg_dis)));
    state.tibiaAngle = acos((2*pow(LEG_LENGHT,2) - leg_dis)/(2*pow(LEG_LENGHT,2)));
}

void Kinematics::motor(STATE state) {
    switch (state.leg_i)
    {
    case 1:
        /* code */
        break;
    
    case 2:
        break;

    case 3:
        break;

    case 4:
        break;
    
    default:
        //RAISE ERROR 
        break;
    }
}
