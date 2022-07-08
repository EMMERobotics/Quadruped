#include "kinematics.h"
#include <Adafruit_PWMServoDriver.h>
#define PI 3.14159

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
//motor pulse length
#define SERVOMIN  72
#define SERVOMAX  422
#define SERVODIFF  350
//==========================================================================================

//============ MOTOR PARAMS =============================================
#define MOTORTIME 400

const int offset_tibia_1 = 16;
const int offset_tibia_2 = 1;
const int offset_tibia_3 = 18;
const int offset_tibia_4 = 0;

const int offset_femur_1 = -2;
const int offset_femur_2 = 2;
const int offset_femur_3 = 7;
const int offset_femur_4 = -2;

const int offset_waist_1 = -9;
const int offset_waist_2 = -4;
const int offset_waist_3 = 8;
const int offset_waist_4 = 0;

/*
#define SERVO_FREQ 50
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
pwm.begin();
pwm.setOscillatorFrequency(27000000);
pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
*/
//==========================================================================================
/*
    Leg Class
*/

//constuctor

//For use with arduino String class only

#if _MCUENABLE == 1
Leg::Leg(   leg_index _leg_i,
            String _waist_motor_id,
            String _femur_motor_id,
            String _tibia_motor_id,
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
    current_x = 0;
    current_y = 0;
    current_z = 0; //relative to its default pose with 45 degree angle between links}

}
#endif

#if _POSIXENABLE == 1

#endif

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

    

    motor(hipAngle, femurAngle, tibiaAngle);

}

void Leg::motor(float hipAngle, float femurAngle, float tibiaAngle) {
    
    #if _POSIXENABLE == 1
    int waist_val = hipAngle/PI*2000 + 500;
    int femur_val = femurAngle/PI*2000 + 500;
    int tibia_val = tibiaAngle/PI*2000 + 500;
    #endif
   
   float waist_val = hipAngle/PI*SERVODIFF + SERVOMIN + waist_offset;
   float femur_val = femurAngle/PI*SERVODIFF + SERVOMIN + femur_offset;
   float tibia_val = tibiaAngle/PI*SERVODIFF + SERVOMIN + tibia_offset;

   //left side needs to be invert both femur and tibia

    if (leg_index == FL || leg_index == BackR) {

    }

    //if (leg_index == not sure if front or back that needs to be invert

   SerialParser(waist_motor_id, waist_val, MOTORTIME);
   SerialParser(femur_motor_id, femur_val, MOTORTIME);
   SerialParser(tibia_motor_id, tibia_val, MOTORTIME);

    
    #if _POSIXENABLE == 1
    string waist;
    string femur;
    string tibia;
    

    switch (leg_i)
    {
    case FL:
        waist = "#1P" + to_string(waist_val + offset_waist_1) + "T400" + "\r\n";
        femur = "#5P" + to_string(femur_val + offset_femur_1) + "T400" + "\r\n";
        tibia = "#9P" + to_string(tibia_val + offset_tibia_1) + "T400" + "\r\n";
        break;
    
    case FR:
        waist = "#2P" + to_string(waist_val + offset_waist_2) + "T400" + "\r\n";
        femur = "#6P" + to_string(femur_val + offset_femur_2) + "T400" + "\r\n";
        tibia = "#10P" + to_string(tibia_val + offset_tibia_2) + "T400" + "\r\n";
        break;

    case BL:
        waist = "#3P" + to_string(waist_val + offset_waist_3) + "T400" + "\r\n";
        femur = "#7P" + to_string(femur_val + offset_femur_3) + "T400" + "\r\n";
        tibia = "#11P" + to_string(tibia_val + offset_tibia_3) + "T400" + "\r\n";
        break;

    case BackR:

        waist = "#4P" + to_string(waist_val + offset_waist_4) + "T400" + "\r\n";
        femur = "#8P" + to_string(femur_val + offset_femur_4) + "T400" + "\r\n";
        tibia = "#12P" + to_string(tibia_val + offset_tibia_4) + "T400" + "\r\n";
        break;
    
    default:
        //RAISE ERROR 
        break;
    }
    #endif

}

Leg leg_FL( FL,
            "#1P",
            "#5P",
            "#9P",
            offset_waist_1,
            offset_tibia_1,
            offset_tibia_1);

Leg leg_FR( FR,
            "#2P",
            "#6P",
            "#10P",
            offset_waist_2,
            offset_tibia_2,
            offset_tibia_2);

Leg leg_BL( BL,
            "#3P",
            "#7P",
            "#11P",
            offset_waist_3,
            offset_tibia_3,
            offset_tibia_3);

Leg leg_BR( BackR,
            "#4P",
            "#8P",
            "#12P",
            offset_waist_4,
            offset_tibia_4,
            offset_tibia_4);

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
    
    /*
    leg_FL.motor(leg_FL.hipAngle, leg_FL.femurAngle, leg_FL.tibiaAngle);
    leg_FR.motor(leg_FR.hipAngle, leg_FR.femurAngle, leg_FR.tibiaAngle);
    leg_BL.motor(leg_BL.hipAngle, leg_BL.femurAngle, leg_BL.tibiaAngle);
    leg_BR.motor(leg_BR.hipAngle, leg_BR.femurAngle, leg_BR.tibiaAngle);
    */
    
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
