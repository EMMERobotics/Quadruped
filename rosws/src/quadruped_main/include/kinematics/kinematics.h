#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#define _MCUENABLE 0
#define _POSIXENABLE 0

#include <math.h>
#include <stdint.h>

#if _POSIXENABLE == 1
#include <iostream>
#include <string>
#endif

#if _MCUENABLE == 1
#include <Arduino.h>
#endif

enum leg_index {
    FL, //0
    FR, //1
    BL, //2
    BackR  //3
};

enum leg_phase {
    SWING,
    STANCE
};

enum robot_phase {
    STILL,
    STEP_TROT,
    TROT,
    STOP_TROT,
    STEP_CRAWL,
    CRAWL_DIS,
    STOP_CRAWL
};

enum robot_mode {
    RPY, 
    NORM,
    CRAWL
};

enum crawl_phase {
    BACK_RIGHT,
    FRONT_RIGHT,
    BD1,
    BACK_LEFT,
    FRONT_LEFT,
    BD2
};

typedef struct state {

    int dt;
    unsigned long current_time;

    //robot frame curremt pose   
    float c_x;
    float c_y;
    float c_z;
    float c_R;
    float c_P;
    float c_Y;

    float p_x;
    float p_y;
    float p_z;

    int ticks;
    bool pairs;

    int com_vx;
    int com_vy;
    int com_vz;
    int com_roll;
    int com_pitch;
    int com_yaw;

    robot_phase exphase;
    robot_phase comphase;

    robot_mode mode;
    crawl_phase crawlphase;
    bool crawl_completed;
    
} STATE;

void gait_controller(STATE &state);
void compute_stance(STATE state);
void compute_swing(STATE state);
void static_trot(STATE state);
void stand(STATE state);

class Leg {
    /*
    This class contains:
        - x,y,z inversel kinematics solver for each leg
        - motor interface
    */
   
   float current_x;
   float current_y;
   float current_z;

public:
    
    leg_index leg_i;
    leg_phase phase;
    
    Leg(leg_index _leg_i);
    //#endif
          
    void compute_IK_XYZ(float x, float y, float z, float row, float pitch, float yaw);
    void compute_swing(STATE &state);
    void compute_stance(STATE &state);

    #if _MCUENABLE == 1
    //void motor(float hipAngle, float femurAngle, float tibiaAngle);
    //void SerialParser(String motor_id, int pos, int time);
    
    void motor_arduino(float hipAngle, float femurAngle, float tibiaAngle);
    #endif

    float hipAngle;
    float femurAngle;
    float tibiaAngle;
    
};

extern Leg leg_FL;
extern Leg leg_FR;
extern Leg leg_BL;
extern Leg leg_BR;

#endif /* KINEMATICS_H_ */
