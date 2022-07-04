#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <math.h>
#include <iostream>

enum leg_index {
    FL, //0
    FR, //1
    BL, //2
    BackR  //3
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
    int mode;
    bool pairs;
    
} STATE;

typedef struct command {
    
    float v_x;
    float v_y;
    float v_z;
    float roll;
    float pitch;
    float yaw;

} COMMAND;

void gait_controller(STATE &state);
void compute_stance(STATE state);
void compute_swing(STATE state);
void static_trot(STATE state);

class Leg {
    /*
    This class contains:
        - x,y,z inversel kinematics solver for each leg
        - motor interface
    */
   
   leg_index leg_i;
   
   
   
   float current_x;
   float current_y;
   float current_z;
   
   bool contact;
   
   

public:
    void motor(float hipAngle, float femurAngle, float tibiaAngle);
    Leg(leg_index _leg_i);
    void compute_IK_XYZ(float x, float y, float z);
    float hipAngle;
    float femurAngle;
    float tibiaAngle;
    
};

extern Leg leg_FL;
extern Leg leg_FR;
extern Leg leg_BL;
extern Leg leg_BR;

#endif /* KINEMATICS_H_ */
