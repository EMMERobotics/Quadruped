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
   
   bool contact;

   int waist_offset;
   int femur_offset;
   int tibia_offset;

   //#if _MCUENABLE == 1
   int waist_motor_id;
   int femur_motor_id;
   int tibia_motor_id;
   //#endif

   #if _POSIXENABLE == 1
   string waist_motor_id;
   string femur_motor_id;
   string tibia_motor_id;
   #endif

public:
    
    leg_index leg_i;
    
    //#if _MCUENABLE == 1
    Leg(  leg_index _leg_i, 
          uint8_t _waist_motor_id,
          uint8_t _femur_motor_id,
          uint8_t _tibia_motor_id,
          int _waist_offset,
          int _femur_offset,
          int _tibia_offset);
    //#endif
          
    void compute_IK_XYZ(float x, float y, float z);

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
