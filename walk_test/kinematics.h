#include <math.h>
#define PI 3.14159

class Kinematics 
{
    //PRIVATE SECTION
    int leg_i; //leg_index (FL, FR, BL, BR) = (1, 2, 3, 4)
    int dt;

    //CONFIGS

    //=========================== INVERSE KINEMATICS PARAMS ====================================
    //link lenght
    #define LEG_LENGHT 110
    float VERT_OFFSET = LEG_LENGHT*sqrt(2); //must be adjustable
    float HOR_OFFSET = 20; //NOT ACTUAL VALUE!!!   <=================================
    //motor offset
    #define HIP_ANGLE_OFFSET 1.5708 //90 degree
    #define FEMUR_ANGLE_OFFSET 0.7854 //45 degree
    #define TIBIA_ANGLE_OFFSET 1.5708 //90degree
    //==========================================================================================

    //PUBLIC SECTION
    public: 
    float hipAngle;
    float femurAngle;
    float tibiaAngle;
    
    //state


    //constructor
    Kinematics(int Leg, int dt_) {
        Leg = leg_i;
        dt_ = dt;

        //set default angle when object is created
        hipAngle = PI/2;
        femurAngle = PI/4;
        tibiaAngle = PI/2;
    }

    //methods

    void compute_IK(int leg_i, float x, float y, float z, float roll, float pitch, float yaw)
    {

        if (leg_i == 2 || leg_i == 4) {
            y = y * -1;
            HOR_OFFSET = HOR_OFFSET * -1;  
        }

        //only x,y,z, for now
        float hip_dis;
        float leg_dis;
        float theta;

        hip_dis = pow((VERT_OFFSET - z), 2) + pow(y +  HOR_OFFSET, 2); //squared
        leg_dis = hip_dis - pow(HOR_OFFSET, 2) + pow(x,2);
        hip_dis = sqrt(hip_dis);
        theta = atan(x/(VERT_OFFSET -z));
        
        hipAngle = acos(HOR_OFFSET/hip_dis) + atan((y +  HOR_OFFSET)/(VERT_OFFSET - z));
        femurAngle = atan(x/(VERT_OFFSET -z)) + acos(leg_dis/ (2*LEG_LENGHT*sqrt(leg_dis)));
        tibiaAngle = acos((2*pow(LEG_LENGHT,2) - leg_dis)/(2*pow(LEG_LENGHT,2)));
    }

    

};
