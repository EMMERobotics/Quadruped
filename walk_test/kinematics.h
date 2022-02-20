#include <math.h>

class Kinematics 
{
    //PRIVATE SECTION
    int leg_i; //leg_index (FL, FR, BL, BR) = (1, 2, 3, 4)

    int dt;
    //config parameters (time ms, lenght mm)
    dt = 100;

    //link lenght
    #define LEG_LENGHT 110
    float VERT_OFFSET = 110*sqrt(2);
    float HOR_OFFSET = 20;
    
    //motor offset
    #define HIP_ANGLE_OFFSET 1.5708 //90 degree
    #define FEMUR_ANGLE_OFFSET 0.7854 //45 degree
    #define TIBIA_ANGLE_OFFSET 1.5708 //90degree

    //PUBLIC SECTION
    public: 
    float hipAngle;
    float femurAngle;
    float tibiaAngle;
    
    //state
    

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
        float theta; // for

        hip_dis = pow((VERT_OFFSET - z), 2) + pow(y +  HOR_OFFSET, 2); //squared
        leg_dis = hip_dis - pow(HOR_OFFSET, 2);
        hip_dis = sqrt(hip_dis);

        leg_dis = leg_dis + pow(x,2);

        
        
        hipAngle = acos(HOR_OFFSET/hip_dis) + atan((y +  HOR_OFFSET)/(VERT_OFFSET - z));
        femurAngle = FEMUR_ANGLE_OFFSET - atan(x/(VERT_OFFSET -z));
        tibiaAngle = acos((2*pow(LEG_LENGHT,2) - leg_dis)/(2*pow(LEG_LENGHT,2)));
    }

    //constructor
    Kinematics(int Leg) {
        Leg = leg_i;
    }

};