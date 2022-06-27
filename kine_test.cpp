#include <iostream>
#include <math.h>
#include "kinematics.h"

#define PI 3.14159

int main () {

    Kinematics leg_FL = Kinematics(1, 10);

    for (int i=10; i>0; --i){
        leg_FL.compute_IK(leg_FL.state);
        //std::cout << leg_FL.hipAngle*180/PI << std::endl;
        //std::cout << "femur: " << int(leg_FL.femurAngle*180/PI) << std::endl;
        //std::cout << "tibia" << leg_FL.tibiaAngle*180/PI << std::endl;
    }

    for (int i=0; i<10; ++i){
        leg_FL.compute_IK(leg_FL.state);
        std::cout << leg_FL.state.hipAngle*180/PI << std::endl;
        std::cout << "femur: " << int(leg_FL.state.femurAngle*180/PI) << std::endl;
        std::cout << "tibia: " << leg_FL.state.tibiaAngle*180/PI << std::endl;
    }
    
    return 0;
}