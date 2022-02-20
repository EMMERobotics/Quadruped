#include <iostream>
#include <math.h>
#include "kinematics.h"

#define PI 3.14159

int main () {

    Kinematics leg_FL = Kinematics(1);
    
    leg_FL.compute_IK(2, -50, 0, 0, 0, 0, 0);
    std::cout << leg_FL.hipAngle*180/PI << std::endl;
    std::cout << leg_FL.femurAngle*180/PI << std::endl;
    std::cout << leg_FL.tibiaAngle*180/PI << std::endl;
    
    return 0;
}