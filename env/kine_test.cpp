#include <math.h>
#include "walk_test/kinematics.h"

//#define PI 3.14159

int dt = 10;
 
STATE robot_state = {
    .dt = dt,
    .current_time = 0,

    //robot frame curremt pose   
    .c_x = 0,
    .c_y = 0,
    .c_z = 0,
    .c_R = 0,
    .c_P = 0,
    .c_Y = 0,

    .p_x = 0,
    .p_y = 0,
    .p_z = 0,

    .ticks = 0,
    .mode = 0,
    .pairs = true

};


int main () {

    for (int i = 0; i<100; ++i){
        //leg_FL.compute_IK(leg_FL.state);
        //std::cout << leg_FL.hipAngle*180/PI << std::endl;
        //std::cout << "femur: " << int(leg_FL.femurAngle*180/PI) << std::endl;
        //std::cout << "tibia" << leg_FL.tibiaAngle*180/PI << std::endl;
        //std::cout << robot_state.ticks << std::endl;
        gait_controller(robot_state);

    }
    
    return 0;
}