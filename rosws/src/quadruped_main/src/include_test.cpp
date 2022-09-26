#include <iostream>
#include <string.h>
#include <kinematics.h>

int dt = 10;

STATE robot_state = { 

    .dt = dt,
    .current_time = 0,

    //robot frame current pose   
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

int main(int argc, char *argv[]) {

/*	
	leg_FL.compute_IK_XYZ(0, 50, 0);
    	leg_BR.compute_IK_XYZ(0, 50, 0);
    	leg_FR.compute_IK_XYZ(0, 50, 0);
    	leg_BL.compute_IK_XYZ(0, 50, 0);
*/
	
	for (int i = 0; i < 100; ++i) {
		
		gait_controller(robot_state);

		std::cout << robot_state.ticks << std::endl;
	}
	return 0;
}
