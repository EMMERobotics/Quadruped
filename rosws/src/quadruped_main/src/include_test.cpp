#include <kinematics.h>

int main(int argc, char *argv[]) {
	
	leg_FL.compute_IK_XYZ(0, 50, 0);
    	leg_BR.compute_IK_XYZ(0, 50, 0);
    	leg_FR.compute_IK_XYZ(0, 50, 0);
    	leg_BL.compute_IK_XYZ(0, 50, 0);
	
	return 0;

}
