/* 
All rights reserved 
Written by Kasipat Wongsamran, 2021

-- IMPORTANT NOTICE --
- This code is for the robot which its femur and tibia are the same lenght ONLY.
- Theta angle is the angle inside the Isosceles triangle of the femur, tibia, and leg displacement.
- Phi is the actual motor input angle -- IN DEGREE -- (motor frame of reference)
- Others angle will be named angleX
*/

#include <stdio.h>
#include <math.h>

#define PI 3.1415926

//robot parameter
float femur = 100.0; // femur == tibia only 
float horizontal_offset = 10.0; // for offsetting y-axis of the robot
float vertical_offset = 0.0;
int leg_FR = 0;
int leg_FL = 1;
int leg_BR = 2;
int leg_BL = 3;

int compute_theta(float x, float y, float z) {

    float thetaA; // two smaller angle
    float phiB; // one large angle which is always equal to thetaB
    float leg_dis; // leg displacement
    float angleA; // rotating leg angle (X-axis) from 90 deg leg to ground displacement
    float angleB; // rotating leg angle (Y-axis) from 90 deg leg to gorund displacement
    float hip_dis; // hip to toe displacement
    float phiA; //femur angle
    float phiC; //hip angle
    float leg_len; 

    // Y-axis
    hip_dis = pow((z + vertical_offset), 2) + pow(y +  horizontal_offset, 2); //squared
    leg_dis = sqrt(hip_dis - pow(horizontal_offset, 2));
    hip_dis = sqrt(hip_dis);

    angleB = acos(horizontal_offset/hip_dis) - ((PI/2) - atan((y +  horizontal_offset)/(z + vertical_offset)));

    // X-axis
    angleB = atan(x/z); // for adding to the motor ref angle
    printf("angleB: %f\n", angleB);
    
    leg_dis = sqrt(pow(leg_dis,2) + pow(x,2));
    printf("leg_dis: %f\n", leg_dis);

    // Z-Axis  
    thetaA = acos(leg_dis/(2*femur));
    printf("thetaA: %f\n", thetaA);

    phiB = PI - (2*thetaA);
    phiA = ((PI/2) -thetaA) + angleB;
    printf("PhiA: %f", phiA);
    
    return 0;

}

int main() {

    float x_in = 0.0;
    float y_in = 0.0;
    float z_in = 130.0;
    
    int a = compute_theta(x_in, y_in, z_in);    

    return 0;
}