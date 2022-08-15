#include "kinematics.h"

//servo dependencies
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define RX2 16
#define TX2 17

#define _ESP32 1 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ========================== new board ============================
//servo params
/*
#define SERVOMIN  72
#define SERVOMAX  422
#define SERVODIFF  350  // SERVOMAX - SERVOMIN
#define SERVO_FREQ 50
*/
#define SERVOMIN  600
#define SERVOMAX  3300
#define SERVODIFF  2700  // SERVOMAX - SERVOMIN
#define SERVO_FREQ 330


//========== Implement motor function here --- BUT this function will still be called in the compute_IK_XYZ() ===
void Leg::motor_arduino(float hipAngle, float femurAngle, float tibiaAngle) {

    int tibia_val;
    int femur_val;
    int waist_val;
    /*
    if(leg_i == FL || leg_i == BL) {
        tibia_val = (PI - tibiaAngle)/PI*SERVODIFF + SERVOMIN + tibia_offset;
        femur_val = (PI/2 + femurAngle)/PI*SERVODIFF + SERVOMIN + femur_offset;
    }

    else {
        tibia_val = tibiaAngle/PI*SERVODIFF + SERVOMIN + tibia_offset;
        femur_val = (PI/2 - femurAngle)/PI*SERVODIFF + SERVOMIN + femur_offset;
    }

    if(leg_i == FL || leg_i == FR) {
        waist_val = (PI - hipAngle)/PI*SERVODIFF + SERVOMIN + waist_offset;
    }

    else {
        waist_val = hipAngle/PI*SERVODIFF + SERVOMIN + waist_offset;
    }
    */

    if(leg_i == FL || leg_i == BL) {
        tibia_val = tibiaAngle/PI*SERVODIFF + SERVOMIN + tibia_offset;
        femur_val = femurAngle/PI*SERVODIFF + SERVOMIN + femur_offset;
    }

    else {
        tibia_val = (PI - tibiaAngle)/PI*SERVODIFF + SERVOMIN + tibia_offset;
        femur_val = (PI - femurAngle)/PI*SERVODIFF + SERVOMIN + femur_offset;
    }

    if(leg_i == FL || leg_i == FR) {
        waist_val = hipAngle/PI*SERVODIFF + SERVOMIN + waist_offset;
    }

    else {
        waist_val = (PI - hipAngle)/PI*SERVODIFF + SERVOMIN + waist_offset;
    }

    //Serial.println("WaistID: " + String(waist_motor_id) + " " + String(waist_val));
//    Serial.println("femurID: " + String(femur_motor_id) + " " + String(femur_val));
    //Serial.println("tibiaID: " + String(tibia_motor_id) + " " + String(tibia_val));
    
    pwm.setPWM(waist_motor_id, 0, waist_val);
    pwm.setPWM(femur_motor_id, 0, femur_val);
    pwm.setPWM(tibia_motor_id, 0, tibia_val);

}

unsigned long currentMillis;
unsigned long previousMillis;
unsigned long start_time;

//====== LOOP Frequency =======
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

void setup() {
    
    //servo shelid init
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    //pwm.setPWM(4, 0, 246);

//    Serial.begin(115200, SERIAL_8N1);
    #if _ESP32 == 1
    //Serial1.begin(9600, SERIAL_8N1, RX2, TX2);
    #endif
    start_time = millis();
    stand(robot_state);
    
}

/* =========  OLD BOARD, NOT USING ANYMORE ======================= 
void Leg::SerialParser(String motor_id, int pos, int time) {
    String str_time = "T"+ String(time);
    String str_pos = String(pos);
    String msg = motor_id + str_pos + str_time + "\r\n";
    Serial.print(msg);
    #if _ESP32 == 1
    Serial1.print(msg);
    #endif
}
*/

void test_IK(STATE state, int x, int y, int z) {
    leg_FL.compute_IK_XYZ(x, y, z);
    leg_BR.compute_IK_XYZ(x, y, z);
    leg_FR.compute_IK_XYZ(x, y, z);
    leg_BL.compute_IK_XYZ(x, y, z);
}

void test_IK_FL(STATE state, int x, int y, int z) {
    leg_FL.compute_IK_XYZ(x, y, z);
    // leg_BR.compute_IK_XYZ(0, 50, 0);
    // leg_FR.compute_IK_XYZ(0, 50, 0);
    // leg_BL.compute_IK_XYZ(0, 50, 0);
}
int sw = 0;
int start = 0;
int d_time = 50;
int dis = 40;
void loop () 
{
    
    
    if (start == 0) {
      
        start = 1;
        for (int y = 0; y < dis; y++) {
            test_IK(robot_state, 0, y, 0);
            delay(d_time);
        }  
    /*
        for (int x = 0; x < dis; x++) {
            test_IK_FL(robot_state, x, dis, 0);
            delay(d_time);
        }  */
    }
    if (sw == 0) {
        sw = 1;
        /*
        for (int x = -dis; x < dis; x++) {
            test_IK_FL(robot_state, x, dis, 0);
            delay(d_time);
        }  */
        for (int y = dis; y > -dis; y--) {
            test_IK(robot_state, dis, y, 0);
            delay(d_time);
        }  
    }
    if (sw == 1) {
        // sw = 2;
        /*
        for (int y = dis; y > -dis; y--) {
            test_IK_FL(robot_state, dis, y, 0);
            delay(d_time);
        }  */
        sw = 0;
        for (int y = -dis; y < dis; y++) {
            test_IK(robot_state, dis, y, 0);
            delay(d_time);
        }  
    }
    /*
    if (sw == 2) {
        sw = 3;
        for (int x = dis; x > -dis; x--) {
            test_IK_FL(robot_state, x, -dis, 0);
            delay(d_time);
        }  
    }
    if (sw == 3) {
        sw = 0;
        for (int y = -dis; y < dis; y++) {
            test_IK_FL(robot_state, -dis, y, 0);
            delay(d_time);
        }  
    }*/

//    delay(1000);
//    test_IK(robot_state);
    

}