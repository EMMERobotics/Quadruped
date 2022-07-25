#include "kinematics.h"

//servo dependencies
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define RX2 16
#define TX2 17

#define _ESP32 1 

// ========================== new board ============================
//servo params
#define SERVOMIN  72
#define SERVOMAX  422
#define SERVODIFF  350  // SERVOMAX - SERVOMIN
#define SERVO_FREQ 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//========== Implement motor function here --- BUT this function will still be called in the compute_IK_XYZ() ===
void Leg::motor_arduino(float hipAngle, float femurAngle, float tibiaAngle) {

    float tibia_val;
    float femur_val;
    float waist_val;

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

void setup() {
    Serial.begin(9600, SERIAL_8N1);
    #if _ESP32 == 1
    //Serial1.begin(9600, SERIAL_8N1, RX2, TX2);
    #endif
    start_time = millis();
    stand(robot_state);

    //servo shelid init
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    

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


void loop () 
{
    currentMillis = millis();
    unsigned long et = currentMillis - start_time;

    if (currentMillis - previousMillis > dt) 
    //init the robot
    {
        if(et > 5000) {
            gait_controller(robot_state);   
        }
        previousMillis = currentMillis;
    } 

}
