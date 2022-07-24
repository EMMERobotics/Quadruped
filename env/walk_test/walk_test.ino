#include "kinematics.h"

//servo dependencies
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#define RX2 16
#define TX2 17

#define _ESP32 1
// ========================== new board ============================
//servo params
// #define SERVOMIN  72
// #define SERVOMAX  422
// #define SERVODIFF  350  // SERVOMAX - SERVOMIN
// #define SERVO_FREQ 50

// const int offset_tibia_1 = 16;
// const int offset_tibia_2 = 1;
// const int offset_tibia_3 = 18;
// const int offset_tibia_4 = 0;

// const int offset_femur_1 = -2;
// const int offset_femur_2 = 2;
// const int offset_femur_3 = 7;
// const int offset_femur_4 = -2;

// const int offset_waist_1 = -9;
// const int offset_waist_2 = -4;
// const int offset_waist_3 = 8;
// const int offset_waist_4 = 0;

// ========================= old board ======================
#define SERVOMIN  500
#define SERVOMAX  2500
#define SERVODIFF  2000 // SERVOMAX - SERVOMIN
#define SERVO_FREQ 50

const int offset_tibia_1 = 90;
const int offset_tibia_2 = 5;
const int offset_tibia_3 = 100;
const int offset_tibia_4 = 0;

const int offset_femur_1 = -10;
const int offset_femur_2 = 10;
const int offset_femur_3 = 40;
const int offset_femur_4 = -10;

const int offset_waist_1 = -50;
const int offset_waist_2 = -25;
const int offset_waist_3 = 45;
const int offset_waist_4 = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//========== Implement motor function here --- BUT this function will still be called in the compute_IK_XYZ() ===
void Leg::motor_arduino(float hipAngle, float femurAngle, float tibiaAngle) {

    float tibia_val;
    float femur_val;
    float waist_val;

    if(leg_i == FL || leg_i == BL) {
        tibia_val = (PI - tibiaAngle)/PI*SERVODIFF + SERVOMIN + tibia_offset;
        femur_val = (PI - femurAngle)/PI*SERVODIFF + SERVOMIN + femur_offset;
    }

    else {
        tibia_val = tibiaAngle/PI*SERVODIFF + SERVOMIN + tibia_offset;
        femur_val = femurAngle/PI*SERVODIFF + SERVOMIN + femur_offset;
    }

    if(leg_i == FL || leg_i == FR) {
        waist_val = (PI - hipAngle)/PI*SERVODIFF + SERVOMIN + waist_offset;
    }

    else {
        waist_val = hipAngle/PI*SERVODIFF + SERVOMIN + waist_offset;
    }

    String waist;
    String femur;
    String tibia;
    

    switch (leg_i)
    {
    case FL:
        waist = "#1P" + String(waist_val + offset_waist_1) + "T400" + "\r\n";
        femur = "#5P" + String(femur_val + offset_femur_1) + "T400" + "\r\n";
        tibia = "#9P" + String(tibia_val + offset_tibia_1) + "T400" + "\r\n";
        break;
    
    case FR:
        waist = "#2P" + String(waist_val + offset_waist_2) + "T400" + "\r\n";
        femur = "#6P" + String(femur_val + offset_femur_2) + "T400" + "\r\n";
        tibia = "#10P" + String(tibia_val + offset_tibia_2) + "T400" + "\r\n";
        break;

    case BL:
        waist = "#3P" + String(waist_val + offset_waist_3) + "T400" + "\r\n";
        femur = "#7P" + String(femur_val + offset_femur_3) + "T400" + "\r\n";
        tibia = "#11P" + String(tibia_val + offset_tibia_3) + "T400" + "\r\n";
        break;

    case BackR:

        waist = "#4P" + String(waist_val + offset_waist_4) + "T400" + "\r\n";
        femur = "#8P" + String(femur_val + offset_femur_4) + "T400" + "\r\n";
        tibia = "#12P" + String(tibia_val + offset_tibia_4) + "T400" + "\r\n";
        break;
    
    default:
        //RAISE ERROR 
        break;
    }

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
    Serial1.begin(9600, SERIAL_8N1, RX2, TX2);
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
