#include "kinematics.h"

//servo dependencies
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define RX2 16
#define TX2 17

#define PI 3.14159

#define _ESP32 1 

#if _ESP32 == 1
#include <Ps3Controller.h>
#endif

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
/* =================== PS3 CONTROLLER PARAMS ==================== */
#define STICKDEADZONE 15
char* ps3_address = "60:f4:94:34:67:5e";

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
    //Serial.println("femurID: " + String(femur_motor_id) + " " + String(femur_val));
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

 COMMAND command {
    
    .v_x = 0,
    .v_y = 0,
    .v_z = 0,
    .roll = 0,
    .pitch = 0,
    .yaw = 0

};

#if _ESP32 == 1
void get_command(COMMAND &command) {
    command.v_x = Ps3.data.analog.stick.ly;
    command.v_y = Ps3.data.analog.stick.lx;
    command.v_z = Ps3.data.analog.stick.ry;

    command.yaw = Ps3.data.analog.stick.rx;

   

    if (command.v_x < STICKDEADZONE && command.v_x > -STICKDEADZONE) {
        command.v_x = 0;
    }

    if (command.v_y < STICKDEADZONE && command.v_y > -STICKDEADZONE) {
        command.v_y = 0;
    }

    if (command.yaw < STICKDEADZONE && command.yaw > -STICKDEADZONE) {
        command.yaw = 0;
    }

    command.v_x = map(command.v_x, -128, 128, 40, -40);
    command.v_y = map(command.v_y, -128, 128, -40, 40);
    command.v_z = map(command.v_z, -128, 128, -40, 40);
    command.yaw = map(command.yaw, -128, 128, -PI/4, PI/4);

    Serial.println(command.v_x);

}
#endif

float a = 0;
float b = 0;
float c = 0;

void setup() {
       
    //servo shelid init
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    //pwm.setPWM(4, 0, 246);

    Serial.begin(115200, SERIAL_8N1);
    #if _ESP32 == 1
    //Serial1.begin(9600, SERIAL_8N1, RX2, TX2);

    Ps3.begin(ps3_address);

    while (!Ps3.isConnected()) {
        Serial.println("Controller is NOT connected!");
        delay(500);
    }

    Serial.println("Controller is connected");

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

void test_IK(COMMAND command) {
    leg_FL.compute_IK_XYZ(command.v_x, command.v_y, command.v_z);
    leg_BR.compute_IK_XYZ(command.v_x, command.v_y, command.v_z);
    leg_FR.compute_IK_XYZ(command.v_x, command.v_y, command.v_z);
    leg_BL.compute_IK_XYZ(command.v_x, command.v_y, command.v_z);

    /*for testing
      Serial.println("V_X: " + String(command.v_x));
      Serial.println("V_Y: " + String(command.v_y));
      Serial.println("V_Z: " + String(command.v_z));
      */
}

void loop () 
{
    currentMillis = millis();
    unsigned long et = currentMillis - start_time;



    if (currentMillis - previousMillis > dt)
    {
      //init the robot
      get_command(command);
//      test_IK(command);  //<========== !DANGER!  DON'T ENABLE BOTH test_IK() and gait_controller() AT THE SAME TIME
      //gait_controller(robot_state, command);
      yaw_stance(command, a, b, c);
      Serial.println(x);
      Serial.println(y);
      Serial.println(z);
      Serial.println("+++++++++++++++++");
      
        if(et > 5000) {

            //gait_controller(robot_state);

        }
        previousMillis = currentMillis;
    } 

}
