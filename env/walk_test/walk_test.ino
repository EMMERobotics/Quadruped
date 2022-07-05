#include "kinematics.h"

#define RX2 16
#define TX2 17

#define _ESP32 1

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

}

void Leg::SerialParser(String motor_id, int pos, int time) {
    String str_time = "T"+ String(time);
    String str_pos = String(pos);
    String msg = motor_id + str_pos + str_time + "\r\n";
    Serial.print(msg);
    #if _ESP32 == 1
    Serial1.print(msg);
    #endif
}

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
