#include "kinematics.h"

unsigned long currentMillis;
long previousMillis;

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


void setup() 
{

  Serial.begin(115200);

}

void loop () 
{
    currentMillis = millis();

    if (currentMillis - previousMillis > dt) 
    {
        previousMillis = currentMillis;
    } 
}
