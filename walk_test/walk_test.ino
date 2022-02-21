#include "kinematics.h"

unsigned long currentMillis;
long previousMillis;

//====== LOOP Frequency =======
int dt = 10;

Kinematics leg_FL = Kinematics(1, dt);
Kinematics leg_FR = Kinematics(2, dt);
Kinematics leg_BL = Kinematics(3, dt);
Kinematics leg_BR = Kinematics(4, dt);

void setup() 
{

}

void loop () 
{
    currentMillis = millis();
    if (currentMillis - previousMillis > dt) 
    {
        previousMillis = currentMillis;
    } 
}
