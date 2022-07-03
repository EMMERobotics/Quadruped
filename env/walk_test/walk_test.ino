#include "kinematics.h"

unsigned long currentMillis;
long previousMillis;

//====== LOOP Frequency =======
int dt = 10;

// 2100 tibia 900 femur
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

String t = "T400";

void setup() {
    Serial.begin(9600);
    //default_init();

}

void tibia(int p) {
    String s1 = "#9P" + String(p + offset_tibia_1) + t;
    String s2 = "#10P" + String(p + offset_tibia_2) + t;
    String s3 = "#11P" + String(p + offset_tibia_3) + t;
    String s4 = "#12P" + String(p + offset_tibia_4) + t;
    Serial.print(s1 + "\r\n");
    Serial.print(s2 + "\r\n");
    Serial.print(s3 + "\r\n");
    Serial.print(s4 + "\r\n");
    
}
// 500 to 2500 ccw ref. from femur 1
void femur(int p) {
    int p_plus = p + 1000;
    String s1 = "#5P" + String(p_plus + offset_femur_1) + t;
    String s2 = "#6P" + String(p + offset_femur_2) + t;
    String s3 = "#7P" + String(p_plus + offset_femur_3) + t;
    String s4 = "#8P" + String(p + offset_femur_4) + t;
    Serial.print(s1+ "\r\n");
    Serial.print(s2+ "\r\n");
    Serial.print(s3+ "\r\n");
    Serial.print(s4+ "\r\n");
}

void waist(int p){
    String s1 = "#1P" + String(p + offset_waist_1) + t;
    String s2 = "#2P" + String(p + offset_waist_2) + t;
    String s3 = "#3P" + String(p + offset_waist_3) + t;
    String s4 = "#4P" + String(p + offset_waist_4) + t;
    Serial.print(s1+ "\r\n");
    Serial.print(s2+ "\r\n");
    Serial.print(s3+ "\r\n");
    Serial.print(s4+ "\r\n");
}

void default_init () {

  waist(1500);
  femur(1500);
  tibia(1500);

}

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
    default_init();
}

void loop () 
{
    currentMillis = millis();

    if (currentMillis - previousMillis > dt) 
    {
        previousMillis = currentMillis;
    } 

}