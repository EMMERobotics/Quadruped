// 2100 tibia 900 femur
const int offset_tibia_1 = 30;
const int offset_tibia_2 = 5;
const int offset_tibia_3 = 40;
const int offset_tibia_4 = 0;

const int offset_femur_1 = -10;
const int offset_femur_2 = 10;
const int offset_femur_3 = 40;
const int offset_femur_4 = -10;

const int offset_waist_1 = -35;
const int offset_waist_2 = -25;
const int offset_waist_3 = 15;
const int offset_waist_4 = 75;

String t = "T400";

void setup() {
    Serial.begin(9600);
    //default_init();

}

//(-)increase the angle between femur and tibia
//(+)decrease the angle between femur and tibia
//ROM (need more info)
void tibia(float degree) {

  int p1;
  int p2;
  p1 = (2000 * degree)/180 + 500;
  p2 = (2000 * (180 - degree))/180 + 500;
  
    Serial.print("#9P" + String(p1 + offset_tibia_1) + t + "\r\n");
    Serial.print("#10P" + String(p2 + offset_tibia_2) + t + "\r\n");
    Serial.print("#11P" + String(p1 + offset_tibia_3) + t + "\r\n");
    Serial.print("#12P" + String(p2 + offset_tibia_4) + t + "\r\n");
    
}

//(-) move femur backward
//(+) move femur forward
//ROM 
void femur(float degree) {

  int p1;
  int p2;
  p1 = (2000 * degree)/180 + 500;
  p2 = (2000 * (180 - degree))/180 + 500;
  
    Serial.print("#5P" + String(p2 + offset_femur_1) + t + "\r\n");
    Serial.print("#6P" + String(p1 + offset_femur_2) + t + "\r\n");
    Serial.print("#7P" + String(p2 + offset_femur_3) + t + "\r\n");
    Serial.print("#8P" + String(p1 + offset_femur_4) + t + "\r\n");
}

//(-) move waist right
//(+) move waist left
//ROM 
void waist(float degree){
  
  int p1;
  int p2;
  p1 = (2000 * degree)/180 + 500;
  p2 = (2000 * (180 - degree))/180 + 500;
  
    Serial.print("#1P" + String(p1 + offset_waist_1) + t + "\r\n");
    Serial.print("#2P" + String(p1 + offset_waist_2) + t + "\r\n");
    Serial.print("#3P" + String(p2 + offset_waist_3) + t + "\r\n");
    Serial.print("#4P" + String(p2 + offset_waist_4) + t + "\r\n");
}

void loop () {
  
  femur(45);
  tibia(90);
  waist(90);

}
