#include <BluetoothSerial.h>

#define RX1 9
#define TX1 10
#define RX2 16
#define TX2 17

char deli[2] = ",";

BluetoothSerial SerialBT;

unsigned long _startMillis;

int timedRead()
{
  int c;
  _startMillis = millis();
  do {
    c = SerialBT.read();
    if (c >= 0) return c;
  } 
  while(millis() - _startMillis < 1000);
  return -1;     // -1 indicates timeout
}

String readStringUntil(char terminator)
{
  String ret;
  int c = timedRead();
  while (c >= 0 && c != terminator)
  {
    ret += (char)c;
    c = timedRead();
  }
  return ret;
}

String MACadd = "98:D3:31:F9:FB:26";
uint8_t address[6]  = {0x98, 0xD3, 0x31, 0xF9, 0xFB, 0x26};
String name = "HC-05";
char *pin = "1234"; //<- standard pin would be provided by default
bool connected;

void setup() {
  
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);
  //Serial1.begin(115200, SERIAL_8N1, RX1, TX1);
  Serial.begin(115200, SERIAL_8N1);
  SerialBT.begin("ESP32test", true); 
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  
  connected = SerialBT.connect(address);
  //connected = SerialBT.connect(name);
  
  if(connected) {
    Serial.println("Connected Succesfully!");
    } 
  
  else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
}

void loop() {

  int array_read[6];
  char charbuff[29];
  String incomingString;

  if (SerialBT.available()) {
    incomingString = readStringUntil('\n');
  }

  //Serial.println(incomingString);

  incomingString.toCharArray(charbuff, 29);
  Serial.println(charbuff);

  char *token;
  token = strtok(charbuff, deli);

  for (int i = 0; i < 6; ++i) {
    if (token != NULL){
      array_read[i] = atoi(token);
      token = strtok(NULL, deli);
    //delay(100);
    }
  }
  for (int i = 0; i < 6; ++i) {
    //Serial.println(String(array_read[i]));
  }
  Serial.println();
  delay(10);
}
