#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

String MACadd = "98:D3:31:F9:FB:26";
uint8_t address[6]  = {0x98, 0xD3, 0x31, 0xF9, 0xFB, 0x26};
String name = "HC-05";
char *pin = "1234"; //<- standard pin would be provided by default
bool connected;

void setup() {
  Serial.begin(115200);
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
  
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  delay(20);
}
