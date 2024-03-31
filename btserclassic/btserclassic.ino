#include "BluetoothSerial.h"
   
BluetoothSerial SerialBT;
    
void setup()
{
  SerialBT.begin("ESP32test");
  delay(1000);
}
    
void loop()
{
  String inputFromOtherSide;
  SerialBT.printf("%f %f\n" , (millis()/1000.0f) , (millis()/1000.0f));
  delayMicroseconds(15000); 
}