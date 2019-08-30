
#include "bluetooth.h"
  
HardwareSerial Serial5(PD2, PC12);
SerialMessenger btSerial(Serial5);

void setup()
{
  btSerial.begin(57600);
  Serial.begin(9600);
}

struct btData btMessage;

void loop()
{

  btMessage = btSerial.checkForNewMessage('&');
  
  if (btMessage.id != '\0') {
    Serial.print(btMessage.id);
    Serial.print(": ");
    Serial.println(btMessage.value);
  }

}
