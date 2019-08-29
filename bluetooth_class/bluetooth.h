#ifndef bluetooth_comms
#define bluetooth_comms

#include "Arduino.h"


#define MESSAGE_LENGTH 64


struct btData {
  char id = '\0';
  float value;
};

class SerialMessenger {
  public:
    //SerialMessenger( HardwareSerial& device);
    SerialMessenger( HardwareSerial& device) {
      hwStream = &device;
    }

    void begin(uint32_t baudRate);
    struct btData checkForNewMessage(const char endMarker);

  private:
    struct btData parseMessage(char* incommingMessage);
    HardwareSerial* hwStream;
    Stream* stream;
    char incomingMessage[MESSAGE_LENGTH];
    size_t idx = 0;
};



#endif
