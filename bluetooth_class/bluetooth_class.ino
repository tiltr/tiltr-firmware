

#define MESSAGE_LENGTH 64

class SerialMessenger {
  public:
    SerialMessenger( HardwareSerial& device) {
      hwStream = &device;
    }
    void begin(uint32_t baudRate);
    char* checkForNewMessage(const char endMarker, bool errors);

  private:
    HardwareSerial* hwStream;
    Stream* stream;
    char incomingMessage[MESSAGE_LENGTH];
    size_t idx = 0;
};

char* SerialMessenger::parseMessage(char* incommingMessage) {
  char* separator = strchr(command, '=');
  if (separator != 0)
  {
    // Actually split the string in 2: replace ':' with 0
    *separator = 0;
    char ID = command[0];
    ++separator;
    int iPosition = atoi(separator);
    float fPosition = atof(separator);
    Serial.println(fPosition);
    Serial.println(iPosition);
    if (iPosition != 999) {
      switch (ID) {
      }
      // .....
    }
  }
}

char* SerialMessenger::checkForNewMessage(const char endMarker, bool errors = false)
{
  stream = hwStream;
  if (stream->available())
  {
    if (stream->peek() == '\r')
    {
      (void)stream->read();
      return nullptr;
    }
    incomingMessage[idx] = stream->read();
    if (incomingMessage[idx] == endMarker)
    {
      incomingMessage[idx] = '\0';
      idx = 0;
      return incomingMessage;
    }
    else
    {
      idx++;
      if (idx > MESSAGE_LENGTH - 1)
      {
        if (errors)
        {
          stream->print(F("{\"error\":\"message too long\"}\n"));
        }
        while (stream->read() != '&')
        {
          delay(50);
        }
        idx = 0;
        incomingMessage[idx] = '\0';
      }
    }
  }
  return nullptr;
}

void SerialMessenger::begin(uint32_t baudRate)
{

  hwStream->begin(baudRate);

}

HardwareSerial Serial5(PD2, PC12);
SerialMessenger btSerial(Serial5);


void setup()
{
  btSerial.begin(57600);
  Serial.begin(9600);
}


void loop()
{
  if (const char* btMessage = btSerial.checkForNewMessage('&'))
  {
    char newMessage[MESSAGE_LENGTH];
    strcpy(newMessage, btMessage);
    Serial.println(newMessage);
  }

}
