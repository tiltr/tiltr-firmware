

#define MESSAGE_LENGTH 64

class SerialMessenger {
  public:
    SerialMessenger( HardwareSerial& device) {
      hwStream = &device;
    }
    void begin(uint32_t baudRate);
    struct btData checkForNewMessage(const char endMarker, bool errors);

  private:
    struct btData parseMessage(char* incommingMessage);
    HardwareSerial* hwStream;
    Stream* stream;
    char incomingMessage[MESSAGE_LENGTH];
    size_t idx = 0;
};



struct btData {
  bool bSensitivity = false, bRoll = false, bPitch = false, bX_position = false;
  bool isZero;
  int sensitivity = 0;
  float roll = 0;
  float pitch = 0;
  float x_position = 0.0;
  char id;
  float value;
};

struct btData last_data;
struct btData SerialMessenger::parseMessage(char* incommingMessage) {
  struct btData parsed_data;

  // Split the command in two values
  char* separator = strchr(incommingMessage, '=');

  // If we're not at the end of the array?
  if (separator != 0)
  {
    // Actually split the string in 2: replace ':' with 0
    *separator = 0;
    char ID = incommingMessage[0];
    ++separator;
    int iPosition = atoi(separator);
    float fPosition = atof(separator);

    if (iPosition != 999) {
      parsed_data.id = ID;
      parsed_data.value = fPosition;
    }

  }

  return parsed_data;
}

struct btData SerialMessenger::checkForNewMessage(const char endMarker, bool errors = false)
{
  struct btData parsed_data;
  stream = hwStream;
  if (stream->available())
  {
    if (stream->peek() == '\r')
    {
      (void)stream->read();
      return parsed_data;
    }
    incomingMessage[idx] = stream->read();
    if (incomingMessage[idx] == endMarker)
    {
      incomingMessage[idx] = '\0';
      idx = 0;
      //Serial.println(incomingMessage);
      parsed_data = parseMessage(incomingMessage);
      return parsed_data;
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

  return parsed_data;
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
