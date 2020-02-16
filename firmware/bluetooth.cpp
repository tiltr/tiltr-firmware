
#include "bluetooth.h"


struct btData SerialMessenger::parseMessage(char* incommingMessage) {
  struct btData parsed_data;

  // Split the command in two values
  char* separator = strchr(incommingMessage, '=');

  //  if (separator != 0)
  //  {
  //    *separator = 0;
  char ID = incommingMessage[0];
  ++separator;
  float value = atof(separator);

  if (value != 999) {
    parsed_data.id = ID;
    parsed_data.value = value;
  }
  //}

  return parsed_data;
}

struct btData SerialMessenger::checkForNewMessage(const char endMarker)
{
  bool errors = false;
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


char* SerialMessenger::returnNewMessage(const char endMarker)
{
  bool errors = false;
  struct btData parsed_data;
  stream = hwStream;
  if (stream->available())
  {
    if (stream->peek() == '\r')
    {
      (void)stream->read();
      return "xx";//parsed_data;
    }
    incomingMessage[idx] = stream->read();
    if (incomingMessage[idx] == endMarker)
    {
      incomingMessage[idx] = '\0';
      idx = 0;
      //Serial.println(incomingMessage);
      //parsed_data = parseMessage(incomingMessage);
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
  return "xx";//parsed_data;
}

void SerialMessenger::begin(uint32_t baudRate)
{

  hwStream->begin(baudRate);

}
