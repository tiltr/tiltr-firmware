#include "Arduino.h"
#include "hall_encoders.h"

wheel_encoder::wheel_encoder(bool right)
{
  if (right){
    
  }
  pinMode(pin, OUTPUT);
  _pin = pin;
} 

void Morse::dot()
{
  digitalWrite(_pin, HIGH);
  delay(250);
  digitalWrite(_pin, LOW);
  delay(250);  
}

void Morse::dash()
{
  digitalWrite(_pin, HIGH);
  delay(1000);
  digitalWrite(_pin, LOW);
  delay(250);
}
