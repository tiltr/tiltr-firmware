#include "Arduino.h"
#include "hall_encoders.h"

wheel_encoder::wheel_encoder(char side)
{
  if (side == 'r' || side = 'R') {
    attachInterrupt(digitalPinToInterrupt(r_hall_B_int), r_hall_a_change, CHANGE);
    attachInterrupt(digitalPinToInterrupt(r_hall_C_int), r_hall_b_change, CHANGE);
    attachInterrupt(digitalPinToInterrupt(r_hall_A_int), r_hall_c_change, CHANGE);
  } else if (side == 'l' || side = 'L'){
    attachInterrupt(digitalPinToInterrupt(hall_B_int), hall_a_change, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hall_C_int), hall_b_change, CHANGE);
    attachInterrupt(digitalPinToInterrupt(hall_A_int), hall_c_change, CHANGE);
  }
  pinMode(pin, OUTPUT);
  _pin = pin;
}

void wheel_encoder::dot()
{
  digitalWrite(_pin, HIGH);
  delay(250);
  digitalWrite(_pin, LOW);
  delay(250);
}

void wheel_encoder::dash()
{
  digitalWrite(_pin, HIGH);
  delay(1000);
  digitalWrite(_pin, LOW);
  delay(250);
}


void wheel_encoder::hall_a_change()
{

  if (digitalRead(hall_A_int)) {
    forward = (digitalRead(hall_B_int) ? true : false);
  }
  if (!digitalRead(hall_A_int)) {
    forward = (digitalRead(hall_B_int) ? false : true);
  }
  counter += (forward ? 1 : (-1));

  digitalWrite(LED, (forward ? HIGH : LOW));
}

void wheel_encoder::hall_b_change()
{
  if (digitalRead(hall_B_int))
  {
    forward = (!digitalRead(hall_C_int) ? false : true);
  }
  if (!digitalRead(hall_B_int))
  {
    forward = (!digitalRead(hall_C_int) ? true : false);
  }
  counter += (forward ? 1 : (-1));
  digitalWrite(LED, (forward ? HIGH : LOW));
}

void wheel_encoder::hall_c_change()
{
  if (digitalRead(hall_C_int)) {
    forward = (!digitalRead(hall_A_int) ? false : true);
  }
  if (!digitalRead(hall_B_int)) {
    forward = (digitalRead(hall_A_int) ? true : false);
  }
  counter += (forward ? 1 : (-1));
  digitalWrite(LED, (forward ? HIGH : LOW));


}
