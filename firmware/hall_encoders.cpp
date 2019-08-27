#include "Arduino.h"
#include "hall_encoders.h"

wheel_encoder::wheel_encoder(char side)
{
  if (side == 'r' || side == 'R') {
    hall_A_int = r_hall_A_int;
    hall_B_int = r_hall_B_int;
    hall_C_int = r_hall_C_int;
    invert = 1;
  }
  else if (side == 'l' || side == 'L') {
    hall_A_int = l_hall_A_int;
    hall_B_int = l_hall_B_int;
    hall_C_int = l_hall_C_int;
    invert = -1;
  }
  timer = millis();
}

float wheel_encoder::get_velocity() {
  velocity = float(((float)wheel_arc_per_tick * ((float)counter - (float)last_counter)) / (1000 * ((float)millis() - (float)timer))) * 1000.0;
  velocity = velocity * (float)invert;
  last_counter = counter ;
  timer = millis();

  return velocity;
}

int wheel_encoder::get_counter() {
  return counter;
}

bool wheel_encoder::get_direction() {
  return forward;
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
