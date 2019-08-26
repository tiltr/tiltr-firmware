/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef hall_encoders
#define hall_encoders

#include "Arduino.h"


#define hall_A_int PA10 //2
#define hall_B_int PB3  //3 
#define hall_C_int PB5  //4

#define r_hall_A_int PB4 //5 
#define r_hall_B_int PA8 //7
#define r_hall_C_int PA9 //8

class wheel_encoder
{
  public:
    wheel_encoder(char side);
    void dot();
    void dash();
  private:
    hall_A_pin
    hall_B_pin
    hall_C_pin
    int _pin;
};

#endif
