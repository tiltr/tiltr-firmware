/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef hall_encoders
#define hall_encoders

#include "Arduino.h"
#define LED PA5
#define wheel_diameter 518.68
#define wheel_arc_per_tick 5.1868

const int l_hall_A_int = 2;
const int l_hall_B_int = 3;
const int l_hall_C_int = 4;

const int r_hall_A_int = 5;
const int r_hall_B_int = 7;
const int r_hall_C_int = 8;


class wheel_encoder
{
  public:
    wheel_encoder(char side);
    void dot();

    int get_counter();
    bool get_direction();
    void dash();
    volatile bool forward = false;
    volatile int counter = 0;
    int hall_A_int;
    int hall_B_int;
    int hall_C_int;
    void hall_a_change();
    void hall_b_change();
    void hall_c_change();
    float get_velocity();
    float velocity;
    long timer;
  private:
    long last_counter;
    int invert ;

    int _pin;
};

#endif
