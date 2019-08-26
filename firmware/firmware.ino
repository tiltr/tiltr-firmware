
#include <PID_v1.h>
/*
  Minimalistic example to to get the wheels turning.
  Look at HoverboardAPI.c to learn which other functions are available.

  Further information on https://github.com/bipropellant
*/


#define wheel_diameter 518.68
#define wheel_arc_per_tick 5.1868
#include <HoverboardAPI.h>
#include <Filters.h>

// filters out changes faster that 5 Hz.
float filterFrequency = 5.0;

// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilter( LOWPASS, filterFrequency );



//Define Variables we'll be connecting to
double Setpoint = 0.2, Input, Output;
double last_setpoint = 0;
double max_PID_output = 150;

//Specify the links and initial tuning parameters
double Kp = 280, Ki = 10, Kd = 2; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



//InterruptIn hall_A_int(PA_10);

//HardwareSerial pc(USBTX, USBRX);
HardwareSerial Serial4(PA1, PA0);

#define hall_A_int PA10
#define hall_B_int PB3
#define hall_C_int PB5

#define r_hall_A_int PB4
#define r_hall_B_int PA8
#define r_hall_C_int PA9
#define LED PA5



//DigitalIn   hall_A_read(PA_10);
//InterruptIn hall_A_int(PA_10);
//DigitalIn   hall_B_read(PB_3);
//InterruptIn hall_B_int(PB_3);
//DigitalIn   hall_C_read(PB_5);
//InterruptIn hall_C_int(PB_5);

volatile bool forward = false;
volatile bool r_forward = false;
volatile int counter = 0;

volatile int r_counter = 0;

void hall_a_rise()
{
  forward = (digitalRead(hall_B_int) ? true : false);
  counter += (forward ? 1 : (-1));
  digitalWrite(LED, (forward ? HIGH : LOW));
}




void hall_a_change()
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

void hall_b_change()
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

void hall_c_change()
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


void r_hall_a_change()
{

  if (digitalRead(r_hall_A_int)) {
    r_forward = (digitalRead(r_hall_B_int) ? true : false);
  }
  if (!digitalRead(r_hall_A_int)) {
    r_forward = (digitalRead(r_hall_B_int) ? false : true);
  }
  r_counter += (r_forward ? 1 : (-1));
 // digitalWrite(LED, (r_forward ? HIGH : LOW));

}

void r_hall_b_change()
{
  if (digitalRead(r_hall_B_int))
  {
    r_forward = (!digitalRead(r_hall_C_int) ? false : true);
  }
  if (!digitalRead(r_hall_B_int))
  {
    r_forward = (!digitalRead(r_hall_C_int) ? true : false);
  }
  r_counter  += (r_forward ? 1 : (-1));
  //digitalWrite(LED, (r_forward ? HIGH : LOW));

}

void r_hall_c_change()
{
  if (digitalRead(r_hall_C_int)) {
    r_forward = (!digitalRead(r_hall_A_int) ? false : true);
  }
  if (!digitalRead(hall_B_int)) {
    r_forward = (digitalRead(r_hall_A_int) ? true : false);
  }
  r_counter  += (r_forward ? 1 : (-1));
  //digitalWrite(LED, (r_forward ? HIGH : LOW));

}


int serialWrapper(unsigned char *data, int len) {
  return (int) Serial4.write(data, len);
}
HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);

void setup() {
  Serial.begin(9600);
  Serial4.begin(115200);

  //attachInterrupt(digitalPinToInterrupt(hall_B_int), hall_a_rise, RISING);
  attachInterrupt(digitalPinToInterrupt(hall_B_int), hall_a_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hall_C_int), hall_b_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hall_A_int), hall_c_change, CHANGE);

  attachInterrupt(digitalPinToInterrupt(r_hall_B_int), r_hall_a_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(r_hall_C_int), r_hall_b_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(r_hall_A_int), r_hall_c_change, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(hall_B_int), hall_a_fall, FALLING);
  pinMode(LED, OUTPUT);
  Setpoint = 0.1;

  myPID.SetOutputLimits((-1)*max_PID_output, max_PID_output);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}
String inString = "";
bool p_flag = true;
long timer = millis();
long timer_2 = millis();
int inByte = 100;
long last_counter = 0;
long r_last_counter = 0;
int speed_var = 300;
int speed_var_2 = 0;
float velocity = 0;
float r_velocity = 0;
int loop_delay = 2;
void print_velocity() {
  if (((millis() - timer) > 175)) {
    velocity = float(((float)wheel_arc_per_tick * ((float)counter - (float)last_counter)) / (1000 * ((float)millis() - (float)timer))) * 10000.0;
    r_velocity = float(((float)wheel_arc_per_tick * ((float)r_counter - (float)r_last_counter)) / (1000 * ((float)millis() - (float)timer))) * 10000.0;

    //Serial.print((counter - last_counter));
    //Serial.print("  ");
    //Serial.print(1000*(millis() - timer));
    Serial.print("velocity = ");
    Serial.print(velocity * 1000);
    Serial.print(" r_velocity = ");
    Serial.println(r_velocity * 1000);
    //Serial.print("\twheel_position = ");
    //Serial.println(wheel_arc_per_tick*(counter));
    last_counter = counter ;
    r_last_counter = r_counter ;
    timer = millis();
  }
}
bool s_flag = false;
void loop() {
  //digitalWrite(LED, HIGH);
  delay(500);
  //float upto = 0.3;
  int upto = 800;
  while (1) {
    //    while (Serial.available() > 0) {
    //      // hoverboard.sendBuzzer((8), 1, 100, PROTOCOL_SOM_NOACK);
    //
    //      int inChar = Serial.read();
    //      if (isDigit(inChar)) {
    //        hoverboard.sendBuzzer((14), 1, 100, PROTOCOL_SOM_NOACK);
    //
    //        // convert the incoming byte to a char and add it to the string:
    //        inString += (char)inChar;
    //      } else {
    //        hoverboard.sendBuzzer((20), 1, 100, PROTOCOL_SOM_NOACK);
    //
    //        Setpoint = inString.toInt();
    //        inString = "";
    //      }
    //    }

    if (s_flag) {

      Setpoint += 0.01;

      if (Setpoint > upto) {
        s_flag = false;

      }
    } else if (!s_flag)  {
      Setpoint -= 0.01;
      if (Setpoint < (upto * -1)) {
        s_flag = true;
      }
    }

    last_setpoint = Setpoint;

    float dt = ((float)millis() - (float)timer) * 1000.0;
    float dx = (((float)counter - (float)last_counter)) * (float)wheel_arc_per_tick;
    dx = lowpassFilter.input( dx );

    Input = (dx / dt) * (-1000);
    timer = millis();
    last_counter = counter;


    //Input = float(((float)wheel_arc_per_tick * ((float)counter - (float)last_counter)) / (1000 * ((float)millis() - (float)timer))) * -10000.0;


    for (int i = 0; i < upto; i++) {
      hoverboard.sendPWM(i, (-1)*i, PROTOCOL_SOM_NOACK);
      delay(loop_delay);
      print_velocity();
    }


    for (int i = upto; i > 0; i--) {
      hoverboard.sendPWM(i, (-1)*i, PROTOCOL_SOM_NOACK);
      delay(loop_delay);
      print_velocity();
    }

    for (int i = 0; i < upto; i++) {
      hoverboard.sendPWM((-1)*i, i, PROTOCOL_SOM_NOACK);
      delay(loop_delay);
      print_velocity();
    }

    for (int i = upto; i > 0; i--) {
      hoverboard.sendPWM((-1)*i, i, PROTOCOL_SOM_NOACK);
      delay(loop_delay);
      print_velocity();
    }
    //hoverboard.sendPWM(speed_var_2, speed_var_2, PROTOCOL_SOM_NOACK);
    //hoverboard.sendPWMData(10, speed_var, 100, 50, 10, PROTOCOL_SOM_NOACK);

    //
    //    //digitalWrite(LED, forward);
    //    if ((millis() - timer) > 20) {
    //      //  speed_var = speed_var * (-1);
    //      float velocity = (counter - last_counter) / (millis() - timer);
    //      Serial.print("velocity = ");
    //      Serial.println(velocity);
    //      last_counter = counter ;
    //      timer = millis();
    //    }

    if (((millis() - timer_2) > 1000) && p_flag) {
      //void HoverboardAPI::sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint16_t buzzerLen, char som) {
      //for (int i = 0; i < 10; i++) {
      hoverboard.sendBuzzer((8 + (1 * 2)), 1, 100, PROTOCOL_SOM_NOACK);
      //delay(50);
      //}
      //      for (int i = 10; i > 0; i--) {
      //        hoverboard.sendBuzzer((8 + (i * 2)), 1, 100, PROTOCOL_SOM_NOACK);
      //        delay(100);
      //      }
      Serial4.println("unlockASCII");
      Serial4.println('P');
      p_flag = false;

    }
  }
  //delay(300);
}
