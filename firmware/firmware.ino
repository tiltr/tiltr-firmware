
#include <PID_v1.h>
/*
  Minimalistic example to to get the wheels turning.
  Look at HoverboardAPI.c to learn which other functions are available.

  Further information on https://github.com/bipropellant
*/


#define wheel_diameter 518.68
#define wheel_arc_per_tick 5.1868
#include <HoverboardAPI.h>


//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double last_setpoint = 0;

//Specify the links and initial tuning parameters
double Kp = 20, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



//InterruptIn hall_A_int(PA_10);

//HardwareSerial pc(USBTX, USBRX);
HardwareSerial Serial4(PA1, PA0);

#define hall_A_int PA10
#define hall_B_int PB3
#define hall_C_int PB5
#define LED PA5


//DigitalIn   hall_A_read(PA_10);
//InterruptIn hall_A_int(PA_10);
//DigitalIn   hall_B_read(PB_3);
//InterruptIn hall_B_int(PB_3);
//DigitalIn   hall_C_read(PB_5);
//InterruptIn hall_C_int(PB_5);

volatile bool forward = false;
volatile int counter = 0;

void hall_a_rise()
{
  forward = (digitalRead(hall_B_int) ? true : false);
  counter += (forward ? 1 : (-1));
  digitalWrite(LED, (forward ? HIGH : LOW));
}




void hall_a_change()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 10)
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
  digitalWrite(PA5, forward);
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
  //attachInterrupt(digitalPinToInterrupt(hall_B_int), hall_a_fall, FALLING);
  pinMode(LED, OUTPUT);
  Setpoint = 1;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}
String inString = "";
bool p_flag = true;
long timer = millis();
long timer_2 = millis();
int inByte = 100;
long last_counter = 0;
int speed_var = 300;
int speed_var_2 = 0;
float velocity = 0;
void print_velocity() {
  if (((millis() - timer) > 175)) {
    velocity = float(((float)wheel_arc_per_tick * ((float)counter - (float)last_counter)) / (1000 * ((float)millis() - (float)timer))) * 10000.0;
    //Serial.print((counter - last_counter));
    //Serial.print("  ");
    //Serial.print(1000*(millis() - timer));
    Serial.print("velocity = ");
    Serial.println(velocity * 1000);
    //Serial.print("\twheel_position = ");
    //Serial.println(wheel_arc_per_tick*(counter));
    last_counter = counter ;
    timer = millis();
  }
}
bool s_flag = false;
void loop() {
  //digitalWrite(LED, HIGH);
  delay(500);
  int upto = 10;
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

      Setpoint += 0.1;

      if (Setpoint > upto) {
        s_flag = false;

      }
    } else if (!s_flag)  {
      Setpoint -= 0.1;
      if (Setpoint < (upto*-1)) {
        s_flag = true;
      }
    }

    last_setpoint = Setpoint;

    Input = float(((float)wheel_arc_per_tick * ((float)counter - (float)last_counter)) / (1000 * ((float)millis() - (float)timer))) * -10000.0;
    myPID.Compute();
    speed_var_2 = Output;

    //    for (int i = 0; i < upto; i++) {
    //      hoverboard.sendPWM(i, 0, PROTOCOL_SOM_NOACK);
    //      print_velocity();
    //    }
    //
    //
    //    for (int i = upto; i > 0; i--) {
    //      hoverboard.sendPWM(i, 0, PROTOCOL_SOM_NOACK);
    //      print_velocity();
    //    }
    //
    //    for (int i = 0; i < upto; i++) {
    //      hoverboard.sendPWM((-1)*i, 0, PROTOCOL_SOM_NOACK);
    //      print_velocity();
    //    }
    //
    //    for (int i = upto; i > 0; i--) {
    //      hoverboard.sendPWM((-1)*i, 0, PROTOCOL_SOM_NOACK);
    //      print_velocity();
    //    }
    hoverboard.sendPWM(speed_var_2, 0, PROTOCOL_SOM_NOACK);
    //hoverboard.sendPWMData(10, speed_var, 100, 50, 10, PROTOCOL_SOM_NOACK);

    Serial.print("input: ");

    Serial.print(Input);
    Serial.print("setpoint: ");
    Serial.print(Setpoint);
    Serial.print("output: ");
    Serial.println(Output);

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
