#include "bluetooth.h"
#include <PID_v1.h>
#include <hall_encoders.h>
#include <HoverboardAPI.h>
#include <Filters.h>

#define LED PA5

#define print_help

HardwareSerial Serial5(PD2, PC12);
SerialMessenger btSerial(Serial5);

HardwareSerial Serial3(PC11, PC10);
SerialMessenger tuningSerial(Serial3);

wheel_encoder left_encoder('L');
wheel_encoder right_encoder('R');

HardwareSerial Serial4(PA1, PA0);
int serialWrapper(unsigned char *data, int len) {
  return (int) Serial4.write(data, len);
}

HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);

double Setpoint, Input, Output;
double Kp = 45, Ki = 5, Kd = 0.9;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void l_hall_a_change() {
  left_encoder.hall_a_change();
}
void l_hall_b_change() {
  left_encoder.hall_b_change();
}
void l_hall_c_change() {
  left_encoder.hall_c_change();
}

void r_hall_a_change() {
  right_encoder.hall_a_change();
}
void r_hall_b_change() {
  right_encoder.hall_b_change();
}
void r_hall_c_change() {
  right_encoder.hall_c_change();
}

void get_mpu_data()
{
  float imu_data = get_imu_data(0);
  //Serial.println(imu_data);
}


/*-----( PID variables )------*/
//Define Variables we'll be connecting to
float aHome = 169.8; //after LiPo fitted in the chassis
double pKp = 7.0 , pKi = 0.0, pKd = 0.01;
//pre ollysdouble pKp = 10.0 , pKi = 0.0, pKd = 0.08;
double aKp = 45.0, aKi = 5.0, aKd = 0.9;

double aInput, aOutput;
double pSetpoint = 0.0, pInput = 0.0, pOutput = 0.0;
double aSetpoint = aHome;
float aOutputMax = 90.0;
float aOutputMin = -90.0;
float pOutputMax = 7.5;
float pOutputMin = -7.5;

PID PIDp(&pInput, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT);
PID PIDa(&aInput, &aOutput, &aSetpoint, aKp, aKi, aKd, DIRECT);


bool stopFlag = false, printTimeFlag = false;
bool pFlag = false, iFlag = false, dFlag = false;
int iTune = 99;
bool printFlag = false, printFlag7 = true, printFlag8 = true;
bool tuneA = false, tuneP = false;
bool posLimitUpdate = false;
void process_data (const char * data)
{
  // for now just display it
  // (but you could compare it to some value, convert to an integer, etc.)
  Serial3.println (data);


  if (posLimitUpdate) {
    pOutputMax = Serial3.parseFloat();
    pOutputMin = pOutputMax * (-1);
    PIDp.SetOutputLimits(pOutputMin, pOutputMax);
    PIDp.SetMode(AUTOMATIC);
    Serial3.print("Angle offset allowed is now: ");
    Serial3.print(pOutputMin);
    Serial3.print(" to ");
    Serial3.println(pOutputMax);
    posLimitUpdate = false;
  }

  //    if (angleLimitUpdate){
  //    aOutputMax = Serial3.parseFloat();
  //    aOutputMin = pOutputMax * (-0.1);
  //    PIDa.SetOutputLimits(aOutputMin, aOutputMax);
  //    PIDa.SetMode(AUTOMATIC);
  //  }



  if ((iTune != 99) && (tuneA)) {
    switch (iTune) {
      case 0:
        aKp = Serial3.parseFloat();
        iTune = 99;
        break;
      case 1:
        aKi = Serial3.parseFloat();
        iTune = 99;
        break;
      case 2:
        aKd = Serial3.parseFloat();
        iTune = 99;
        break;
      case 5:
        aHome = Serial3.parseFloat();
        iTune = 99;
        break;
    }
    delay(2);
    PIDa.SetTunings(aKp, aKi, aKd);
    Serial3.print("New PID values: Kp = ");
    Serial3.print(aKp);
    Serial3.print(" Ki = ");
    Serial3.print(aKi);
    Serial3.print(" Kd = ");
    Serial3.println(aKd);
    Serial3.print("Setpoint: ");
    Serial3.println(aHome);
    tuneA = false;
  }

  if ((iTune != 99) && (tuneP)) {
    switch (iTune) {
      case 0:
        pKp = Serial3.parseFloat();
        iTune = 99;
        break;
      case 1:
        pKi = Serial3.parseFloat();
        iTune = 99;
        break;
      case 2:
        pKd = Serial3.parseFloat();
        iTune = 99;
        break;
      case 5:
        aHome = Serial3.parseFloat();
        iTune = 99;
        break;
    }
    delay(2);
    PIDp.SetTunings(pKp, pKi, pKd);
    Serial3.print("New PID values: Kp = ");
    Serial3.print(pKp);
    Serial3.print(" Ki = ");
    Serial3.print(pKi);
    Serial3.print(" Kd = ");
    Serial3.println(pKd);
    Serial3.print("Setpoint: ");
    Serial3.println(aHome);
    tuneP = false;
  }


  //  if (*data != 'q') {
  //    printFlag = false;
  //  }





  switch (*data) {
#ifdef print_help
    case 'h':
      Serial3.println("Displaying help");
      Serial3.println("c = stop");
      Serial3.println("x = tune posisiton loop");
      Serial3.println("m = printTimeFlag");
      Serial3.println("l = show current tuning values (all)");
      Serial3.println("q = printFlag");
      Serial3.println("p i & d, next number will tune");
      Serial3.println("g = go");
      Serial3.println("t = tune home angle");
      Serial3.println("e = printFlag7");
      Serial3.println("v = printFlag8");
      Serial3.println("[ = pOutput limits");
      break;
#endif
    case 'e':
      printFlag7 = !printFlag7;
      iTune = 99;
      break;
    case 'v':
      printFlag8 = !printFlag8;
      iTune = 99;
      break;
    case '#':
      Serial3.println("Angle loop selected..");
      tuneA = true;
      iTune = 99;
      break;
    case 'c':
      stopFlag = true;
      iTune = 99;
      break;
    case 'x':
      Serial3.println("Position loop selected..");
      tuneP = true;
      iTune = 99;
      break;
    case 'm':
      printTimeFlag = !printTimeFlag;
      iTune = 99;
      break;
    case '[':
      posLimitUpdate = true;
      iTune = 99;
      break;
    case 'l':
      Serial3.print("Current PID values (angle): Kp = ");
      Serial3.print(aKp);
      Serial3.print(" Ki = ");
      Serial3.print(aKi);
      Serial3.print(" Kd = ");
      Serial3.println(aKd);
      Serial3.print("Current PID values (pos): Kp = ");
      Serial3.print(pKp);
      Serial3.print(" Ki = ");
      Serial3.print(pKi);
      Serial3.print(" Kd = ");
      Serial3.println(pKd);
      Serial3.print("Setpoint: ");
      Serial3.println(aHome);
      Serial3.print("Angle offset allowed is now: ");
      Serial3.print(pOutputMin);
      Serial3.print(" to ");
      Serial3.println(pOutputMax);
      break;
    case 'q':
      printFlag = !printFlag;
      iTune = 99;
      break;
    case 'p':
      pFlag = true;
      iTune = 0;
      //Serial3.println (*data);
      if (tuneA) {
        Serial3.print("Current PID values (angle): Kp = ");
        Serial3.print(aKp);
        Serial3.print(" Ki = ");
        Serial3.print(aKi);
        Serial3.print(" Kd = ");
        Serial3.println(aKd);
      } else if (tuneP) {
        Serial3.print("Current PID values (pos): Kp = ");
        Serial3.print(pKp);
        Serial3.print(" Ki = ");
        Serial3.print(pKi);
        Serial3.print(" Kd = ");
        Serial3.println(pKd);
      } else {
        Serial3.println("Please select tuneA or tuneP");
      }
      Serial3.println(" Enter float:");
      while (Serial3.available () == 0) {}
      break;
    case 'i':
      iFlag = true;
      iTune = 1;

      if (tuneA) {
        Serial3.print("Current PID values (angle): Kp = ");
        Serial3.print(aKp);
        Serial3.print(" Ki = ");
        Serial3.print(aKi);
        Serial3.print(" Kd = ");
        Serial3.println(aKd);
      } else if (tuneP) {
        Serial3.print("Current PID values (pos): Kp = ");
        Serial3.print(pKp);
        Serial3.print(" Ki = ");
        Serial3.print(pKi);
        Serial3.print(" Kd = ");
        Serial3.println(pKd);
      } else {
        Serial3.println("Please select tuneA or tuneP");
      }
      Serial3.println(" Enter float:");
      break;
    case 'd':
      dFlag = true;
      iTune = 2;
      if (tuneA) {
        Serial3.print("Current PID values (angle): Kp = ");
        Serial3.print(aKp);
        Serial3.print(" Ki = ");
        Serial3.print(aKi);
        Serial3.print(" Kd = ");
        Serial3.println(aKd);
      } else if (tuneP) {
        Serial3.print("Current PID values (pos): Kp = ");
        Serial3.print(pKp);
        Serial3.print(" Ki = ");
        Serial3.print(pKi);
        Serial3.print(" Kd = ");
        Serial3.println(pKd);
      } else {
        Serial3.println("Please select tuneA or tuneP");
      }
      Serial3.println(" Enter float:");
      break;
    case 'g':
      stopFlag = false;
      iTune = 99;
      break;
    case 't':
      //stopFlag = false;
      iTune = 5;
      break;

  }


}  // end of process_data



void setup() {

  // Comms to PC
  Serial.begin(115200);
  // Comms to hoverboard
  Serial4.begin(115200);
  // Comms for bluetooth
  btSerial.begin(57600);
  // Comms for tuning
  tuningSerial.begin(1000000);

  init_mpu();

  attachInterrupt(digitalPinToInterrupt(left_encoder.hall_B_int), l_hall_a_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoder.hall_C_int), l_hall_b_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoder.hall_A_int), l_hall_c_change, CHANGE);

  attachInterrupt(digitalPinToInterrupt(right_encoder.hall_B_int), r_hall_a_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder.hall_C_int), r_hall_b_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder.hall_A_int), r_hall_c_change, CHANGE);

  pinMode(LED, OUTPUT);


  //Setpoint = 169.8;
  PIDa.SetOutputLimits(-300, 300);
  PIDa.SetMode(AUTOMATIC);

}
bool p_flag = true;
long timer = millis();
long timer_2 = millis();
long imu_startup_timer = 30000;

char global_ID;
float global_value;
void print_velocity() {
  if (((millis() - timer) > 175)) {

    Serial.print("velocity = ");
    Serial.print(left_encoder.get_velocity() * 1000);
    Serial.print(" r_velocity = ");
    Serial.print(right_encoder.get_velocity() * 1000);
    Serial.print(" , angle: ");
    Serial.print(get_imu_data(1));
    Serial.print(", ");
    Serial.print(global_ID);
    Serial.print("val: ");
    Serial.println(global_value);
    timer = millis();
  }
}
void test_motors(int upto, int loop_delay) {
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
}
bool s_flag = false;

struct btData btMessage;

void loop() {

  //print_velocity();
  get_mpu_data();
  Input = get_imu_data(2);

  char* tuningMessage = tuningSerial.returnNewMessage('\n');
  if (tuningMessage != "xx") {
    Serial3.println(tuningMessage);
    process_data(tuningMessage);
  } else {
    //Serial3.println(tuningMessage);
  }


  PIDa.Compute();

  Serial.print(Input);
  Serial.print("\t");
  Serial.println(Output);

  if (millis() < imu_startup_timer) {
    hoverboard.sendBuzzer(10, 1, 10, PROTOCOL_SOM_NOACK);
    delay((imu_startup_timer - millis()) / 20);
    hoverboard.sendBuzzer(8, 1, 40, PROTOCOL_SOM_NOACK);
    delay((imu_startup_timer - millis()) / 20);
  }

  if (millis() > imu_startup_timer) {
    if (!stopFlag) {
      hoverboard.sendPWM((-1)*Output, (-1)*Output, PROTOCOL_SOM_NOACK);
    }
  }


  if (((millis() - timer_2) > 1000) && p_flag) {
    //void HoverboardAPI::sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint16_t buzzerLen, char som)
    hoverboard.sendBuzzer(10, 1, 100, PROTOCOL_SOM_NOACK);
    Serial4.println("unlockASCII");
    Serial4.println('P');
    p_flag = false;
  }

  btMessage = btSerial.checkForNewMessage('&');

  if (btMessage.id != '\0') {
    global_ID = btMessage.id;
    global_value = btMessage.value;
  }

}
