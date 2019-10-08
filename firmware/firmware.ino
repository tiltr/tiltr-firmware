#include "bluetooth.h"
#include "parse_serial_tuning.h"
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

int hoverboardDeadzone = 0;//40;
int aOutputOffset = hoverboardDeadzone;




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


serialTuningParser serialTuner;

/*-----( PID variables )------*/
//Define Variables we'll be connecting to
//float aHome = 169.8; //robot balancing
float aHome = serialTuner.parameters.aHome;//165.88; //robot propped up
double pKp = 7.0 , pKi = 0.0, pKd = 0.01;
//pre ollysdouble pKp = 10.0 , pKi = 0.0, pKd = 0.08;
//double aKp = 45.0, aKi = 5.0, aKd = 0.9;
//double aKp = 45.0, aKi = 0.0, aKd = 0.0;

double aInput, aOutput;
double pSetpoint = 0.0, pInput = 0.0, pOutput = 0.0;
double aSetpoint = aHome;
float aOutputMax = 90.0;
float aOutputMin = -90.0;
float pOutputMax = 7.5;
float pOutputMin = -7.5;




PID PIDp(&pInput, &pOutput, &pSetpoint, serialTuner.parameters.pKp, serialTuner.parameters.pKi, serialTuner.parameters.pKd, DIRECT);
PID PIDa(&aInput, &aOutput, &aSetpoint, serialTuner.parameters.aKp, serialTuner.parameters.aKi, serialTuner.parameters.aKd, DIRECT);




void setup() {

  // Comms to PC
  Serial.begin(115200);
  Serial.print("OK");
  // Comms to hoverboard
  Serial4.begin(115200);
  // Comms for bluetooth
  btSerial.begin(57600);
  Serial2.begin(57600);
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
  PIDa.SetOutputLimits(-100, 100);
  PIDa.SetMode(AUTOMATIC);

  PIDp.SetOutputLimits(serialTuner.parameters.pOutputMin, serialTuner.parameters.pOutputMax);
  PIDp.SetMode(AUTOMATIC);

}
bool p_flag = true;
long timer = millis();
long timer_2 = millis();
//long imu_startup_timer = 30000;
long imu_startup_timer = 30000;
//long imu_startup_timer = 30;

char global_ID;
float global_value;
void print_velocity() {
  if (((millis() - timer) > 50)) {

    Serial5.print("l_velocity = ");
    Serial5.print(left_encoder.get_velocity());
    Serial5.print(" r_velocity = ");
    Serial5.println(right_encoder.get_velocity());
    //    Serial.print(" , angle: ");
    //    Serial.print(get_imu_data(1));
    //    Serial.print(", ");
    //    Serial.print(global_ID);
    //    Serial.print("val: ");
    //    Serial.println(global_value);
    timer = millis();
  }
}
void test_motors(int from, int upto, int loop_delay) {
  for (int i = from; i < upto; i++) {
    hoverboard.sendPWM(i, (-1)*i, PROTOCOL_SOM_NOACK);
    delay(loop_delay);
    print_velocity();
  }
  for (int i = upto; i > from; i--) {
    hoverboard.sendPWM(i, (-1)*i, PROTOCOL_SOM_NOACK);
    delay(loop_delay);
    print_velocity();
  }
  for (int i = from; i < upto; i++) {
    hoverboard.sendPWM((-1)*i, i, PROTOCOL_SOM_NOACK);
    delay(loop_delay);
    print_velocity();
  }
  for (int i = upto; i > from; i--) {
    hoverboard.sendPWM((-1)*i, i, PROTOCOL_SOM_NOACK);
    delay(loop_delay);
    print_velocity();
  }
}
bool s_flag = false;

void apply_tunings() {
  if (serialTuner.parameters.updateAnglePID) {
    PIDa.SetTunings(serialTuner.parameters.aKp, serialTuner.parameters.aKi, serialTuner.parameters.aKd);
    serialTuner.parameters.updateAnglePID = false;
    Serial5.println("Angle PID gains updated");
  } else if (serialTuner.parameters.updatePositionPID) {
    PIDp.SetTunings(serialTuner.parameters.pKp, serialTuner.parameters.pKi, serialTuner.parameters.pKd);
    serialTuner.parameters.updatePositionPID = false;
    Serial5.println("Position PID gains updated");
  }
  if (aSetpoint != serialTuner.parameters.aSetpoint) {
    aSetpoint = serialTuner.parameters.aSetpoint;
  }
  if (serialTuner.parameters.pOutputLimitChanged) {
    PIDp.SetOutputLimits(serialTuner.parameters.pOutputMin, serialTuner.parameters.pOutputMax);
    serialTuner.parameters.pOutputLimitChanged = false;
  }


  if (serialTuner.parameters.printFlag) {
    Serial5.print("current angle: ");
    Serial5.println(aInput);
  }

  serialTuner.parameters.printFlag = false;

}

struct btData btMessage;
bool imu_ready = false;

int encoderTimer = 200;
long last_encoder_time = millis();

//float ticks_per_second;

void loop() {
  //
//  while (1) {
//    // test_motors(30, 200, 50);
//    char* tuningMessage = btSerial.returnNewMessage('\n');
//    if (tuningMessage != "xx") {
//      Serial5.println(tuningMessage);
//      serialTuner.parse_message(tuningMessage);
//      
//    }
//    //hoverboard.sendPWM(serialTuner.parameters.directMotorSpeed, 0, PROTOCOL_SOM_NOACK);
//    hoverboard.sendPWMData(serialTuner.parameters.directMotorSpeed, 0, 300, -300, 1, PROTOCOL_SOM_NOACK);
//
//  }



  //print_velocity();
  get_mpu_data();
  aInput = get_imu_data(2);
  Serial.println(aInput);

  char* tuningMessage = btSerial.returnNewMessage('\n');
  if (tuningMessage != "xx") {
    Serial5.println(tuningMessage);
    serialTuner.parse_message(tuningMessage);
    apply_tunings();



    //Drive mode
    if (serialTuner.parameters.drive_mode_active) {
      if (serialTuner.parameters.forward) {
        aSetpoint = (serialTuner.parameters.aSetpoint + serialTuner.parameters.steering_gain);
      } else if (serialTuner.parameters.backward) {
        aSetpoint = (serialTuner.parameters.aSetpoint - serialTuner.parameters.steering_gain);
      }
    } else {
      aSetpoint = serialTuner.parameters.aSetpoint;
    }


    //    process_data(tuningMessage);
  } else {
    //Serial3.println(tuningMessage);
  }

  encoderTimer = (abs(aOutput) < 40.0) ? 200 : (int)(2000 * (1 / ((aOutput * aOutput) / 180)));
  ///encoderTimer = (int)(1000*(1/(aOutput * aOutput) / 180));
  //encoderTimer = 1/(1000*(((aOutput * aOutput) / 180)));

  if (millis() > (encoderTimer + last_encoder_time)) {
    pInput = right_encoder.get_velocity();
    float invalid = left_encoder.get_velocity();
    get_mpu_data();
    PIDp.Compute();
    if (serialTuner.parameters.printIMU) {
      Serial5.print(aInput);
      Serial5.print("    ");
      //      //    Serial5.print(aOutput);
      //      Serial5.print(" P:error,setpoint, input, output  ");
      //      Serial5.print(pInput - pSetpoint);
      //      //Serial5.print("    ");
      //      Serial5.print("    ");
      //      Serial5.print(pSetpoint);
      //      Serial5.print("    ");
      //      Serial5.print(pInput);
      //      Serial5.print("    po ");
      //      Serial5.print(pOutput);
      //      get_mpu_data();
      //      Serial5.print("  tps: ");
      //      Serial5.print(right_encoder.get_ticks_per_second());
      //      Serial5.print("  cnt: ");
      //      Serial5.print(right_encoder.counter);
      Serial5.print("    aout");
      Serial5.println(aOutput);
      //      Serial5.print("    encTim ");
      //      Serial5.print(encoderTimer);
      //      Serial5.print("    encTimMth ");
      //      Serial5.print((int)(2000 * (1 / ((aOutput * aOutput) / 180))));
      //
      //
      //      Serial5.println(" ");
    }
    last_encoder_time = millis();
  }
  if (serialTuner.parameters.positionModeEnable) {
    aSetpoint = serialTuner.parameters.aSetpoint - pOutput;
  }

  //ticks_per_second = right_encoder.get_ticks_per_second();
  PIDa.Compute();

  if (serialTuner.parameters.printIMU) {
    //Serial5.print(left_encoder.get_velocity());
    //Serial5.print("    ");
    //    Serial5.print("A:error,setpoint, input, output  ");
    //    Serial5.print(aInput - aSetpoint);
    //    //Serial5.print("    ");
    //    Serial5.print("    ");
    //    Serial5.print(aSetpoint);
    //    Serial5.print("    ");
    //    Serial5.print(aInput);
    //    Serial5.print("    ");
    ////    Serial5.print(aOutput);
    //    Serial5.print(" P:error,setpoint, input, output  ");
    //    Serial5.print(pInput - pSetpoint);
    //    //Serial5.print("    ");
    //    Serial5.print("    ");
    //    Serial5.print(pSetpoint);
    //    Serial5.print("    ");
    //    Serial5.print(pInput);
    //    Serial5.print("    ");
    //    Serial5.print(pOutput);
    //print_velocity();
    //    Serial5.println(" ");
  }

  if (millis() < imu_startup_timer) {

    hoverboard.sendBuzzer(10, 1, 10, PROTOCOL_SOM_NOACK);
    delay((imu_startup_timer - millis()) / 20);
    hoverboard.sendBuzzer(8, 1, 40, PROTOCOL_SOM_NOACK);
    delay((imu_startup_timer - millis()) / 20);

    if (serialTuner.parameters.skipStartupTimer) {
      imu_startup_timer = 0;

    }
  }
  if (millis() > imu_startup_timer) {
    if (imu_ready == false) {
      Serial5.println("########## READY #########");
      imu_ready = true;
    }
    if (!serialTuner.parameters.enable_motors) {
      aOutputOffset = hoverboardDeadzone + serialTuner.parameters.aDeadzone;
      if (aOutput > 0) {
        //hoverboard.sendPWM((-1) * (aOutput + aOutputOffset), (-1) * (aOutput + aOutputOffset), PROTOCOL_SOM_NOACK);
        //hoverboard.sendPWM((-1) * (aOutput + aOutputOffset), 0, PROTOCOL_SOM_NOACK);
        hoverboard.sendPWMData((-1) * (aOutput + aOutputOffset), 0, 300, -300, 0, PROTOCOL_SOM_NOACK);
        //     void sendPWMData(int16_t pwm, int16_t steer = 0, int speed_max_power = 600, int speed_min_power = -600, int speed_minimum_pwm = 10, char som = PROTOCOL_SOM_ACK);

      } else {
        // hoverboard.sendPWM((-1)*aOutput, (-1)*aOutput, PROTOCOL_SOM_NOACK);
        //hoverboard.sendPWM((-1) * (aOutput - aOutputOffset), (-1) * (aOutput - aOutputOffset), PROTOCOL_SOM_NOACK);
        //hoverboard.sendPWM((-1) * (aOutput - aOutputOffset), 0, PROTOCOL_SOM_NOACK);
        hoverboard.sendPWMData((-1) * (aOutput - aOutputOffset), 0, 300, -300, 0, PROTOCOL_SOM_NOACK);
      }
    }
  }


  if (((millis() - timer_2) > 1000) && p_flag) {
    //void HoverboardAPI::sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint16_t buzzerLen, char som)
    hoverboard.sendBuzzer(10, 1, 100, PROTOCOL_SOM_NOACK);
    Serial4.println("unlockASCII");
    Serial4.println('P');
    p_flag = false;
  }
  //
  //  btMessage = btSerial.checkForNewMessage('&');
  //
  //  if (btMessage.id != '\0') {
  //    global_ID = btMessage.id;
  //    global_value = btMessage.value;
  //  }

}
