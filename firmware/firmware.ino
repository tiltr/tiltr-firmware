/*
    Author: Tom Queen
    Serial ports:
    PA1, PA0   - Hoverboard comms
    PD2, PC12  - General serial (bluetooth or hard line)
    PC11, PC10 - General serial (bluetooth or hard line)
    See https://github.com/tiltr/tiltr-PCB/blob/master/mainboard/mainboard/Schematic.pdf for more info
*/
#include "bluetooth.h"
#include "parse_serial_tuning.h"
#include "PID_v1.h"
#include <hall_encoders.h>
#include <HoverboardAPI.h>
#include <Filters.h>
// #include <ros.h>

#define LED PA5
#define print_help

// Port for control/tuning
HardwareSerial Serial3(PC11, PC10);
SerialMessenger tuningSerial(Serial3); // Serial hardware handler
serialTuningParser serialTuner; // Serial message parser

// Hoverboard comms
HardwareSerial Serial4(PA1, PA0);
int serialWrapper(unsigned char *data, int len) {
  return (int) Serial4.write(data, len);
}
HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);

//struct btData btMessage; // I think this is something to do with android bluetooth control

wheel_encoder left_encoder('L');
wheel_encoder right_encoder('R');

/*-----( PID variables )------*/
double aInput, aOutput;
double aSetpoint = serialTuner.parameters.aHome;
float aOutputMax = 100.0;
float aOutputMin = -100.0;
double pSetpoint = 0.0, pInput = 0.0, pOutput = 0.0;
PID PIDp(&pInput, &pOutput, &pSetpoint, serialTuner.parameters.pKp, serialTuner.parameters.pKi, serialTuner.parameters.pKd, DIRECT);
PID PIDa(&aInput, &aOutput, &aSetpoint, serialTuner.parameters.aKp, serialTuner.parameters.aKi, serialTuner.parameters.aKd, DIRECT);

/*-----( Timers )------*/
bool unlock_ascii_on_startup = true;
int print_velocity_period = 50; // ms
int print_period = 50; // ms
long imu_startup_timer = 3000; // ms
int encoderTimer = 200; // Default frequency for calculating wheel velocities (ms), increases when robot is slow
long print_velocity_timer = millis();
long last_encoder_time = millis();
long print_timer = millis();
bool imu_ready = false;

void setup() {

  // Comms to PC
  Serial.begin(115200);
  Serial.print("OK");

  // Comms to hoverboard
  Serial4.begin(115200);

  // Comms for control/tuning
  tuningSerial.begin(57600);

  // Start MPU6050
  init_mpu();

  attachInterrupt(digitalPinToInterrupt(left_encoder.hall_B_int), l_hall_a_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoder.hall_C_int), l_hall_b_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoder.hall_A_int), l_hall_c_change, CHANGE);

  attachInterrupt(digitalPinToInterrupt(right_encoder.hall_B_int), r_hall_a_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder.hall_C_int), r_hall_b_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder.hall_A_int), r_hall_c_change, CHANGE);

  pinMode(LED, OUTPUT);

  PIDa.SetOutputLimits(aOutputMin, aOutputMax);
  PIDa.SetMode(AUTOMATIC);

  PIDp.SetOutputLimits(serialTuner.parameters.pOutputMin, serialTuner.parameters.pOutputMax);
  PIDp.SetMode(AUTOMATIC);

}

// Call frequently to stop imu buffer overflowing
void get_mpu_data() {
  float imu_data = get_imu_data(0);
}

void print_velocity() {
  if (((millis() - print_velocity_timer) > print_velocity_period)) {
    Serial3.print("l_velocity = ");
    Serial3.print(left_encoder.get_velocity());
    Serial3.print(" r_velocity = ");
    Serial3.println(right_encoder.get_velocity());
    print_velocity_timer = millis();
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

void apply_tunings() {
  if (serialTuner.parameters.updateAnglePID) {
    PIDa.SetTunings(serialTuner.parameters.aKp, serialTuner.parameters.aKi, serialTuner.parameters.aKd);
    serialTuner.parameters.updateAnglePID = false;
    Serial3.println("Angle PID gains updated");
  } else if (serialTuner.parameters.updatePositionPID) {
    PIDp.SetTunings(serialTuner.parameters.pKp, serialTuner.parameters.pKi, serialTuner.parameters.pKd);
    serialTuner.parameters.updatePositionPID = false;
    Serial3.println("Position PID gains updated");
  }
  if (aSetpoint != serialTuner.parameters.aSetpoint) {
    aSetpoint = serialTuner.parameters.aSetpoint;
  }
  if (serialTuner.parameters.pOutputLimitChanged) {
    PIDp.SetOutputLimits(serialTuner.parameters.pOutputMin, serialTuner.parameters.pOutputMax);
    serialTuner.parameters.pOutputLimitChanged = false;
  }
  if (serialTuner.parameters.printFlag) {
    Serial3.print("current angle: ");
    Serial3.println(aInput);
  }
  serialTuner.parameters.printFlag = false;
}

void scary_startup_sounds(long current_timer) {
  hoverboard.sendBuzzer(10, 1, 10, PROTOCOL_SOM_NOACK);
  delay((current_timer - millis()) / 20);
  hoverboard.sendBuzzer(8, 1, 40, PROTOCOL_SOM_NOACK);
  delay((current_timer - millis()) / 20);
}

void unlock_ascii() {
  hoverboard.sendBuzzer(10, 1, 100, PROTOCOL_SOM_NOACK);
  Serial4.println("unlockASCII"); // Unlock ascii protocol
  Serial4.println('P'); // Disable poweroff & constant beeping
}


void loop() {
  // Call frequently to stop buffer overflowing,
  // should be called via timer-interrupt
  get_mpu_data();

  while (1) {
    test_motors(0, 200, 1);
  }

  char* tuningMessage = tuningSerial.returnNewMessage('\n');
  if (tuningMessage != "xx") {
    Serial3.println(tuningMessage);
    serialTuner.parse_message(tuningMessage);
    apply_tunings();

    if (serialTuner.parameters.drive_mode_active) {
      //Drive mode
      if (serialTuner.parameters.forward) {
        aSetpoint = (serialTuner.parameters.aSetpoint + serialTuner.parameters.steering_gain);
      } else if (serialTuner.parameters.backward) {
        aSetpoint = (serialTuner.parameters.aSetpoint - serialTuner.parameters.steering_gain);
      }
    } else {
      // Not drive mode
      aSetpoint = serialTuner.parameters.aSetpoint;
    }
  }

  // Calculate wheel speed more often when we're telling the robot to move quickly
  encoderTimer = (abs(aOutput) < 40.0) ? 200 : (int)(2000 * (1 / ((aOutput * aOutput) / 180)));

  if (millis() > (encoderTimer + last_encoder_time)) {
    pInput = 0.0; //right_encoder.get_velocity();
    // Clear imu buffer (just in case calcs take too long)
    get_mpu_data();
    PIDp.Compute();
    last_encoder_time = millis();
  }

  aInput = get_imu_data(2);

  if ((millis() > (print_timer + print_period)) && serialTuner.parameters.printIMU) {
    Serial3.print("aInput: ");
    Serial3.print(aInput);
    Serial3.print("  , aout: ");
    Serial3.println(aOutput);
    print_timer = millis();
  }

  // Adjust goal angle based off velocity to hold a position
  if (serialTuner.parameters.positionModeEnable) {
    // aSetpoint = serialTuner.parameters.aSetpoint - pOutput;
  }

  PIDa.Compute();

  if (millis() > imu_startup_timer) {
    if (imu_ready == false) {
      Serial3.println("########## READY #########");
      imu_ready = true;
    }
    if (!serialTuner.parameters.enable_motors) {
      if (aOutput > 0) {
        // PWM, steer, speed_max_power, speed_min_power, som (acknowlegdement?)
        hoverboard.sendPWMData((-1) * (aOutput + serialTuner.parameters.aDeadzone), 0, 300, -300, 0, PROTOCOL_SOM_NOACK);
      } else {
        hoverboard.sendPWMData((-1) * (aOutput - serialTuner.parameters.aDeadzone), 0, 300, -300, 0, PROTOCOL_SOM_NOACK);
      }
    }
  }

  // Make scary sounds during startup to warn that the motors are about to turn on
  if (millis() < imu_startup_timer) {
    scary_startup_sounds(imu_startup_timer);
    if (serialTuner.parameters.skipStartupTimer) {
      imu_startup_timer = 0;
    }
  }

  if ((millis() > 2000) && unlock_ascii_on_startup) {
    unlock_ascii();
    unlock_ascii_on_startup = false;
  }

}

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
