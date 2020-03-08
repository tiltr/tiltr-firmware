#include "PID_v1.h"
#include <HoverboardAPI.h>

#define INITIAL_HOME_ANGLE 3.67

// Hoverboard comms
HardwareSerial Serial4(PA1, PA0);
int serialWrapper(unsigned char *data, int len) {
  return (int) Serial4.write(data, len);
}
HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);


/*-----( PID variables )------*/
double aInput, aOutput;
double aSetpoint = INITIAL_HOME_ANGLE;
float aOutputMax = 200.0;
float aOutputMin = -200.0;
double aKp_1 = 140.0, aKi_1 = 0.0, aKd_1 = 0.0;
double aKp_2 = 90.0, aKi_2 = 0.0, aKd_2 = 0.0;
PID PIDa(&aInput, &aOutput, &aSetpoint, aKp_1, aKi_1, aKd_1, DIRECT);


/*-----( Timers )------*/
bool unlock_ascii_on_startup = true;
long imu_startup_timer = 30000; // ms
long print_mpu_timer = millis();
int print_mpu_period = 1000;

void setup() {

  // PC comms
  Serial.begin(115200);

  // Comms to hoverboard
  Serial4.begin(115200);

  // Start MPU6050
  init_mpu();

  PIDa.SetSampleTime(20);
  PIDa.SetOutputLimits(aOutputMin, aOutputMax);
  PIDa.SetMode(AUTOMATIC);
}

long duration;
unsigned long ElapsedTime;

void loop() {

  if (millis() > (print_mpu_timer + print_mpu_period)) {
    // Serial.println(aInput);
    print_mpu_timer = millis();
  }

  aInput = get_imu_data(2);

  if (millis() > imu_startup_timer) {
    // PWM, steer, speed_max_power, speed_min_power, som (acknowlegdement?)
    double angle_error = abs(INITIAL_HOME_ANGLE - aInput); //distance away from setpoint
    if (angle_error < 0.7) {
      PIDa.SetTunings(aKp_1, aKi_1, aKd_1);
    } else {
      PIDa.SetTunings(aKp_2, aKi_2, aKd_2);
    }
    if (PIDa.Compute()) {
      hoverboard.sendPWMData((aOutput * (-1)), 0, 100, -100, 0, PROTOCOL_SOM_NOACK);
    }
    //hoverboard.sendSpeedData(i, i, 120, 0, PROTOCOL_SOM_NOACK);
  }


  // Make scary sounds during startup to warn that the motors are about to turn on
  if (millis() < imu_startup_timer) {
    // scary_startup_sounds(imu_startup_timer);
    // print_startup_countdown(1000);
    graph_startup_countdown();
  } else {
    Serial.print(aInput);
    Serial.print(" ");
    Serial.println(aOutput);
  }

  if ((millis() > 2000) && unlock_ascii_on_startup) {
    unlock_ascii();
    unlock_ascii_on_startup = false;
  }

  delay(5);

}


//int square_count_1 = 0;
//void square_out() {
//  if (square_count_1 < 100) {
//    Serial.print(50);
//    Serial.print(" ");
//    Serial.println(0);
//  } else {
//    Serial.print(0);
//    Serial.print(" ");
//    Serial.println(50);
//  }
//  if (square_count_1 > 200) {
//    square_count_1 = 0;
//  }
//  square_count_1++;
//}

void graph_startup_countdown() {
  if (millis() < imu_startup_timer) {
    int remaining = imu_startup_timer - millis();
    Serial.print(remaining);
    Serial.print(" ");
    Serial.println(-remaining);
  }
}

long last = millis();
void print_startup_countdown(int period_ms) {
  if (millis() > (last + period_ms)) {
    Serial.print("T - ");
    Serial.print((imu_startup_timer - millis()) / 1000.0);
    Serial.println(" seconds");
    last = millis();
  }
}

bool toggle_pitch;
unsigned long pitch_timer = millis();
void scary_startup_sounds(long total_startup_duration) {
  if (millis() > pitch_timer) {
    int pitch = toggle_pitch ? 10 : 8;
    hoverboard.sendBuzzer(pitch, 1, 40, PROTOCOL_SOM_NOACK);
    pitch_timer = millis() + (total_startup_duration - millis()) / 20;
    toggle_pitch = !toggle_pitch;
  }
}

void unlock_ascii() {
  hoverboard.sendBuzzer(10, 1, 100, PROTOCOL_SOM_NOACK);
  Serial4.println("unlockASCII"); // Unlock ascii protocol
  Serial4.println('P'); // Disable poweroff & constant beeping
}
