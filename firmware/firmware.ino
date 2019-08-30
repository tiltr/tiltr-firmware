#include "bluetooth.h"
#include <PID_v1.h>
#include <hall_encoders.h>
#include <HoverboardAPI.h>
#include <Filters.h>

#define LED PA5

HardwareSerial Serial5(PD2, PC12);
SerialMessenger btSerial(Serial5);

wheel_encoder left_encoder('L');
wheel_encoder right_encoder('R');

HardwareSerial Serial4(PA1, PA0);

int serialWrapper(unsigned char *data, int len) {
  return (int) Serial4.write(data, len);
}
HoverboardAPI hoverboard = HoverboardAPI(serialWrapper);

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


void setup() {

  // Comms to PC
  Serial.begin(115200);
  // Comms to hoverboard
  Serial4.begin(115200);
  // Comms for bluetooth
  btSerial.begin(57600);

  init_mpu();

  attachInterrupt(digitalPinToInterrupt(left_encoder.hall_B_int), l_hall_a_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoder.hall_C_int), l_hall_b_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoder.hall_A_int), l_hall_c_change, CHANGE);

  attachInterrupt(digitalPinToInterrupt(right_encoder.hall_B_int), r_hall_a_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder.hall_C_int), r_hall_b_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoder.hall_A_int), r_hall_c_change, CHANGE);

  pinMode(LED, OUTPUT);

}
bool p_flag = true;
long timer = millis();
long timer_2 = millis();

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

  print_velocity();
  get_mpu_data();

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
