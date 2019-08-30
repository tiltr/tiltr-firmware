
#include <PID_v1.h>
#include <hall_encoders.h>
#include <HoverboardAPI.h>
#include <Filters.h>

#define LED PA5

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

void ISR_IMU_timer(HardwareTimer*)
{
  float imu_data = get_imu_data(0);
  //Serial.println(imu_data);
}


void setup() {

  // Comms to PC
  Serial.begin(115200);
  // Comms to hoverboard
  Serial4.begin(115200);

  init_mpu();

#if defined(TIM1)
  TIM_TypeDef *Instance = TIM1;
#else
  TIM_TypeDef *Instance = TIM2;
#endif

  // Instantiate HardwareTimer object. Thanks to 'new' instanciation, HardwareTimer is not destructed when setup() function is finished.
  HardwareTimer *MyTim = new HardwareTimer(Instance);


  MyTim->setMode(2, TIMER_OUTPUT_COMPARE);  // In our case, channekFalling is configured but not really used. Nevertheless it would be possible to attach a callback to channel compare match.
  MyTim->setOverflow(400, HERTZ_FORMAT); // 10 Hz
  MyTim->attachInterrupt(ISR_IMU_timer);
  MyTim->resume();






  // configure LED for output


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

void print_velocity() {
  if (((millis() - timer) > 175)) {

    Serial.print("velocity = ");
    Serial.print(left_encoder.get_velocity() * 1000);
    Serial.print(" r_velocity = ");
    Serial.print(right_encoder.get_velocity() * 1000);
    Serial.println(" , angle: ");
    //Serial.println(get_imu_data(1));
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
void loop() {

  print_velocity();

//  if (((millis() - timer_2) > 1000) && p_flag) {
//    //void HoverboardAPI::sendBuzzer(uint8_t buzzerFreq, uint8_t buzzerPattern, uint16_t buzzerLen, char som)
//    hoverboard.sendBuzzer(10, 1, 100, PROTOCOL_SOM_NOACK);
//    Serial4.println("unlockASCII");
//    Serial4.println('P');
//    p_flag = false;
//  }
  //
  //  for (int i = 0; i < 3; i++) {
  //    Serial.print(get_imu_data(i));
  //  }
  //  Serial.println("");


}
