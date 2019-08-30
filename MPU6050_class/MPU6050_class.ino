
#include "imu.h"


//MPU6050 mpu;
IMU emu;

void dmpDataReadyLocal() {
  emu.dmpDataReady();
}


void setup()
{
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReadyLocal, RISING);
//  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), emu.dmpDataReady, RISING);
    
}


void loop() {
  // if programming failed, don't try to do anything
  if (!emu.dmpReady) return;


}
