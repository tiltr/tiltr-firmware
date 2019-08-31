#include <PID_v1.h>
#include <L298N.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h> // NEWLINE : include WatchDogTimer arduino lib

//#define print_help
//#define properStart
//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////PIN MAPPINGS///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
////Bluetooth pins
//#define HC_05_TXD_ARDUINO_RXD 15
//#define HC_05_RXD_ARDUINO_TXD 14
//#define HC_05_SETUPKEY        17
//

#define ppr 2752

//Left motor pins
#define lEN 5
#define lIN1 6
#define lIN2 7

//Right motor pins
#define rEN 3
#define rIN1 8
#define rIN2 10

//mpu
#define INTERRUPT_PIN 2

#define lIntPin 3  // the pin we are interested in
#define lDirPin 11
#define rIntPin 4
#define rDirPin 9

//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////END OF PIN MAPPINGS////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
#define HC_05_TXD_ARDUINO_RXD 16
#define HC_05_RXD_ARDUINO_TXD 15
#define HC_05_SETUPKEY        14
/*-----( Declare objects )-----*/
SoftwareSerial BTSerial(HC_05_TXD_ARDUINO_RXD, HC_05_RXD_ARDUINO_TXD); // RX | TX
/*-----( Declare Variables )-----*/
char inData[80];
char commandString[80];
byte index = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////VARIABLES//////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//This one's good: -45.9
//#define aHome -45.9
//45.5
//46.3
//#define angleMax -45.56
//#define aHome -45.6
//float aHome = -45.6;
//float aHome = -47.47; //before LiPo fitted in the chassis
float aHome = -49.20; //after LiPo fitted in the chassis
//float aHome = -46.6;
//#define angleMin -45.63
//45.5
//46.3
volatile bool lFlag = false;
volatile bool rFlag = false;
volatile signed long lCount = 0;
volatile signed long rCount = 0;

/*-----( PID variables )------*/
//Define Variables we'll be connecting to

double aInput, aOutput;
double pSetpoint = 0.0, pInput = 0.0, pOutput = 0.0;
double aSetpoint = aHome;
float aOutputMax = 90.0;
float aOutputMin = -90.0;
float pOutputMax = 7.5;
float pOutputMin = -7.5;


//Specify the links and initial tuning parameters
//double aKp = 30, aKi = 0.1, aKd = 1;
double pKp = 7.0 , pKi = 0.0, pKd = 0.01;
//pre ollysdouble pKp = 10.0 , pKi = 0.0, pKd = 0.08;
double aKp = 7.0, aKi = 8.0, aKd = 0.66;
//double aKp = 70 , aKi = 140, aKd = 4.9;

//double aKp = 40 , aKi = 0.0, aKd = 0.0;
PID PIDp(&pInput, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT);
PID PIDa(&aInput, &aOutput, &aSetpoint, aKp, aKi, aKd, DIRECT);


L298N right_motor(rEN, rIN1, rIN2);
L298N left_motor(lEN, lIN1, lIN2);
/*-----( Declare objects )-----*/
//SoftwareSerial BTSerial(HC_05_TXD_ARDUINO_RXD, HC_05_RXD_ARDUINO_TXD); // RX | TX
/*-----( Declare Variables )-----*/
//char inData[80];
//char commandString[80];
//byte index = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////MPU6050 STUFF//////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////END OF MPU6050 STUFF///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////






// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  BTSerial.begin(57600);  // HC-05 default speed in AT command mode

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  //Serial.begin(115200); //works, crashes main loop during comms
  Serial.begin(1000000); // 1MBps //lets try this
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  //\wdt_enable(WDTO_2S); // NEWLINE : Activate the WTD with a 2S counter (before MPU init)
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
  //  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //My offsets:
  mpu.setXGyroOffset(70);
  mpu.setYGyroOffset(-26);
  mpu.setZGyroOffset(108);
  mpu.setZAccelOffset(3763);
  /*
     //New pcb orientation
     Sensor readings with offsets: -8  -7  16374 -1  0 1
    Your offsets: -729  -2947 3763  70  -26 108
    Data is printed as: acelX acelY acelZ giroX giroY giroZ


  */
  /*  // supply your own gyro offsets here, scaled for min sensitivity (came with code)
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  */
  /*
    Sensor readings with offsets: 4 -1  16389 0 0 0
    Your offsets: -2422 -1275 4542  74  -21 94

    Data is printed as: acelX acelY acelZ giroX giroY giroZ
    Check that your sensor readings are close to 0 0 16384 0 0 0
  */

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  PIDa.SetMode(AUTOMATIC);


  PIDa.SetOutputLimits(-90, 90);
  PIDa.SetSampleTime(10);

  PIDp.SetMode(AUTOMATIC);
  PIDp.SetOutputLimits(-7.5, 7.5);
  PIDp.SetSampleTime(60);

  // pin change interrupt (D12)
  PCMSK0 |= bit (PCINT4);  // want pin 12
  PCIFR  |= bit (PCIF0);    // clear any outstanding interrupts
  PCICR  |= bit (PCIE0);    // enable pin change interrupts for D8 to D12

  // pin change interrupt (D4)
  PCMSK2 |= bit (PCINT20);  // want pin 4
  PCIFR  |= bit (PCIF2);    // clear any outstanding interrupts
  PCICR  |= bit (PCIE2);    // enable pin change interrupts for D0 to D7


  Serial.println("Setup complete!");


}

double processBluetooth(void);
char incomingByte = 'x';
#define MAX_INPUT 10

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
// here to process incoming serial data after a terminator received
bool stopFlag = false, angleFlag = true, printTimeFlag = false;
bool pFlag = false, iFlag = false, dFlag = false;
int iTune = 99;
bool printFlag = false, printFlag7 = true, printFlag8 = true;
bool tuneA = false, tuneP = false;
bool posLimitUpdate = false;
void process_data (const char * data)
{
  // for now just display it
  // (but you could compare it to some value, convert to an integer, etc.)
  Serial.println (data);


  if (posLimitUpdate) {
    pOutputMax = Serial.parseFloat();
    pOutputMin = pOutputMax * (-1);
    PIDp.SetOutputLimits(pOutputMin, pOutputMax);
    PIDp.SetMode(AUTOMATIC);
    Serial.print("Angle offset allowed is now: ");
    Serial.print(pOutputMin);
    Serial.print(" to ");
    Serial.println(pOutputMax);
    posLimitUpdate = false;
  }

  //    if (angleLimitUpdate){
  //    aOutputMax = Serial.parseFloat();
  //    aOutputMin = pOutputMax * (-0.1);
  //    PIDa.SetOutputLimits(aOutputMin, aOutputMax);
  //    PIDa.SetMode(AUTOMATIC);
  //  }



  if ((iTune != 99) && (tuneA)) {
    switch (iTune) {
      case 0:
        aKp = Serial.parseFloat();
        iTune = 99;
        break;
      case 1:
        aKi = Serial.parseFloat();
        iTune = 99;
        break;
      case 2:
        aKd = Serial.parseFloat();
        iTune = 99;
        break;
      case 5:
        aHome = Serial.parseFloat();
        iTune = 99;
        break;
    }
    delay(2);
    PIDa.SetTunings(aKp, aKi, aKd);
    Serial.print("New PID values: Kp = ");
    Serial.print(aKp);
    Serial.print(" Ki = ");
    Serial.print(aKi);
    Serial.print(" Kd = ");
    Serial.println(aKd);
    Serial.print("Setpoint: ");
    Serial.println(aHome);
    tuneA = false;
  }

  if ((iTune != 99) && (tuneP)) {
    switch (iTune) {
      case 0:
        pKp = Serial.parseFloat();
        iTune = 99;
        break;
      case 1:
        pKi = Serial.parseFloat();
        iTune = 99;
        break;
      case 2:
        pKd = Serial.parseFloat();
        iTune = 99;
        break;
      case 5:
        aHome = Serial.parseFloat();
        iTune = 99;
        break;
    }
    delay(2);
    PIDp.SetTunings(pKp, pKi, pKd);
    Serial.print("New PID values: Kp = ");
    Serial.print(pKp);
    Serial.print(" Ki = ");
    Serial.print(pKi);
    Serial.print(" Kd = ");
    Serial.println(pKd);
    Serial.print("Setpoint: ");
    Serial.println(aHome);
    tuneP = false;
  }


  //  if (*data != 'q') {
  //    printFlag = false;
  //  }





  switch (*data) {
#ifdef print_help
    case 'h':
      Serial.println("Displaying help");
      Serial.println("c = stop");
      Serial.println("w = angleFlag");
      Serial.println("x = tune posisiton loop");
      Serial.println("m = printTimeFlag");
      Serial.println("l = show current tuning values (all)");
      Serial.println("q = printFlag");
      Serial.println("p i & d, next number will tune");
      Serial.println("g = go");
      Serial.println("t = tune home angle");
      Serial.println("e = printFlag7");
      Serial.println("v = printFlag8");
      Serial.println("[ = pOutput limits");
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
      Serial.println("Angle loop selected..");
      tuneA = true;
      iTune = 99;
      break;
    case 'c':
      stopFlag = true;
      iTune = 99;
      break;
    case 'w':
      angleFlag = !angleFlag;
      iTune = 99;
      break;
    case 'x':
      Serial.println("Position loop selected..");
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
      Serial.print("Current PID values (angle): Kp = ");
      Serial.print(aKp);
      Serial.print(" Ki = ");
      Serial.print(aKi);
      Serial.print(" Kd = ");
      Serial.println(aKd);
      Serial.print("Current PID values (pos): Kp = ");
      Serial.print(pKp);
      Serial.print(" Ki = ");
      Serial.print(pKi);
      Serial.print(" Kd = ");
      Serial.println(pKd);
      Serial.print("Setpoint: ");
      Serial.println(aHome);
      Serial.print("Angle offset allowed is now: ");
      Serial.print(pOutputMin);
      Serial.print(" to ");
      Serial.println(pOutputMax);
      break;
    case 'q':
      printFlag = !printFlag;
      iTune = 99;
      break;
    case 'p':
      pFlag = true;
      iTune = 0;
      //Serial.println (*data);
      if (tuneA) {
        Serial.print("Current PID values (angle): Kp = ");
        Serial.print(aKp);
        Serial.print(" Ki = ");
        Serial.print(aKi);
        Serial.print(" Kd = ");
        Serial.println(aKd);
      } else if (tuneP) {
        Serial.print("Current PID values (pos): Kp = ");
        Serial.print(pKp);
        Serial.print(" Ki = ");
        Serial.print(pKi);
        Serial.print(" Kd = ");
        Serial.println(pKd);
      } else {
        Serial.println("Please select tuneA or tuneP");
      }
      Serial.println(" Enter float:");
      while (Serial.available () == 0) {}
      break;
    case 'i':
      iFlag = true;
      iTune = 1;

      if (tuneA) {
        Serial.print("Current PID values (angle): Kp = ");
        Serial.print(aKp);
        Serial.print(" Ki = ");
        Serial.print(aKi);
        Serial.print(" Kd = ");
        Serial.println(aKd);
      } else if (tuneP) {
        Serial.print("Current PID values (pos): Kp = ");
        Serial.print(pKp);
        Serial.print(" Ki = ");
        Serial.print(pKi);
        Serial.print(" Kd = ");
        Serial.println(pKd);
      } else {
        Serial.println("Please select tuneA or tuneP");
      }
      Serial.println(" Enter float:");
      break;
    case 'd':
      dFlag = true;
      iTune = 2;
      if (tuneA) {
        Serial.print("Current PID values (angle): Kp = ");
        Serial.print(aKp);
        Serial.print(" Ki = ");
        Serial.print(aKi);
        Serial.print(" Kd = ");
        Serial.println(aKd);
      } else if (tuneP) {
        Serial.print("Current PID values (pos): Kp = ");
        Serial.print(pKp);
        Serial.print(" Ki = ");
        Serial.print(pKi);
        Serial.print(" Kd = ");
        Serial.println(pKd);
      } else {
        Serial.println("Please select tuneA or tuneP");
      }
      Serial.println(" Enter float:");
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


double pOutputOld = 0;
void processIncomingByte (const byte inByte)
{
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
  {

    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte

      // terminator reached! process input_line here ...
      process_data (input_line);

      // reset buffer for next time
      input_pos = 0;
      break;

    case '\r':   // discard carriage return
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;

  }  // end of switch

} // end of processIncomingByte
float angleOffset = 0;
long gOldTime = 0, gOldTime2 = 0;
double twist = 0.0;
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  if (printTimeFlag) {
    Serial.print("outer Loop time: ");
    Serial.println((micros() - gOldTime));
    gOldTime = micros();
  }



  if (printTimeFlag) {
    (mpuInterrupt) ? Serial.println("mpu interrupt high") : Serial.println("mpu interrupt low");
    Serial.print("fifoCount = ");
    Serial.print(fifoCount);
    Serial.print(" out of: ");
    Serial.println(packetSize);
  }

  // wait for MPU interrupt or extra packet(s) available
  //(process rest of code when there isn't data waiting (mpu int pin low and buffer doesn't have a full packet)
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .

    if (printTimeFlag) {
      Serial.print("Inner Loop time: ");
      Serial.println((micros() - gOldTime2));
      gOldTime2 = micros();
    }



    if (BTSerial.available()) {
      //int avail = printDebugTimer("bt is available");
      angleOffset = (processBluetooth());
      //zeroFlag = (angleOffset == 0.0) ? true : false;
      //int comp = printDebugTimer("bt has been processed");
      if (printFlag) {
        Serial.print(" angle offset: ");
        Serial.print(angleOffset);
        Serial.println(" New aHome: ");
      }
      //aSetpoint = aHomeConst + angleOffset;//(float)((angleOffset-5000.0)/1000.0);
      pSetpoint = (angleOffset * 3); //(float)((angleOffset-5000.0)/1000.0);
      if (printFlag) {
        //Serial.println(pSetpoint);
      }
      Serial.println("pSetpoint, angleOffset");
      Serial.println(pSetpoint);
      Serial.println(angleOffset);
      
    }


    while (Serial.available () > 0) {
      processIncomingByte (Serial.read ());
    }


    pInput = (double)getSpeedLeft();
    if (printFlag7) {
      Serial.print(pInput);
      //Serial.print(" ");
      //      Serial.print("pInput: ");
      //      Serial.print(pInput);
    }
    //    if (pInput == 0.0 && pSetpoint == 0.0) {
    //      PIDp.Compute(true);
    //    } else {
    PIDp.Compute(false);
    if ((pOutputOld != pOutput) && ((pOutput > 2.0) || (pOutput < -2.0))) {
      pOutput = pOutput * 1.15;
      pOutputOld = pOutput;
    }
    //    }
    if (printFlag8) {
      if (printFlag7) {
        //Serial.print(" pSetpoint: ");
        Serial.print(" ");
        Serial.print(pSetpoint);
        //Serial.print(" pOutput: ");
        Serial.print(" ");
        Serial.println(pOutput);
      } else {
        Serial.print(aInput);
        Serial.print(" ");
      }
    }
    //    if ((-1.1) < pOutput < 1.1){
    //      pOutput = 0.0;
    //    }
    //


    //    if (angleOffset == 0.0) {
    //      aSetpoint = aHome;
    //    } else {
    //      aSetpoint = aHome + pOutput;
    //      //aSetpoint = aHome + (5*angleOffset);
    //    }
    //
    //



    aSetpoint = aHome + pOutput;
    //aSetpoint = aHome + (angleOffset*4);

    //    if (printFlag7) {
    //      if (!angleFlag) {
    //        Serial.print(" angle: ");
    //        Serial.println(aSetpoint);
    //      } else {
    //        Serial.print(" ");
    //        //Serial.print(" desired angle: ");
    //        Serial.print(aSetpoint);
    //        Serial.print("  ");
    //        //Serial.print(" current angle: ");
    //        Serial.println(aInput);
    //      }
    //    }
    PIDa.Compute(false);

    if (printFlag8) {
      if (!printFlag7) {
        Serial.print(aSetpoint);
        Serial.print(" ");
        Serial.println(aOutput);
      }
    }

    if (stopFlag == true) {
      left_motor.stop();
      right_motor.stop();
    } else {
      //PIDa.Compute();
      float fSpeed = aOutput;

      if (fSpeed > 0.0) {
        left_motor.setSpeed(fSpeed - twist);
        right_motor.setSpeed(fSpeed + twist);
        //        left_motor.setSpeed(fSpeed);
        //        right_motor.setSpeed(fSpeed);
        left_motor.forward();
        right_motor.forward();
      } else if (fSpeed < 0.0) {
        fSpeed = fSpeed - ((2) * fSpeed);
        left_motor.setSpeed(fSpeed - twist);
        right_motor.setSpeed(fSpeed + twist);
        //        left_motor.setSpeed(fSpeed);
        //        right_motor.setSpeed(fSpeed);
        left_motor.backward();
        right_motor.backward();
      }

    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  //Calling getIntStatus() should clear fifo overflow flag inside mpu6050 - check this
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if (printTimeFlag) {
    Serial.print("FIFOCount = ");
    Serial.println(fifoCount);
  }
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    //if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    if (printTimeFlag) {
      Serial.println("f1");
    }
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    if (printTimeFlag) {
      Serial.println("f2");
    }


    //Hack
    while (fifoCount >= packetSize) {
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }


    //#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    aInput = ypr[1] * 180 / M_PI;
    //if (45.3 < aInput < 46.3){
    //    if (angleMin < aInput < angleMax){
    //      aInput = aHome;
    //    }

    if (printFlag) {
      //Serial.println(aInput);
    }
    //#ifdef PRINT_ANGLES
    //        Serial.print("ypr\t");
    //        Serial.print(ypr[0] * 180 / M_PI);
    //Serial.print("\t");
    //Serial.println(ypr[1] * 180 / M_PI);
    //Serial.print("\t");
    //Serial.println(ypr[2] * 180 / M_PI);
    //#endif
    //#endif
    //  Serial.println(ypr[2] * 180 / M_PI);

    // blink LED to indicate activity
    //    blinkState = !blinkState;
    //    digitalWrite(LED_PIN, blinkState);
  }
}


#define INPUT_SIZE 10
double processBluetooth(void) {
  double roll = 0.0;
  // Get next command from Serial (add 1 for final 0)
  char input[INPUT_SIZE + 1];
  byte size = BTSerial.readBytesUntil('e', input, INPUT_SIZE);
  // Add the final 0 to end the C string
  input[size] = 0;
  if ((size == 0) || (size == 1)) {
    roll = 0.0;
  } else {
    Serial.println(input);

    // Read each command pair
    char* command = strtok(input, "&");
    while (command != 0)
    {
      // Split the command in two values
      char* separator = strchr(command, '=');
      if (separator != 0)
      {
        // Actually split the string in 2: replace ':' with 0
        *separator = 0;
        char ID = command[0];
        ++separator;
        int iPosition = atoi(separator);
        iPosition -= 500;
        float fPosition = (float)iPosition / 5000.0;
        if (iPosition != 999) {
          switch (ID) {
            case 's':
              //sensitivity = iPosition;
              //sensRX = true;
              // Serial.print(position); Serial.print(",");
              break;
            case 'r':
              Serial.println(fPosition);
              roll = (fPosition * 3);
              break;
            case 'i':
              Serial.println(fPosition);
              roll = (fPosition * 3);
              //Serial.println(roll);
              break;
            case '~':
              roll = 0;
              twist = (fPosition * 300);
              Serial.print("Twist: ");
              Serial.println(twist);
              break;
          }
          Serial.print("ID: ");
          Serial.println(ID);
          if ((ID != 'r')&&(ID != 'i')) {
            roll = 0;
          }
          if (ID != '~') {
            twist = 0;
          }
        }
      }
      // Find the next command in input string
      //command = strtok(0, "&");
      command = 0;
    }
  }

  return roll;
}



float countToDistanceM = (PI * 0.1) / ppr;

long old_lCount = 0, old_lTime = 0;
long old_rCount = 0, old_rTime = 0;

float getSpeedLeft(void) {
  //Serial.print("Enter GetSpeedLeft. dt: ");
  long dt = micros() - old_lTime;
  //  Serial.print(dt);
  //  Serial.print(" dt_sec: ");
  double dt_sec = ((double)dt / 1000000.0);
  //  Serial.print( dt_sec);
  //PI*0.1 (dia in meters)
  //(ppr*PI*0.1) / lCount = distance traveled in meters

  float vL;
  //float newPos = (countToDistanceM/(float)lCount);
  if (lCount != old_lCount) {
    float delta_pos = (countToDistanceM * (lCount - old_lCount));
    //    Serial.print(" delta_pos: ");
    //    Serial.print(delta_pos);
    vL = delta_pos / dt_sec;
  } else {
    vL = 0.0;
  }
  old_lCount = lCount;
  //  Serial.print(" old_lCount: ");
  //  Serial.print(old_lCount);
  old_lTime = micros();
  //  Serial.print(" old_lTime: ");
  //  Serial.println(old_lTime);
  //  Serial.print(" VL: ");
  //  Serial.print(vL);
  return (vL);
}
float getSpeedRight(void) {
  long dt = micros() - old_rTime;
  double dt_sec = ((double)dt / 1000000.0);
  float vR;
  if (rCount != old_rCount) {
    float delta_pos = (countToDistanceM * (rCount - old_rCount));
    vR = delta_pos / dt_sec;
  } else {
    vR = 0.0;
  }
  old_rCount = rCount;
  old_rTime = micros();

  return (vR);
}

// Left encoder
ISR (PCINT0_vect)
{
  lFlag = true;
  if (PINB & bit (4)) { // if pin D12 was high
    if (digitalRead(lDirPin) == HIGH) {
      lCount++;
    } else {
      lCount--;
    }  // end of PCINT2_vect
  }
}

//Right encoder
ISR (PCINT2_vect)
{
  rFlag = true;
  if (PIND & bit (4)) { // if pin D4 was high
    if (digitalRead(rDirPin) == HIGH) {
      rCount++;
    } else {
      rCount--;
    }  // end of PCINT2_vect
  }
}




//Adapdable PID tunings
//
//
//int aggressivePIDTimeout = 75;
//int aggressivePIDAngleThreshold = 4;
//
////get angle
//
//angleError = (aHome - currentAngle);
//if (angleError < 0) {
//  angleError = angleError * (-1);
//}
//
//
//
////flag logic may be wrong
//if (((aggressivePIDtimer + aggressivePIDTimeout) < millis()) && (angleError > aggressivePIDAngleThreshold) && (!aggressiveFlag)) {
//  PIDp.SetOutputLimits(pKp2, pKi2, pKd2);
//  aggressivePIDtimer = millis();
//  //set aggressive flag
//  aggressiveFlag = true;
//  chillFlag = false;
//}
//
//if (((chillPIDtimer + chillPIDTimeout) < millis()) && (angleError > chillPIDAngleThreshold) && (!chillFlag)) {
//  PIDp.SetOutputLimits(pKp, pKi, pKd);
//  chillPIDtimer = millis();
//  //set aggressive flag
//  chillFlag = true;
//  aggressiveFlag = false;
//}

