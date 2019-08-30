
/*-----( Import needed libraries )-----*/
#include <string.h>
#include <I2Cdev.h>

//Plotter plot;


#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


#include <L298N.h>
#include <PID_v1.h>


//Bluetooth pins
#define HC_05_TXD_ARDUINO_RXD 16
#define HC_05_RXD_ARDUINO_TXD 15
#define HC_05_SETUPKEY        D12


// Calculate based on max input size expected for one command
#define INPUT_SIZE 30

#define ppr 2752 //pulses per revolution

//Left motor pins
#define lEN 5
#define lIN1 6
#define lIN2 7

//Right motor pins
#define rEN 3
#define rIN1 8
#define rIN2 10

L298N right_motor(rEN, rIN1, rIN2);
L298N left_motor(lEN, lIN1, lIN2);

//#define aHome 186
//#define aHome 195


#define aHome -45.6

#define INTERRUPT_PIN 2

#define lIntPin 3  // the pin we are interested in
#define lDirPin 11
#define rIntPin 4
#define rDirPin 9


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


void leftCB(void);
void stop_robot(void);

volatile bool lFlag = false;
volatile bool rFlag = false;
volatile signed long lCount = 0;
volatile signed long rCount = 0;




/*-----( PID variables )------*/
//Define Variables we'll be connecting to

double aInput, aOutput;
double pSetpoint, pInput, pOutput;
double aSetpoint = aHome;


//Specify the links and initial tuning parameters
//double aKp = 30, aKi = 0.1, aKd = 1;
double pKp = 0.5 , pKi = 0, pKd = 0;
double aKp = 7 , aKi = 7, aKd = 0.66;
PID PIDp(&pInput, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT);
PID PIDa(&aInput, &aOutput, &aSetpoint, aKp, aKi, aKd, DIRECT);


/*-----( Declare objects )-----*/
//SoftwareSerial BTSerial(HC_05_TXD_ARDUINO_RXD, HC_05_RXD_ARDUINO_TXD); // RX | TX
HardwareSerial Serial5(PD2, PC12);
//HardwareSerial Serial5(PC12, PD2, PC12);

/*-----( Declare Variables )-----*/
char inData[80];
char commandString[80];
//byte index = 0;



unsigned long time;
unsigned long timer_x = 0;



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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup()   /****** SETUP: RUNS ONCE ******/
{

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }


  pinMode(HC_05_SETUPKEY, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode

  digitalWrite(HC_05_SETUPKEY, HIGH);  // Set command mode when powering up


  pinMode(13, OUTPUT);



  Serial.begin(115200);   // For the Arduino IDE Serial Monitor
  Serial.println("YourDuino.com HC-05 Bluetooth Module AT Command Utility V1.02");
  Serial.println("Set Serial Monitor to 'Both NL & CR' and '9600 Baud' at bottom right");

  //Wire.begin();
  //mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);
//  BTSerial.begin(9600);  // HC-05 default speed in AT command mode
  Serial5.begin(57600);


  PIDa.SetMode(AUTOMATIC);
  PIDp.SetMode(AUTOMATIC);
  PIDp.SetOutputLimits(-255, 255);
  PIDa.SetOutputLimits(-70, 70);
  PIDa.SetSampleTime(20);

  pinMode(lIntPin, INPUT_PULLUP);
  //pinMode(11, INPUT_PULLUP);
  //pinMode(rightIntPin, INPUT_PULLUP);
  //  attachInterrupt(digitalPinToInterrupt(lIntPin), leftCB, RISING);


  timer_x = millis();

  Serial.println("Setup finished");
  //  plot.Begin();
  //  plot.AddTimeGraph("left wheel PID", 500, "Input", pInput, "Setpoint", pSetpoint, "Output", pOutput);
  //void AddTimeGraph( String title, int pointsDisplayed, String label1, Variable1Type variable1, String label2, Variable2Type variable2, ... )
  //p.AddTimeGraph( "Some title of a graph", 500, "label for x", x );



}//--(end setup )---

bool rollRX = false;
bool pitchRX = false;
bool sensRX = false;
bool x_posRX = true;
bool rollRX2 = false;
bool pitchRX2 = false;

float x_position = 0.0;
float roll = 0;
float pitch = 0;
int sensitivity = 0;
bool flag = true;
enum rx { UNDEF, s, r, p };
int timer1 = millis();
int timer2 = millis();
int duration = 200;

int serialTimer = 0;
#define SerialTime 100
void do_robot_go(void);
void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  // if programming failed, don't try to do anything
//  if (!dmpReady) return;


  // wait for MPU interrupt or extra packet(s) available
  while (1){//(!mpuInterrupt && fifoCount < packetSize) {



    if ((serialTimer + SerialTime) < millis()) {
      // READ from HC-05 and WRITE to Arduino Serial Monitor
      if (Serial5.available()) {

        // Get next command from Serial (add 1 for final 0)
        char input[INPUT_SIZE + 1];
        byte size = Serial5.readBytes(input, INPUT_SIZE);
        // Add the final 0 to end the C string
        input[size] = 0;
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
            float fPosition = atof(separator);
            Serial.println(fPosition);
            Serial.println(iPosition);
            if (iPosition != 999) {
              switch (ID) {
                case 's':
                  sensitivity = iPosition;
                  //sensRX = true;
                  // Serial.print(position); Serial.print(",");
                  break;
                case 'r':
                  roll = fPosition;
                  rollRX = true;
                  aSetpoint = roll;
                  Serial.print("roll");
                  Serial.println(fPosition);
                  //Serial.print(position); Serial.print(",");
                  break;
                case 'p':
                  pitch = fPosition;
                  pSetpoint = pitch;
                  pitchRX = true;
                  Serial.print("pitch");
                  Serial.println(fPosition);
                  //Serial.print(position); Serial.print(",");
                  break;
                case 'x':
                  Serial.print("x");
                  Serial.println(fPosition);
                  x_position = fPosition;
                  x_posRX = true;
                  break;
              }

              if (pitchRX) {
                pitchRX2 = true;
              }
              if (rollRX) {
                rollRX2 = true;
              }
              rollRX = false;
              pitchRX = false;
              //char buf[INPUT_SIZE + 1];
              //sprintf(buf, "%s = %d", &command[0], position);
              //Serial.write(buf);
              // Do something with servoId and position
            }
          }
          // Find the next command in input string
          command = strtok(0, "&");
        }

      }
      serialTimer = millis();
    }


      //pitchRX2 = true;
      //x_posRX = false;

      if ((pitchRX2 || rollRX2) && (!x_posRX)) {
//        timer_x = millis();
        //int tracker = (int)((lCount + rCount ) / 2); //???
        int tracker = (int)lCount;
        if (roll > 1.0) {
          roll = 1.0;
        }
        if (pitch > 1.0) {
          pitch = 1.0;
        }
        if (roll < -1.0) {
          roll = -1.0;
        }
        if (pitch < -1.0) {
          pitch = -1.0;
        }
        if (roll < 0.0) {
          roll = 0.0 - roll;
        }
        if (pitch < 0.0) {
          pitch = 0.0 - pitch;
        }



        int y = map(int(roll * 100), 0, 100, 0, 254);
        analogWrite(13, y);
        //Maybe put offsets here sometime?
        //    -0.2
        //    -0.65
        //
        //    +0.2
        //    +0.65/2 = 0.325
        //    +0.525
        Serial.print(roll);
        //Serial.print((roll-0.35));
        Serial.print(",");
        //Serial.println(pitch-0.525);
        Serial.println(pitch);
        pitchRX2 = false;
        rollRX2 = false;
        // delay(10);


      } else if (x_posRX) {
        timer_x = millis();

        //Below line should go: While robot isn't settled at it's goal...
        aSetpoint = aHome; //upright
        pSetpoint = x_position; //where we've told it to go
        //    do_robot_go(); //go do robot

        //If we lose contact for 750ms, give angle mode a chance to take over.
        if ((millis() - timer_x) > 700) {
          x_posRX = false;
        }


      } else if (((!x_posRX) && (!pitchRX2) && (!rollRX2)) && ((millis() - timer_x) > 800)) {

        stop_robot();
        pSetpoint = x_position; //where we've told it to go last hopefully
        aSetpoint = aHome;
      } else {
        pSetpoint = 0; //where we've told it to go last hopefully
        aSetpoint = aHome;
        do_robot_go(); //go do robot
      }


      //    if (lFlag) {
      //
      //      lFlag = false;
      //    }

      do_robot_go(); //go do robot
    }


    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //Serial.print("roll:");
      aInput = ypr[2] * 180 / M_PI;
      //Serial.println(aInput);
      //    Serial.print("lCount:");
      //    Serial.print(lCount);
      //
      //    Serial.print("\trCount:");
      //    Serial.println(rCount);
      do_robot_go(); //go do robot


    }

  }



  //--(end main loop )---

  int whereAmI(void) {
    //int tracker = (int)((lCount + rCount ) / 2);
    int tracker = (int)lCount;
    return tracker;
  }

  bool haltPos = true;
  void stop_robot(void) {
    if (haltPos) {
      x_position = whereAmI();
    }

    //don't keep taking new readings or we'll at best drift into something
    haltPos = false;


  }


  void do_robot_go(void) {

    //  Serial.print("Count:");
    //  Serial.println(lCount);


    //  int tracker = (int)((lCount + rCount ) / 2);//oh, here it is
    int tracker = (int)lCount;
    //mpu6050.update();
    //aInput = ((mpu6050.getAngleX()) + 200); //Probably wrong axis
    pInput = tracker;//write this function
    aSetpoint = aHome;
    //PIDa.Compute();
    PIDa.Compute();
    //Serial.println("");





    //pSetpoint = 0;
    //pInput = tracker;
    //PIDp.Compute();
    float angleWeight = 0.8;
    float positionWeight = 0.2;
    //float fSpeed = ((aOutput * angleWeight) + (pOutput * positionWeight));
    float fSpeed = aOutput;
    //float fSpeed = ((aOutput * angleWeight));// + (pOutput * positionWeight));
    //  float fSpeed = aOutput;
    if (fSpeed > 0.0) {
      left_motor.setSpeed(fSpeed);
      //  right_motor.forward();
      left_motor.backward();
      // right_motor.backward();
    } else if (fSpeed < 0.0) {
      fSpeed = fSpeed - ((2) * fSpeed);
      left_motor.setSpeed(fSpeed);
      //    right_motor.backward();
      left_motor.forward();
      //right_motor.forward();

    }
    //  right_motor.setSpeed(fSpeed);
    //left_motor.setSpeed(fSpeed);
    // right_motor.setSpeed(fSpeed);
    // Serial.write(27);       // ESC command
    //  Serial.print("[2J");    // clear screen command
    //  Serial.write(27);
    //  Serial.print("[H");     // cursor to home command
    //  Serial.print("Tracker:");
    //  Serial.print(lCount);
    //  Serial.print("\tInput:");
    //  Serial.print(aInput);
    //  Serial.print("\tSetpoint:");
    //  Serial.print(aSetpoint);
    //  Serial.print("\tOutput:");
    //  Serial.print(fSpeed);

    //
    //  Serial.print("lTracker:");
    //  Serial.print(lCount);
    //  Serial.print("\trTracker:");
    //  Serial.print(rCount);
    //  Serial.print("\taInput:");
    //  Serial.print(aInput);
    //  Serial.print("\tpInput:");
    //  Serial.print(pInput);
    //  Serial.print("\taSetpoint:");
    //  Serial.print(aSetpoint);
    //  Serial.print("\tpSetpoint:");
    //  Serial.print(pSetpoint);
    //  Serial.print("\tfSpeed:");
    //  Serial.println(fSpeed);

    //plot.Plot();


  }
  /*-----( Declare User-written Functions )-----*/
  //NONE

  //*********( THE END )***********



 
  //void leftCB(void) {
  //  lFlag = true;
  //  bool direc = (digitalRead(lDirPin) == HIGH) ? true : false;
  //  if (digitalRead(lIntPin) == HIGH) {
  //    if (direc) {
  //      lCount--;
  //    } else {
  //      lCount++;
  //    }
  //  }
  //  //(digitalRead(lDirPin)) ? lCount-- : lCount++;
  //}
  //void rightCB(void) {
  //  rFlag = true;
  //  rDir = digitalRead(5);
  //  (rDir) ? rCount-- : rCount++;
  //}
