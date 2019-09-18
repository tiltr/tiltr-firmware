#ifndef PARSE_SERIAL_TUNING
#define PARSE_SERIAL_TUNING

#include "Arduino.h"

//#define INITIAL_HOME_ANGLE 165.32 // degrees
//#define INITIAL_HOME_ANGLE 167.5 // degrees
#define INITIAL_HOME_ANGLE 162.82 // box


struct tuningParameters {

  float aHome = INITIAL_HOME_ANGLE;
  //double aKp, aKi, aKd;
  //double pKp, pKi, pKd;
  double pKp = 9.0 , pKi = 0.0, pKd = 0.01;
  //pre ollysdouble pKp = 10.0 , pKi = 0.0, pKd = 0.08;
  //double aKp = 45.0, aKi = 5.0, aKd = 0.9;
  double aKp = 18.0, aKi = 5.0, aKd = 0.1;
  double aInput, aOutput;
  double pSetpoint = 0.0, pInput = 0.0, pOutput = 0.0;
  double aSetpoint = aHome;
  int aDeadzone = 2;

  float aOutputMax = 90.0;
  float aOutputMin = -90.0;
  float pOutputMax = 5;
  float pOutputMin = -5;
  float steering_gain = 0.0;

  bool pOutputLimitChanged = false;

  bool skipStartupTimer = false;
  double directMotorSpeed = 0.0;

  bool enable_motors = false;
  int iTune = 99;
  bool tuneA = false, tuneP = false;
  bool posLimitUpdate = false;
  bool updateAnglePID = false;
  bool updatePositionPID = false;
  bool printFlag = false;
  bool printIMU = true;

  bool positionModeEnable = true;
  bool forward = false;
  bool backward = false;
  bool left = false;
  bool right = false;
  
  bool drive_mode_active = false;

};



class serialTuningParser {
  public:
    //    serialTuningParser(PID& PIDa) {
    //      pid_A = &PIDa;
    //      pid_P = &PIDp;
    //    }
    //    //   serialTuningParser( HardwareSerial& device) {
    //      hwStream = &device;
    //    }
    serialTuningParser() {}; //char* serialTuningMessage) {
    //initialise ?
    //    }
    //void begin(uint32_t baudRate);
    void parse_message(const char* serialTuningMessage);
    //void serialTuningParser::parse_message(const char* message) {

    struct tuningParameters parameters;
  private:
    void update_parameters(struct tuningParameters parameters);
    void print_multi_float(String variable_name, float old_value, float new_value);
    void print_multi_bool(String variable_name, bool old_value, bool new_value);
    void print_all();
    void print_changes();
    struct tuningParameters last_parameters;
    char key_a = '\0';
    char key_b = '\0';
};



#endif
