#include "parse_serial_tuning.h"
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

bool stopFlag = false, angleFlag = true, printTimeFlag = false;
bool pFlag = false, iFlag = false, dFlag = false;
int iTune = 99;
bool printFlag = false, printFlag7 = true, printFlag8 = true;
bool tuneA = false, tuneP = false;
bool posLimitUpdate = false;

//serialTuningParser::serialTuningParser(){ {//char* serialTuningMessage) {
//  key_a = '\0';
//  key_b = '\0';
//}


void serialTuningParser::print_multi(String variable_name, float old_value, float new_value){
    String prefix = "Old " + variable_name + ": ";
    Serial.println(prefix);
    Serial.print("Old aKp: ");
    Serial.print(parameters.aKp);
    Serial.print("New aKp: ");
    Serial.println(parameters.aKp);
}

void serialTuningParser::print_changes() {
  if (last_parameters.aKp != parameters.aKp) {
    Serial.print("Old aKp: ");
    Serial.print(parameters.aKp);
    Serial.print("New aKp: ");
    Serial.println(parameters.aKp);
  }


}

void serialTuningParser::parse_message(const char* message) {
  struct tuningParameters parameters;

  // fill key_a and key_b (if needed) variables before continuing
  if (key_a == '\0') {
    key_a = *message;
    return;
  } else if ((key_b == '\0') && (key_a != 's' || key_a != 'g' || key_a != 'h' || key_a != '#')) {
    key_b = *message;
    return;
  }

  // outer state machine
  switch (key_a) {
    case 'a':
      // inner state machine
      switch (key_b) {
        case 'p':
          parameters.aKp = Serial3.parseFloat();
          break;
        case 'i':
          parameters.aKi = Serial3.parseFloat();
          break;
        case 'd':
          parameters.aKd = Serial3.parseFloat();
          break;
        default:
          break;
      }
      break;
    case '#':
      parameters.aSetpoint = Serial3.parseFloat();
      break;
    case 'g':
      parameters.enable_motors = true;
      break;
    case 's':
      parameters.enable_motors = false;
      break;
  }

  key_a = '\0';
  key_b = '\0';
}
