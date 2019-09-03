#include "parse_serial_tuning.h"


void serialTuningParser::print_multi_float(String variable_name, float old_value, float new_value) {
  String prefix = "Old " + variable_name + ": ";
  String midfix = ". New " + variable_name + ": ";
  Serial3.print(prefix);
  Serial3.print(old_value);
  Serial3.print(midfix);
  Serial3.println(new_value);
}


void serialTuningParser::print_changes() {
  Serial3.println("serial3printchanges");
  if (last_parameters.aKp != parameters.aKp) {
    print_multi_float("aKp", last_parameters.aKp, parameters.aKp);
    last_parameters.aKp = parameters.aKp;
  }

  if (last_parameters.aKp != parameters.aKp) {
    print_multi_float("aKp", last_parameters.aKp, parameters.aKp);
  }

  if (last_parameters.aKp != parameters.aKp) {
    print_multi_float("aKp", last_parameters.aKp, parameters.aKp);
  }

  if (last_parameters.aSetpoint != parameters.aSetpoint) {
    print_multi_float("aKp", last_parameters.aSetpoint, parameters.aSetpoint);
  }

}

void serialTuningParser::parse_message(const char* message) {
  //  struct tuningParameters parameters;
  //Serial3.print("in parse message: ");
  //Serial3.println(message);
  char* separator;

  // fill key_a and key_b (if needed) variables before continuing
  if (key_a == '\0') {
    key_a = *message;
    switch (key_a) {
      case 'a':
        Serial3.println("Tuning angle, select p i or d");
        break;
      default:
        Serial3.print("Key_a: ");
        Serial3.println(key_a);
        break;
    }
    return;
  } else if ((key_b == '\0') && (key_a != 's' || key_a != 'g' || key_a != 'h' || key_a != '#')) {


    // Split the command in two values
    char* separator = strchr(message, '=');

    //  if (separator != 0)
    //  {
    //    *separator = 0;
    char ID = message[0];
    key_b = ID;
    Serial3.print("Key_b: ");
    Serial3.println(key_b);
    ++separator;
    //return;
  }


  Serial3.print("Key_a: ");
  Serial3.print(key_a);
  Serial3.print(" Key_b: ");
  Serial3.println(key_b);


  // outer state machine
  switch (key_a) {
    case 'a':
      // inner state machine
      switch (key_b) {
        case 'p':{
          float value = atof(separator);
          Serial3.print("val: ");
          Serial3.println(value);
          parameters.aKp = value;
          Serial3.println(parameters.aKp);
          //parameters.aKp = Serial3.parseFloat();          
          //Serial3.println(parameters.aKp);
          break;
        }
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

  print_changes();
  last_parameters = parameters;
  key_a = '\0';
  key_b = '\0';

}
