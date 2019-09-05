#include "parse_serial_tuning.h"


void serialTuningParser::print_multi_float(String variable_name, float old_value, float new_value) {
  String prefix = "Old " + variable_name + ": ";
  String midfix = ". New " + variable_name + ": ";
  Serial5.print(prefix);
  Serial5.print(old_value);
  Serial5.print(midfix);
  Serial5.println(new_value);
}

void serialTuningParser::print_multi_bool(String variable_name, bool old_value, bool new_value) {
  String prefix = "Old " + variable_name + ": ";
  String midfix = ". New " + variable_name + ": ";
  String str = "Old " + variable_name + ": " +  old_value + ". Now: " + new_value;
  Serial5.print(prefix);
  Serial5.print(old_value);
  Serial5.print(midfix);
  Serial5.println(new_value);
}

void serialTuningParser::print_all() {
  String setpoint = "s - setpoint: ";
  String akp = "p - aKp: ";
  String aki = "i - aKi: ";
  String akd = "d - aKd: ";
  Serial5.print(setpoint);
  Serial5.println(parameters.aSetpoint);

  Serial5.print(akp);
  Serial5.println(parameters.aKp);

  Serial5.print(aki);
  Serial5.println(parameters.aKi);

  Serial5.print(akd);
  Serial5.println(parameters.aKd);
  parameters.printFlag = true;
}

void serialTuningParser::print_changes() {
  if (last_parameters.aKp != parameters.aKp) {
    print_multi_float("aKp", last_parameters.aKp, parameters.aKp);
  }

  if (last_parameters.aKi != parameters.aKi) {
    print_multi_float("aKi", last_parameters.aKi, parameters.aKi);
  }

  if (last_parameters.aKd != parameters.aKd) {
    print_multi_float("aKd", last_parameters.aKd, parameters.aKd);
  }

  if (last_parameters.aSetpoint != parameters.aSetpoint) {
    print_multi_float("setpoint", last_parameters.aSetpoint, parameters.aSetpoint);
  }

  if (last_parameters.enable_motors != parameters.enable_motors) {
    print_multi_bool("Enable:", last_parameters.enable_motors, parameters.enable_motors);
  }
  last_parameters.aKp = parameters.aKp;
}

void serialTuningParser::parse_message(const char* message) {
  //  struct tuningParameters parameters;
  //Serial5.print("in parse message: ");
  //Serial5.println(message);
  char* separator;
  float value;

  // fill key_a and key_b (if needed) variables before continuing
  if (key_a == '\0') {
    key_a = *message;
    switch (key_a) {
      case 'a':
        Serial5.println("Tuning angle, select p i or d");
        break;
      case 's':
        Serial5.print("Tuning setpoint. e.g. s=167.5");
        break;
      case 'p':
        print_all();
        key_a = '\0';
        key_b = '\0';
        break;
      case 'o':
        parameters.printIMU = !parameters.printIMU;
        key_a = '\0';
        key_b = '\0';
        break;
      case 'g':
        parameters.enable_motors = true;
        print_changes();
        last_parameters = parameters;
        key_a = '\0';
        key_b = '\0';
        break;
      case 'c':
        parameters.enable_motors = false;
        print_changes();
        last_parameters = parameters;
        key_a = '\0';
        key_b = '\0';
        break;
      default:
        break;
    }
    return;
  } else if ((key_a == 's') || ((key_b == '\0') && (key_a != 's' || key_a != 'g' || key_a != 'h' || key_a != '#'))) {

    char* separator = strchr(message, '=');
    char ID = message[0];
    key_b = ID;
    ++separator;
    value = atof(separator);
    Serial5.print("val: ");
    Serial5.println(value);
  }


  Serial5.print("Key_a: ");
  Serial5.print(key_a);
  Serial5.print(" Key_b: ");
  Serial5.println(key_b);


  // outer state machine
  switch (key_a) {
    case 'a':
      // inner state machine
      switch (key_b) {
        case 'p':
          parameters.aKp = value;
          parameters.updateAnglePID = true;
          break;
        case 'i':
          parameters.aKi = value;
          parameters.updateAnglePID = true;
          break;
        case 'd':
          parameters.aKd = value;
          parameters.updateAnglePID = true;
          break;
        default:
          break;

      }
      break;
    case 's':
      parameters.aSetpoint = value;
      break;

  }

  print_changes();
  last_parameters = parameters;
  key_a = '\0';
  key_b = '\0';

}
