#include "parse_serial_tuning.h"


void serialTuningParser::print_multi_float(String variable_name, float old_value, float new_value) {
  String prefix = "Old " + variable_name + ": ";
  String midfix = ". New " + variable_name + ": ";
  Serial3.print(prefix);
  Serial3.print(old_value);
  Serial3.print(midfix);
  Serial3.println(new_value);
}

void serialTuningParser::print_multi_bool(String variable_name, bool old_value, bool new_value) {
  String prefix = "Old " + variable_name + ": ";
  String midfix = ". New " + variable_name + ": ";
  String str = "Old " + variable_name + ": " +  old_value + ". Now: " + new_value;
  Serial3.print(prefix);
  Serial3.print(old_value);
  Serial3.print(midfix);
  Serial3.println(new_value);
}

void serialTuningParser::print_all() {
  String asetpoint = "s - aSetpoint: ";
  String akp = "p - aKp: ";
  String aki = "i - aKi: ";
  String akd = "d - aKd: ";
  String steering_gain = "Steering gain: ";
  String pkp = "p - pKp: ";
  String pki = "i - pKi: ";
  String pkd = "d - pKd: ";
  String psetpoint = "s - pSetpoint: ";
  String poutputlimit = "o - pOutputMax/Min: +";
  Serial3.print(asetpoint);
  Serial3.print(parameters.aSetpoint);
  Serial3.print(" +/- ");
  Serial3.println(parameters.steering_gain);

  Serial3.print(akp);
  Serial3.println(parameters.aKp);

  Serial3.print(aki);
  Serial3.println(parameters.aKi);

  Serial3.print(akd);
  Serial3.println(parameters.aKd);

  Serial3.print(steering_gain );
  Serial3.println(parameters.steering_gain );

  Serial3.print(pkp);
  Serial3.println(parameters.pKp);

  Serial3.print(pki);
  Serial3.println(parameters.pKi);

  Serial3.print(pkd);
  Serial3.println(parameters.pKd);

  Serial3.print("Position mode ");
  String positionMode = parameters.positionModeEnable ? "enabled" : "disabled";
  Serial3.println(positionMode);

  Serial3.print("Motors ");
  String motors = parameters.positionModeEnable ? "enabled" : "disabled";
  Serial3.println(motors);

  Serial3.print(psetpoint);
  Serial3.println(parameters.pSetpoint);

  Serial3.print(poutputlimit);
  Serial3.print(parameters.pOutputMax);
  Serial3.print(", ");
  Serial3.println(parameters.pOutputMin);

  Serial3.print("aDeadzone: ");
  Serial3.println(parameters.aDeadzone);

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

  if (last_parameters.pKp != parameters.pKp) {
    print_multi_float("pKp", last_parameters.pKp, parameters.pKp);
  }

  if (last_parameters.pKi != parameters.pKi) {
    print_multi_float("pKi", last_parameters.pKi, parameters.pKi);
  }

  if (last_parameters.pKd != parameters.pKd) {
    print_multi_float("pKd", last_parameters.pKd, parameters.pKd);
  }

  if (last_parameters.aSetpoint != parameters.aSetpoint) {
    print_multi_float("aSetpoint", last_parameters.aSetpoint, parameters.aSetpoint);
  }

  if (last_parameters.pSetpoint != parameters.pSetpoint) {
    print_multi_float("pSetpoint", last_parameters.pSetpoint, parameters.pSetpoint);
  }

  if (last_parameters.enable_motors != parameters.enable_motors) {
    print_multi_bool("Enable:", last_parameters.enable_motors, parameters.enable_motors);
  }

  if (last_parameters.steering_gain != parameters.steering_gain) {
    print_multi_float("Steeting gain:", last_parameters.steering_gain, parameters.steering_gain);
  }

  if (last_parameters.positionModeEnable != parameters.positionModeEnable) {
    print_multi_bool("Position loop enable:", last_parameters.positionModeEnable, parameters.positionModeEnable);
  }

  if (last_parameters.pOutputMax != parameters.pOutputMax) {
    print_multi_float("Max angle from pOut:", last_parameters.pOutputMax, parameters.pOutputMax);
  } if (last_parameters.pOutputMin != parameters.pOutputMin) {
    print_multi_float("Min angle from pOut:", last_parameters.pOutputMin, parameters.pOutputMin);
  }

  if (last_parameters.aDeadzone != parameters.aDeadzone) {
    print_multi_float("Deadzone:", last_parameters.aDeadzone, parameters.aDeadzone);
  }



  //last_parameters.aKp = parameters.aKp;
  last_parameters = parameters;
}

void serialTuningParser::parse_message(const char* message) {
  //  struct tuningParameters parameters;
  //Serial3.print("in parse message: ");
  //Serial3.println(message);
  char* separator;
  float value;

  // fill key_a and key_b (if needed) variables before continuing
  if (key_a == '\0') {
    key_a = *message;
    switch (key_a) {
      case 'a':
        Serial3.println("Tuning angle, select p i or d");
        break;
      case 'v':
        Serial3.println("Tuning position, select p i or d");
        break;
      case 's':
        Serial3.print("Tuning setpoint. e.g. s=167.5");
        break;
      case 'd':
        Serial3.print("Drive mode active! Press q to quit");
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
      case 'x':
        parameters.skipStartupTimer = true;
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
    Serial3.print("val: ");
    Serial3.println(value);
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
        case 's':
          parameters.aSetpoint = value;
          break;
        case 'z':
          parameters.aDeadzone = value;
          break;
        case 'm':
          parameters.directMotorSpeed = value;
        default:
          break;

      }
      break;
    case 'v':
      // inner state machine
      switch (key_b) {
        case 'p':
          parameters.pKp = value;
          parameters.updatePositionPID = true;
          break;
        case 'i':
          parameters.pKi = value;
          parameters.updatePositionPID = true;
          break;
        case 'd':
          parameters.pKd = value;
          parameters.updatePositionPID = true;
          break;
        case 'e':
          parameters.positionModeEnable = !parameters.positionModeEnable;
          break;
        case 's':
          parameters.pSetpoint = value;
          break;
        case 'o':
          parameters.pOutputLimitChanged = true;
          parameters.pOutputMax = value;
          parameters.pOutputMin = value * (-1.0);
          break;
        default:
          break;

      }
      break;
    case 'd':
      parameters.drive_mode_active = true;
      parameters.forward = false;
      parameters.backward = false;
      parameters.left = false;
      parameters.right = false;
      switch (key_b) {
        case 'w':
          parameters.forward = true;
          break;
        case 's':
          parameters.backward = true;
          break;
        case 'a':
          parameters.left = true;
          break;
        case 'd':
          parameters.right = true;
          break;
        case 'r':
          parameters.steering_gain += 0.1;
          break;
        case 'f':
          parameters.steering_gain -= 0.1;
          break;
        case 'z':
          parameters.steering_gain = 0.0;
          break;
        case 'q':
          parameters.drive_mode_active = false;
          Serial3.print("Drive mode disabled.");
          key_a = '\0';
          break;
        default:
          break;
      }
      print_changes();
      key_b = '\0';
      return;
      break;

  }

  print_changes();
  last_parameters = parameters;
  key_a = '\0';
  key_b = '\0';

}
