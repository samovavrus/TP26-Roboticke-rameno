#include <vector>

#pragma once

#include "robot.h"
#include "src\PCA9685\PCA9685.h"

class RobotServoController : public Robot::JointActuators {
public:

  RobotServoController(TwoWire& Wire_, const std::vector<actuator_parameters>& parameters_)
    : Robot::JointActuators(), pwmController(Wire_), parameters(parameters_) {
    }

  void init() {
    pwmController.resetDevices();
    pwmController.init();
    pwmController.setPWMFreqServo();
  }

  bool writeAngles(const float* angles, std::size_t n) {
    for (int i = 0; i < n; i++) {
      if (angles[i] > parameters[i].maxAngle || angles[i] < parameters[i].minAngle) return false;
      float degrees = (angles[i] - parameters[i].offset) * parameters[i].gain * 180.0f / PI;
      pwmController.setChannelPWM(i, pwmServo.pwmForAngle(degrees));
    }
    return true;
  }


private:
  PCA9685 pwmController;       ///< Low-level PCA9685 PWM driver instance
  PCA9685_ServoEval pwmServo;  ///< Helper to map degrees → PWM values
  std::vector<actuator_parameters> parameters;
};

