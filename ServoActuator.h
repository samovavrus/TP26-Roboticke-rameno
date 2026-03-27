#include <vector>

#pragma once

#include "robot.h"
#include "src\PCA9685\PCA9685.h"

/**
 * @class RobotServoController
 * @brief Hardware interface for servo motor control via PCA9685 PWM driver
 * 
 * Implements the JointActuators interface to convert computed joint angles
 * into PWM control signals for servo motors. Handles:
 * - Joint angle to PWM conversion with calibration parameters
 * - Servo frequency initialization and PWM channel configuration
 * - Joint angle limit enforcement to prevent hardware damage
 * 
 * Each joint requires:
 * - Minimum and maximum angle constraints
 * - Calibration gain and offset factors
 * 
 * Works with STM32 and I2C-based PCA9685 16-channel PWM expander for
 * simultaneous multi-servo control with precise timing.
 */
class RobotServoController : public Robot::JointActuators {
public:

  /**
   * @brief Initializes servo controller with I2C bus and calibration parameters
   * @param Wire_ Reference to the I2C bus (TwoWire) for PCA9685 communication
   * @param parameters_ Vector of actuator_parameters (one per joint) containing:
   *                    - minAngle, maxAngle: joint limits in radians
   *                    - gain, offset: calibration factors for angle-to-PWM conversion
   * 
   * Does not take ownership of the Wire_ reference; caller must manage the I2C bus.
   * Parameters are stored as a reference; the caller must ensure the vector
   * remains valid for the lifetime of this object.
   */
  RobotServoController(TwoWire& Wire_, const std::vector<actuator_parameters>& parameters_)
    : Robot::JointActuators(), pwmController(Wire_), parameters(parameters_) {
    }

  /**
   * @brief Initializes PWM driver hardware and configures servo frequency
   * 
   * Operations performed:
   * 1. Reset all PCA9685 devices on the I2C bus
   * 2. Initialize PCA9685 registers and communication
   * 3. Set PWM frequency to standard servo frequency (typically 50 Hz)
   * 
   * Must be called once during system initialization before sending commands
   * to servos. Typically invoked in the main setup() or initialization task.
   */
  void init() {
    pwmController.resetDevices();
    pwmController.init();
    pwmController.setPWMFreqServo();
  }

  /**
   * @brief Writes joint angles to all servo actuators
   * @param angles Array of joint angles in radians
   * @param n Number of joints (array size; should match configured DOF)
   * @return true if all angles were within valid ranges and successfully written to hardware,
   *         false if any angle exceeded its joint limits (no angles are written in this case)
   * 
   * Conversion process for each joint i:
   * 1. Check: angle_min ≤ angles[i] ≤ angle_max
   * 2. If violated: abort and return false (fail-safe behavior)
   * 3. If valid: convert to degrees: deg_i = (angles[i] - offset_i) * gain_i * 180/π
   * 4. Map degrees to PWM pulse width using servo calibration
   * 5. Write PWM value to PCA9685 channel i
   * 
   * @note Operates in a fail-safe manner: if ANY angle violates its constraints,
   *       no PWM values are written. This prevents partial joint movements that
   *       could cause unsafe arm configurations.
   */
  bool writeAngles(const float* angles, std::size_t n) {
    for (int i = 0; i < n; i++) {
      if (angles[i] > parameters[i].maxAngle || angles[i] < parameters[i].minAngle) return false;
      float degrees = (angles[i] - parameters[i].offset) * parameters[i].gain * 180.0f / PI;
      pwmController.setChannelPWM(i, pwmServo.pwmForAngle(degrees));
    }
    return true;
  }


private:
  PCA9685 pwmController;       ///< Low-level PCA9685 I2C PWM driver instance
  PCA9685_ServoEval pwmServo;  ///< Servo helper: maps degree values to PWM pulse widths
  std::vector<actuator_parameters> parameters; ///< Joint configuration: one entry per servo (gain, offset, limits)
};

