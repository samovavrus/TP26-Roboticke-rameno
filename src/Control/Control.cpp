/**
 * @file Control.cpp
 * @brief Implementation of the main robot control task and helpers.
 */
#include <Arduino.h>
#undef B1
#undef B0
#undef F

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <STM32FreeRTOS.h>

#include "Control.h"
#include "../UI/ui_screens.h"
#include "../../ServoActuator.h"
#include "../../robot.h"
#include "../Trajectory/Trajectory.h"

/// @brief FreeRTOS task handle for the main control task.
TaskHandle_t HandleTaskControl;
/// @brief Queue for UI -> Control commands.
QueueHandle_t gUiToControlQueue = NULL;
/// @brief Queue for Control -> UI state updates.
QueueHandle_t gControlToUiQueue = NULL;

/**
 * @brief Convert roll-pitch-yaw angles to a rotation matrix.
 * @param roll Rotation about X axis in radians.
 * @param pitch Rotation about Y axis in radians.
 * @param yaw Rotation about Z axis in radians.
 * @return 3x3 rotation matrix.
 */
static Matrix<3, 3> rpyToRotation(const float roll, const float pitch, const float yaw) {
  return (
    Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
  ).toRotationMatrix();
}

/**
 * @brief Convert a rotation matrix to roll-pitch-yaw angles.
 * @param rot 3x3 rotation matrix.
 * @param roll Output roll angle in radians.
 * @param pitch Output pitch angle in radians.
 * @param yaw Output yaw angle in radians.
 */
static void rotationToRpy(const Matrix<3, 3>& rot, float& roll, float& pitch, float& yaw) {
  pitch = asinf(-rot(2, 0));
  roll = atan2f(rot(2, 1), rot(2, 2));
  yaw = atan2f(rot(1, 0), rot(0, 0));
}

/**
 * @brief FreeRTOS task for kinematics, IK solving, and servo actuation.
 * @param pvParameters Unused task parameter.
 */
void TaskControl(void* pvParameters) {
  TwoWire Wire2(PF0, PF1);
  Wire2.begin();
  Wire2.setClock(400000);

  std::vector<actuator_parameters> servo_parameters;
  servo_parameters.resize(6);
  servo_parameters[0] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, 0 };
  servo_parameters[1] = { -5.0 / 10.0 * PI, 5.2 / 10.0 * PI, 2.0 / 3.0, 0.015 };
  servo_parameters[2] = { -8.2 / 10.0 * PI, 0.636 * PI, -2.0 / 3.0, 0.12 };
  servo_parameters[3] = { -7.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, -0.2 };
  servo_parameters[4] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, 0.0 };
  servo_parameters[5] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, -2.0 / 3.0, 0.0 };

  RobotServoController servo_controller(Wire2, servo_parameters);
  servo_controller.init();

  static const RotationZ R1;
  static const RotationX R2;
  static const RotationX R3;
  static const RotationX R4;
  static const RotationZ R5;
  static const RotationY R6;

  const std::array<const RotationMatrix*, 6> joint_axes = { &R1, &R2, &R3, &R4, &R5, &R6 };

  const std::array<Matrix<3, 1>, 6> link_translations = {
    (Matrix<3,1>() << 0.0f, 0.0f, 0.094f).finished(),
    (Matrix<3,1>() << 0.0f, 0.0f, 0.105f).finished(),
    (Matrix<3,1>() << 0.0f, 0.0f, 0.147f).finished(),
    (Matrix<3,1>() << 0.0f, 0.009f, 0.097f).finished(),
    (Matrix<3,1>() << 0.015f, -0.0215f, 0.0f).finished(),
    (Matrix<3,1>() << 0.0f, 0.0f, 0.070f).finished()
  };

  Robot::RobotKinematics<6> kinematics(link_translations, joint_axes);

  Matrix<6,1> current_theta = Matrix<6,1>::Zero();
  current_theta(0) = CONTROL_START_THETA_0_RAD;
  current_theta(1) = CONTROL_START_THETA_1_RAD;
  current_theta(2) = CONTROL_START_THETA_2_RAD;
  current_theta(3) = CONTROL_START_THETA_3_RAD;
  current_theta(4) = CONTROL_START_THETA_4_RAD;
  current_theta(5) = CONTROL_START_THETA_5_RAD;

  Matrix<3, 1> current_pos = Matrix<3, 1>::Zero();
  Matrix<3, 3> current_rot = Matrix<3, 3>::Identity();

  auto refreshPoseFromTheta = [&]() {
    const auto fk = kinematics.forwardKinematics(current_theta);
    current_pos = fk.first;
    current_rot = fk.second;
  };

  refreshPoseFromTheta();

  if (!servo_controller.writeAngles(current_theta.data(), 6)) {
    Serial.println("Startup pose is outside configured servo limits.");
  }

  float tol_pos = CONTROL_IK_TOL_POS;
  float tol_ori = CONTROL_IK_TOL_ORI;
  float lambda = CONTROL_IK_LAMBDA;
  int max_iter = CONTROL_IK_MAX_ITER;

  UiControlCommandMessage latest_command = {};
  latest_command.mode = UI_CONTROL_MODE_POSE;
  for (int i = 0; i < 6; ++i) {
    latest_command.joint_rad[i] = current_theta(i);
  }
  latest_command.x = current_pos(0);
  latest_command.y = current_pos(1);
  latest_command.z = current_pos(2);
  rotationToRpy(current_rot, latest_command.roll, latest_command.pitch, latest_command.yaw);

  auto publishState = [&](uint8_t status) {
    UiControlStateMessage state = {};
    state.status = status;

    for (int i = 0; i < 6; ++i) {
      state.joint_rad[i] = current_theta(i);
    }

    state.x = current_pos(0);
    state.y = current_pos(1);
    state.z = current_pos(2);
    rotationToRpy(current_rot, state.roll, state.pitch, state.yaw);

    if (gControlToUiQueue != NULL) {
      (void)xQueueOverwrite(gControlToUiQueue, &state);
    }
  };

  publishState(UI_CONTROL_STATUS_OK);

  uint8_t lastStatus = 0xFF;

  while (1) {
    if (gUiToControlQueue != NULL) {
      UiControlCommandMessage msg;
      while (xQueueReceive(gUiToControlQueue, &msg, 0) == pdPASS) {
        latest_command = msg;
      }
    }


    // --- Trajectory override ---
    static bool s_wasUsingTrajectory = false;
    bool isUsingTrajectory = false;

    if (gTrajectoryTargetPose.valid && gTrajectoryTargetPose.active) {
      isUsingTrajectory = true;
      latest_command.mode = UI_CONTROL_MODE_POSE;

      latest_command.x = gTrajectoryTargetPose.x;
      latest_command.y = gTrajectoryTargetPose.y;
      latest_command.z = gTrajectoryTargetPose.z;

      latest_command.roll  = gTrajectoryTargetPose.roll;
      latest_command.pitch = gTrajectoryTargetPose.pitch;
      latest_command.yaw   = gTrajectoryTargetPose.yaw;
    }

    if (isUsingTrajectory && !s_wasUsingTrajectory) {
      Serial.println("Control using trajectory target");
    } else if (!isUsingTrajectory && s_wasUsingTrajectory) {
      Serial.println("Control using manual UI");
    }
    s_wasUsingTrajectory = isUsingTrajectory;

    const uint8_t activeMode = (latest_command.mode == UI_CONTROL_MODE_JOINT)
      ? UI_CONTROL_MODE_JOINT
      : UI_CONTROL_MODE_POSE;

    uint8_t loopStatus = UI_CONTROL_STATUS_OK;

    if (activeMode == UI_CONTROL_MODE_JOINT) {
      Matrix<6, 1> requested_theta = Matrix<6, 1>::Zero();
      for (int i = 0; i < 6; ++i) {
        requested_theta(i) = latest_command.joint_rad[i];
      }

      const bool in_limits = servo_controller.writeAngles(requested_theta.data(), 6);
      if (in_limits) {
        current_theta = requested_theta;
        refreshPoseFromTheta();
      } else {
        loopStatus = UI_CONTROL_STATUS_OUT_OF_LIMITS;
      }
    } else {
      Matrix<3, 1> target_pos;
      target_pos << latest_command.x, latest_command.y, latest_command.z;
      const Matrix<3, 3> target_rot = rpyToRotation(
        latest_command.roll,
        latest_command.pitch,
        latest_command.yaw
      );

      Matrix<6, 1> solved_theta = current_theta;
      const bool ik_success = kinematics.SolveIK(
        target_pos,
        target_rot,
        solved_theta,
        tol_pos,
        tol_ori,
        lambda,
        max_iter
      );

      if (ik_success) {
        const bool in_limits = servo_controller.writeAngles(solved_theta.data(), 6);
        if (in_limits) {
          current_theta = solved_theta;
          refreshPoseFromTheta();
        } else {
          loopStatus = UI_CONTROL_STATUS_OUT_OF_LIMITS;
        }
      } else {
        loopStatus = UI_CONTROL_STATUS_IK_FAILED;
      }
    }

    if (loopStatus != lastStatus) {
      if (loopStatus == UI_CONTROL_STATUS_IK_FAILED) {
        Serial.println("IK nenasla riesenie, ostava posledny validny stav.");
      } else if (loopStatus == UI_CONTROL_STATUS_OUT_OF_LIMITS) {
        Serial.println("Pozadovany povel je mimo limitov serv, ostava posledny validny stav.");
      }
      lastStatus = loopStatus;
    }

    publishState(loopStatus);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
