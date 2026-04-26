#include <Arduino.h>
#undef B1
#undef B0
#undef F

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <STM32FreeRTOS.h>
#include "STM32FreeRTOSConfig.h"
#include "src\UI\ui_screens.h"
#include "src\LiquidMenu\LiquidCrystal_I2C.h"
#include "src\LiquidMenu\LiquidMenu.h"
#include "src\Button.h"
#include "src\Joystick.h"
#include "src\Keypad\Keypad.h"
#include "src\DistanceSensor\RangingSensor.h"

#include "ServoActuator.h"
#include "robot.h"


TaskHandle_t HandleTaskControl;
TaskHandle_t HandleTaskUI;
TaskHandle_t HandleTaskSensor;
QueueHandle_t gUiToControlQueue = NULL;
QueueHandle_t gControlToUiQueue = NULL;

static Matrix<3, 3> rpyToRotation(const float roll, const float pitch, const float yaw) {
  return (
    Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
  ).toRotationMatrix();
}

static void rotationToRpy(const Matrix<3, 3>& rot, float& roll, float& pitch, float& yaw) {
  pitch = asinf(-rot(2, 0));
  roll = atan2f(rot(2, 1), rot(2, 2));
  yaw = atan2f(rot(1, 0), rot(0, 0));
}


void setup(void) {

  Serial.begin(250000);
  Wire.begin();

  gUiToControlQueue = xQueueCreate(1, sizeof(UiControlCommandMessage));
  if (gUiToControlQueue == NULL) {
    Serial.println("Failed to create UI->Control queue");
  }

  gControlToUiQueue = xQueueCreate(1, sizeof(UiControlStateMessage));
  if (gControlToUiQueue == NULL) {
    Serial.println("Failed to create Control->UI queue");
  }

  xTaskCreate(TaskControl,
              "Control",
              5000,
              NULL,
              tskIDLE_PRIORITY + 4,
              &HandleTaskControl);

  xTaskCreate(TaskUI,
              "UI",
              1500,
              NULL,
              tskIDLE_PRIORITY + 2,
              &HandleTaskUI);


xTaskCreate(TaskSensor,
              "Sensor",
              1000,
              NULL,
              tskIDLE_PRIORITY + 3,
              &HandleTaskSensor);

vTaskStartScheduler();
}

void loop(void) {
}


void TaskControl(void* pvParameters) {

  TwoWire Wire2(PF0, PF1);
  Wire2.begin();
  Wire2.setClock(400000);

  std::vector<actuator_parameters> servo_parameters;
  servo_parameters.resize(6);
  servo_parameters[0] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, 0 };
  servo_parameters[1] = { -5.0 / 10.0 * PI, 5.2 / 10.0 * PI, 2.0 / 3.0, 0.015 };
  servo_parameters[2] = { -8.2 / 10.0 * PI, 5.0 / 10.0 * PI, -2.0 / 3.0, 0.12 };
  servo_parameters[3] = { -7.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, -0.2 };
  servo_parameters[4] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, 2.0 / 3.0, 0.0 };
  servo_parameters[5] = { -6.0 / 10.0 * PI, 6.0 / 10.0 * PI, -2.0 / 3.0, 0.0 };

  RobotServoController servo_controller(Wire2, servo_parameters);
  servo_controller.init();

  // --- Definícia kinematiky robota ---
  // Rotačné osi každého kĺbu (uprav podľa konštrukcie robota)
  static const RotationZ R1;  // kĺb 1 – otáča v základni
  static const RotationX R2;  // kĺb 2 – zdvíha rameno
  static const RotationX R3;  // kĺb 3 – ohýba rameno
  static const RotationX R4;  // kĺb 4 – rotuje predlaktie
  static const RotationZ R5;  // kĺb 5 – ohýba zápästie
  static const RotationY R6;  // kĺb 6 – rotuje efektor

  const std::array<const RotationMatrix*, 6> joint_axes = { &R1, &R2, &R3, &R4, &R5, &R6 };

  // 3D Translačné vektory podľa výkresu. 
  const std::array<Matrix<3, 1>, 6> link_translations = {
    (Matrix<3,1>() << 0.0f, 0.0f, 0.094f).finished(),  // rotacia okolo Z (servo1) a rotacia okolo X (servo2)
    (Matrix<3,1>() << 0.0f, 0.0f, 0.105f).finished(),  // rotacia okolo X (servo3)
    (Matrix<3,1>() << 0.0f, 0.0f, 0.147f).finished(),  // rotacia okolo X (servo4)
    (Matrix<3,1>() << 0.0f, 0.009f, 0.097f).finished(),  // rotacia okolo Z (servo5)
    // offset zápästia. Ak je mimo osi do boku v smere X, zmeň 0.0 na 0.0125 atd...
    (Matrix<3,1>() << 0.015f, -0.0215f, 0.0f).finished(), // rotacia okolo X (servo6)
    (Matrix<3,1>() << 0.0f, 0.0f, 0.070f).finished()   // efektor
  };

  // Vytvorenie objektu kinematiky - ZMENIŤ link_lengths na link_translations!
  Robot::RobotKinematics<6> kinematics(link_translations, joint_axes);

  Matrix<6,1> current_theta = Matrix<6,1>::Zero();
  current_theta(0) = 0.0f;
  current_theta(1) = M_PI / 4.0f;
  current_theta(2) = M_PI / 4.0f;
  current_theta(3) = 0.0f;
  current_theta(4) = 0.0f;
  current_theta(5) = 0.0f;

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

  // Parametre pre IK solver.
  float tol_pos = 1e-4f;  // Tolerancia polohy (napr. 0.1 mm)
  float tol_ori = 1e-3f;  // Tolerancia orientácie 
  float lambda = 0.01f;   // Regularizačný/tlmiaci faktor (damping factor)
  int max_iter = 50;      // Maximálny počet iterácií

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

void TaskSensor(void* pvParameters) {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  
  const uint32_t Ts_ms = 50; 
  // I2C bus: SDA=PB4 (D5), SCL=PA8 (D7)
  TwoWire Wire3(PB4, PA8);
  Wire3.begin();
  Wire3.setClock(400000);
  RangingSensor rangingSensor(Wire3);
  if (!rangingSensor.init(Ts_ms, RangingSensor::DistanceMode::Short)) {
    Serial.println("RangingSensor init failed!");
    vTaskDelete(NULL);
    return;
  }

  if (rangingSensor.initSD(PE4)) {
    Serial.println("SD Card initialized for RangingSensor!");
    rangingSensor.enableLogging(true, true); // 1.param: zapne logovanie, 2.param: vymaze existujuci subor
  } else {
    Serial.println("SD Card init failed! Logging disabled.");
  }

  rangingSensor.startContinuous();
  Serial.println("RangingSensor initialized");

  while (1) {
    MeasurementData measurement = rangingSensor.read(x, y, z, roll, pitch, yaw);
    
 /*   if (measurement.valid) {
      Serial.print("Distance: ");
      Serial.print(measurement.distance_mm);
      Serial.print(" mm | Pose: (");
      Serial.print(measurement.pose.x, 3); Serial.print(", ");
      Serial.print(measurement.pose.y, 3); Serial.print(", ");
      Serial.print(measurement.pose.z, 3); Serial.println(")");
    }*/

    vTaskDelay(pdMS_TO_TICKS(Ts_ms));
  }

}

