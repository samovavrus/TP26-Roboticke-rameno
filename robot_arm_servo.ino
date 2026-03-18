#include <Arduino.h>
#undef B1
#undef B0
#undef F

#include "Eigen/Dense"

#include <STM32FreeRTOS.h>
#include "STM32FreeRTOSConfig.h"
#include "src\LiquidMenu\LiquidCrystal_I2C.h"
#include "src\LiquidMenu\LiquidMenu.h"
#include "src\Button.h"
#include "src\Joystick.h"
#include "src\Keypad\Keypad.h"

#include "ServoActuator.h"
#include "robot.h"


TaskHandle_t HandleTaskControl;
TaskHandle_t HandleTaskUI;
TaskHandle_t HandleTaskSensor;


void setup(void) {

  Serial.begin(250000);
  Wire.begin();

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


void TaskUI(void* pvParameters) {


  LiquidCrystal_I2C lcd(0x27, 16, 2);
  lcd.init();
  lcd.backlight();

  const char keys[4][4] = {
    { '1', '2', '3', 'A' },
    { '4', '5', '6', 'B' },
    { '7', '8', '9', 'C' },
    { '.', '0', '-', 'D' }
  };
  byte rowPins[4] = { 4, 5, 6, 7 };
  byte colPins[4] = { 8, 9, 10, 11 };

  Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, 4, 4);
  Button button_variable(12, true);
  Joystick joystick(A1, A0);


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

  // --- Test priamej kinematiky ---
  Matrix<6,1> theta = Matrix<6,1>::Zero();  // všetky kĺby v nulovej polohe

  auto result = kinematics.forwardKinematics(theta);
  Matrix<3,1> pos = result.first;
  Matrix<3,3> rot = result.second;

  auto J = kinematics.getJacobian(theta);   // 3x6 pozicny Jacobian

  Serial.println("J:");
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 6; ++c) {
      Serial.print(J(r, c), 6);
      if (c < 5) Serial.print(" ");
    }
    Serial.println();
  }

  while (1) {

  }
}

#include "src\VL53L1X\VL53L1X.h"

void TaskSensor(void* pvParameters) {

  const float Ts = 0.1;
  TwoWire Wire3(PB4, PA8);
  Wire3.begin();
  Wire3.setClock(400000);
  VL53L1X sensor_TOF;
  sensor_TOF.setBus(&Wire3);

  sensor_TOF.init();
  sensor_TOF.setMeasurementTimingBudget((uint32_t)(0.8 * Ts * 1000.0));
  sensor_TOF.setDistanceMode(VL53L1X::Short);
  sensor_TOF.setTimeout((uint16_t)(Ts * 1.5 * 1000.0));
  sensor_TOF.startContinuous((uint32_t)(Ts * 1000.0));

  while (1) {

  }

}

