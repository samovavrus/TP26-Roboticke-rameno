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

  lcd.begin(16,2);
  lcd.backlight();
  lcd.clear();

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  while (1) {

    int joyX = analogRead(A0);
    int joyY = analogRead(A1);

    lcd.setCursor(0,0);
    lcd.print("X:");
    lcd.print(joyX);
    lcd.print(" ");

    lcd.setCursor(8,0);
    lcd.print("Y:");
    lcd.print(joyY);
    lcd.print(" ");

    lcd.setCursor(0,1);

    if(!digitalRead(2)) lcd.print("A ");
    else if(!digitalRead(3)) lcd.print("B ");
    else if(!digitalRead(4)) lcd.print("C ");
    else if(!digitalRead(5)) lcd.print("D ");
    else if(!digitalRead(6)) lcd.print("E ");
    else if(!digitalRead(8)) lcd.print("JOY ");
    else lcd.print("None ");

    vTaskDelay(pdMS_TO_TICKS(200));
  }
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
  static const RotationY R2;  // kĺb 2 – zdvíha rameno
  static const RotationY R3;  // kĺb 3 – ohýba rameno
  static const RotationX R4;  // kĺb 4 – rotuje predlaktie
  static const RotationY R5;  // kĺb 5 – ohýba zápästie
  static const RotationX R6;  // kĺb 6 – rotuje efektor

  const std::array<const RotationMatrix*, 6> joint_axes = { &R1, &R2, &R3, &R4, &R5, &R6 };

  // Dĺžky článkov [m] – T1..T6 z PDF (uprav podľa fyzického robota)
  const std::array<float, 6> link_lengths = {
    0.100f,   // T1
    0.000f,   // T2
    0.150f,   // T3
    0.135f,   // T4
    0.000f,   // T5
    0.070f    // T6
  };

  // Vytvorenie objektu kinematiky
  Robot::RobotKinematics<6> kinematics(link_lengths, joint_axes);

  // --- Test priamej kinematiky ---
  Matrix<6,1> theta = Matrix<6,1>::Zero();  // všetky kĺby v nulovej polohe

  auto result = kinematics.forwardKinematics(theta);
  Matrix<3,1> pos = result.first;
  Matrix<3,3> rot = result.second;

  Serial.print("FK x="); Serial.println(pos(0), 4);
  Serial.print("FK y="); Serial.println(pos(1), 4);
  Serial.print("FK z="); Serial.println(pos(2), 4);
  // Očakávané pri theta=0: x=0, y=0, z=sum(L) = 0.455 m

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

