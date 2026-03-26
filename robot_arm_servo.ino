#include <Arduino.h>
#undef B1
#undef B0
#undef F

#include "Eigen/Dense"

#include <STM32FreeRTOS.h>
#include "STM32FreeRTOSConfig.h"
#include "ui_screens.h"
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

  pinMode(2, INPUT_PULLUP); // A button

  int screen = 0;
  bool lastButtonState = HIGH;

  while (1) {

    // --- simulované hodnoty (zatiaľ)
    float x = analogRead(A0) / 10.0;
    float y = analogRead(A1) / 10.0;
    float z = (x+y)/2;

    float roll  = x/5;
    float pitch = y/5;
    float yaw   = z/5;

    // --- čítanie tlačidla A
    bool currentState = digitalRead(2);

    // detekcia stlačenia (falling edge)
    if (lastButtonState == HIGH && currentState == LOW) {
      screen = !screen;   // prepni obrazovku
    }

    lastButtonState = currentState;

    // --- vykreslenie
    if (screen == 0) {
      drawXYZ(lcd, x, y, z);
    } else {
      drawRPY(lcd, roll, pitch, yaw);
    }

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

  Serial.print("FK x="); Serial.println(pos(0), 4);
  Serial.print("FK y="); Serial.println(pos(1), 4);
  Serial.print("FK z="); Serial.println(pos(2), 4);
  // Očakávané pri theta=0: x=0, y=0, z=sum(L) = 0.455 m
  auto J = kinematics.getJacobian(theta);   // 3x6 pozicny Jacobian

  Serial.println("J:");
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 6; ++c) {
      Serial.print(J(r, c), 6);
      if (c < 5) Serial.print(" ");
    }
    Serial.println();
  }

    // 1. Zadefinovanie cieľovej polohy pr (napríklad [X, Y, Z] v metroch)
  Matrix<3, 1> target_pos;
  target_pos << 0.15f,  // X
                0.0f,   // Y
                0.20f;  // Z

  // 2. Zadefinovanie cieľovej orientácie Rr (napríklad len identita = rovnaká orientácia ako v nulovej polohe)
  Matrix<3, 3> target_rot = Matrix<3, 3>::Identity(); 

  // 3. Počiatočný odhad kĺbov 'theta'
  // Najlepšie je sem dať AKTUÁLNE natočenie (teraz pre test dáme samé nuly)
  Matrix<6, 1> current_theta = Matrix<6, 1>::Zero();

  // 4. Pripravenie parametrov pre solver
  float tol_pos = 1e-4f;  // Tolerancia polohy (napr. 0.1 mm)
  float tol_ori = 1e-3f;  // Tolerancia orientácie 
  float lambda = 0.01f;   // Regularizačný/tlmiaci faktor (damping factor)
  int max_iter = 50;      // Maximálny počet iterácií

  // 5. Samotné zavolanie funkcie
  bool success = kinematics.SolveIK(
      target_pos, 
      target_rot, 
      current_theta, // Pozor, táto premenná sa vo vnútri funkcie upraví na výsledok!
      tol_pos, 
      tol_ori, 
      lambda, 
      max_iter
  );

  // 6. Kontrola výsledku
  if (success) {
      Serial.println("IK úspešne našla riešenie!");
      Serial.println("Nové uhly kĺbov (v radiánoch):");
      for(int i = 0; i < 6; i++) {
          Serial.println(current_theta(i), 4);
      }
  } else {
      Serial.println("IK zlyhala / nekonvergovala.");
  }

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10));
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
    vTaskDelay(pdMS_TO_TICKS(10));
  }

}

