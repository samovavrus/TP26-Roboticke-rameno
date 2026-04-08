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
#include "src\DistanceSensor\RangingSensor.h"

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

  pinMode(2, INPUT_PULLUP); // Button A - change screen
  pinMode(3, INPUT_PULLUP); // Button B - change mode

  int screen = 0;
  int mode = 0;

  bool lastButtonA = HIGH;
  bool lastButtonB = HIGH;

  const char* pairXYZ[3] = {"XY","YZ","ZX"};
  const char* pairRPY[3] = {"RP","PY","YR"};

  // ---- persistent variables
  float x = 0;
  float y = 0;
  float z = 0;

  float roll  = 0;
  float pitch = 0;
  float yaw   = 0;

  while (1) {

    int joyX = analogRead(A0);
    int joyY = analogRead(A1);
    Serial.println(joyX);
    Serial.println(joyY);

    const int CENTER = 520;
    const int DEADZONE = 20;

    float dx = 0;
    float dy = 0;

    if (joyX > CENTER + DEADZONE)
      dx = (joyX - (CENTER + DEADZONE)) * 0.05;

    else if (joyX < CENTER - DEADZONE)
      dx = (joyX - (CENTER - DEADZONE)) * 0.05;

    if (joyY > CENTER + DEADZONE)
      dy = (joyY - (CENTER + DEADZONE)) * 0.05;

    else if (joyY < CENTER - DEADZONE)
      dy = (joyY - (CENTER - DEADZONE)) * 0.05;

    // ---- apply control based on mode
    switch(mode)
    {
      case 0: // XY / RP
        x += dx;
        y += dy;

        roll  += dx;
        pitch += dy;
      break;

      case 1: // YZ / PY
        y += dx;
        z += dy;

        pitch += dx;
        yaw   += dy;
      break;

      case 2: // ZX / YR
        z += dx;
        x += dy;

        yaw  += dx;
        roll += dy;
      break;
    }

    // ---- read buttons
    bool currentA = digitalRead(2);
    bool currentB = digitalRead(3);

    if (lastButtonA == HIGH && currentA == LOW) {
      screen = !screen;
      lcd.clear();
    }

    if (lastButtonB == HIGH && currentB == LOW) {
      mode = (mode + 1) % 3;
    }

    lastButtonA = currentA;
    lastButtonB = currentB;

    // ---- draw
    if (screen == 0) {
      drawXYZ(lcd, x, y, z, pairXYZ[mode]);
    } 
    else {
      drawRPY(lcd, roll, pitch, yaw, pairRPY[mode]);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
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
  // Lokálne pole pre prevod z matice na primitívny typ
  float target_angles_array[6];
  
  while (1) {
    // 1. Aktualizácia cieľovej pozície `target_pos` a `target_rot`.
    // V budúcnosti tu budeš čítať premenné, ktoré ti prichádzajú napr. z TaskUI, z joysticku a podobne.
    // target_pos(0) += ... (napr. posun cez joystick)

    // 2. Riešenie inverznej kinematiky z existujúcej polohy
    // current_theta slúži ako vstup (odhad) a rovno do neho skočí vyriešený výsledok
    bool success = kinematics.SolveIK(
        target_pos, 
        target_rot, 
        current_theta, 
        tol_pos, 
        tol_ori, 
        lambda, 
        max_iter
    );

    // 3. Bezpečný zápis na na fyzické servá cez PCA9685
    if (success) {
        // Konverzia typu z Eigen matice na obyčajné pole plavucích radových čísel C++ 
        for(int i = 0; i < 6; i++) {
            target_angles_array[i] = current_theta(i);
        }

        // Zápis do Servo Actuatorov, funkcia vracia 'false' ak to presiahne konfigurované min/max uhly serv
        bool in_limits = servo_controller.writeAngles(target_angles_array, 6);
        
        if (!in_limits) {
            Serial.println("Výstraha: Vypočítané IK uhly prekračujú zadané limity serv! Preskakujem zápis.");
        }
    } else {
        Serial.println("IK nenašla riešenie: Zvolená poloha je pravdepodobne nedosiahnuteľná.");
    }

    // 4. Pauza medzi iteráciami, typicky 20 Hz (50 ms) alebo 50 Hz (20 ms). 
    // Nutné pre správne fungovanie FreeRTOS.
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void TaskSensor(void* pvParameters) {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  
  const uint32_t Ts_ms = 100;  // 100ms sample period
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
  rangingSensor.startContinuous();
  Serial.println("RangingSensor initialized");

  while (1) {
    MeasurementData measurement = rangingSensor.read(x, y, z, roll, pitch, yaw);
    
    Serial.print("Distance: ");
    Serial.print(measurement.distance_mm);
    Serial.print(" mm | Pose: (");
    Serial.print(measurement.pose.x, 3); Serial.print(", ");
    Serial.print(measurement.pose.y, 3); Serial.print(", ");
    Serial.print(measurement.pose.z, 3); Serial.println(")");

    if (measurement.valid) {
      // TODO: Store measurement with pose for map reconstruction
    }

    vTaskDelay(pdMS_TO_TICKS(Ts_ms));
  }

}

